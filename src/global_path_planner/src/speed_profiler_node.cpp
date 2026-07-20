// speed_profiler_node.cpp
// 판단 속도 프로파일 ROS 어댑터.
//   Sub: /selected_path (nav_msgs/Path, base_link) — 입력 경로(회피 반영된 최종 경로)
//        /obstacle_info (katri_msgs/ObstacleInfoArray) — ADAS 추종/정지 (Phase2)
//        /Ego_topic     (morai_msgs/EgoVehicleStatus)  — 자차 속도 (Phase2)
//   Pub: /selected_path_profiled (nav_msgs/Path) — 입력 경로 복사 + pose.position.z = 목표속도[kph]
// 속도 프로파일 계산은 순수 코어 gpp::SpeedProfiler 가 담당.

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/MarkerArray.h>
#include <katri_msgs/ObstacleInfoArray.h>
#include <morai_msgs/EgoVehicleStatus.h>

#include <algorithm>
#include <cmath>
#include <mutex>
#include <sstream>
#include <iomanip>
#include <string>
#include <vector>

#include "global_path_planner/speed_profiler.h"

class SpeedProfilerNode {
 public:
  SpeedProfilerNode() : nh_(), pnh_("~") {
    pnh_.param<std::string>("input_topic", input_topic_, "/selected_path");
    pnh_.param<std::string>("output_topic", output_topic_, "/selected_path_profiled");
    pnh_.param<std::string>("obstacle_topic", obstacle_topic_, "/obstacle_info");
    pnh_.param<std::string>("ego_topic", ego_topic_, "/Ego_topic");
    pnh_.param<double>("obstacle_timeout", obstacle_timeout_, 0.5);
    pnh_.param<double>("ego_timeout", ego_timeout_, 0.5);
    pnh_.param<bool>("publish_markers", publish_markers_, true);
    pnh_.param<double>("speed_height_scale", speed_height_scale_, 0.05);  // [m per kph] 스카이라인 높이
    pnh_.param<double>("rotary_timeout", rotary_timeout_, 0.5);           // /rotary/* stale[s] 넘으면 무시

    gpp::SpeedProfileConfig& c = planner_.cfg;
    pnh_.param("max_speed_kph", c.max_speed_kph, c.max_speed_kph);
    pnh_.param("lateral_accel_limit_mps2", c.lateral_accel_limit_mps2, c.lateral_accel_limit_mps2);
    pnh_.param("curve_speed_safety_factor", c.curve_speed_safety_factor, c.curve_speed_safety_factor);
    pnh_.param("minimum_curve_speed_kph", c.minimum_curve_speed_kph, c.minimum_curve_speed_kph);
    pnh_.param("brake_decel_mps2", c.brake_decel_mps2, c.brake_decel_mps2);
    pnh_.param("path_resolution_min", c.path_resolution_min, c.path_resolution_min);
    pnh_.param("curvature_smoothing_window_m", c.curvature_smoothing_window_m, c.curvature_smoothing_window_m);
    pnh_.param("use_following", c.use_following, c.use_following);
    pnh_.param("acc_headway_s", c.acc_headway_s, c.acc_headway_s);
    pnh_.param("acc_min_gap_m", c.acc_min_gap_m, c.acc_min_gap_m);
    pnh_.param("acc_gain_k", c.acc_gain_k, c.acc_gain_k);
    pnh_.param("acc_lane_gate_m", c.acc_lane_gate_m, c.acc_lane_gate_m);
    pnh_.param("acc_standoff_m", c.acc_standoff_m, c.acc_standoff_m);
    pnh_.param("use_obstacle_stop", c.use_obstacle_stop, c.use_obstacle_stop);
    pnh_.param("stop_distance_m", c.stop_distance_m, c.stop_distance_m);
    pnh_.param("v_static_max_mps", c.v_static_max_mps, c.v_static_max_mps);
    sanitizeConfig(c);

    sub_path_ = nh_.subscribe(input_topic_, 1, &SpeedProfilerNode::pathCb, this);
    sub_obs_ = nh_.subscribe(obstacle_topic_, 1, &SpeedProfilerNode::obsCb, this);
    sub_ego_ = nh_.subscribe(ego_topic_, 1, &SpeedProfilerNode::egoCb, this);
    sub_rstop_ = nh_.subscribe("/rotary/stop_wall_s", 1, &SpeedProfilerNode::rotaryStopCb, this);
    sub_rcap_ = nh_.subscribe("/rotary/speed_cap_kph", 1, &SpeedProfilerNode::rotaryCapCb, this);
    pub_path_ = nh_.advertise<nav_msgs::Path>(output_topic_, 1);
    pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("/speed_profile_markers", 1);

    ROS_INFO("[speed_profiler] ready. %s -> %s (z=target kph)", input_topic_.c_str(),
             output_topic_.c_str());
  }

 private:
  static void sanitizeConfig(gpp::SpeedProfileConfig& c) {
    auto pos = [](double& v, double lo, const char* n) {
      if (v < lo) { ROS_WARN("[speed_profiler] %s=%.3f < %.3f -> clamp", n, v, lo); v = lo; }
    };
    pos(c.max_speed_kph, 0.0, "max_speed_kph");
    pos(c.lateral_accel_limit_mps2, 0.1, "lateral_accel_limit_mps2");
    c.curve_speed_safety_factor = std::max(0.1, std::min(1.0, c.curve_speed_safety_factor));
    pos(c.minimum_curve_speed_kph, 0.0, "minimum_curve_speed_kph");
    pos(c.brake_decel_mps2, 0.1, "brake_decel_mps2");
    pos(c.path_resolution_min, 1e-3, "path_resolution_min");
    pos(c.curvature_smoothing_window_m, 1e-3, "curvature_smoothing_window_m");
    pos(c.acc_headway_s, 0.0, "acc_headway_s");
    pos(c.acc_min_gap_m, 0.0, "acc_min_gap_m");
    pos(c.acc_lane_gate_m, 0.0, "acc_lane_gate_m");
    pos(c.stop_distance_m, 0.0, "stop_distance_m");
    pos(c.v_static_max_mps, 0.0, "v_static_max_mps");
    if (c.min_points < 3) c.min_points = 3;
  }

  void obsCb(const katri_msgs::ObstacleInfoArray::ConstPtr& msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    latest_obs_ = *msg;
    last_obs_time_ = ros::Time::now();
    has_obs_ = true;
  }
  void rotaryStopCb(const std_msgs::Float64::ConstPtr& msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    rotary_stop_ = msg->data; rotary_stop_time_ = ros::Time::now(); has_rotary_stop_ = true;
  }
  void rotaryCapCb(const std_msgs::Float64::ConstPtr& msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    rotary_cap_ = msg->data; rotary_cap_time_ = ros::Time::now(); has_rotary_cap_ = true;
  }
  void egoCb(const morai_msgs::EgoVehicleStatus::ConstPtr& msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    ego_speed_mps_ = std::hypot(msg->velocity.x, msg->velocity.y);
    last_ego_time_ = ros::Time::now();
    has_ego_ = true;
  }

  void pathCb(const nav_msgs::Path::ConstPtr& msg) {
    const ros::Time now = ros::Time::now();
    if (msg->poses.empty()) return;  // 빈 경로는 발행 안 함

    std::vector<gpp::Obstacle> obs;
    double ego_speed = 0.0;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      const bool obs_fresh = has_obs_ && (now - last_obs_time_).toSec() <= obstacle_timeout_;
      if (obs_fresh) {
        obs.reserve(latest_obs_.obstacles.size());
        for (const auto& o : latest_obs_.obstacles) {
          gpp::Obstacle g;
          g.id = o.id; g.x = o.x_m; g.y = o.y_m; g.speed = o.speed_mps;
          g.width = o.width_m; g.length = o.length_m;
          obs.push_back(g);
        }
      }
      if (has_ego_ && (now - last_ego_time_).toSec() <= ego_timeout_) ego_speed = ego_speed_mps_;
    }

    std::vector<gpp::Vec2> P;
    P.reserve(msg->poses.size());
    for (const auto& ps : msg->poses) P.emplace_back(ps.pose.position.x, ps.pose.position.y);

    gpp::SpeedProfileResult res = planner_.plan(P, obs, ego_speed);

    // 외부 제약(로터리 등) 적용: fresh 한 stop_wall/cap 을 프로파일에 반영 + 제동 재전파
    double rstop = -1.0, rcap = -1.0;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (has_rotary_stop_ && (now - rotary_stop_time_).toSec() <= rotary_timeout_) rstop = rotary_stop_;
      if (has_rotary_cap_ && (now - rotary_cap_time_).toSec() <= rotary_timeout_) rcap = rotary_cap_;
    }
    if (rstop >= 0.0 || rcap >= 0.0)
      planner_.applyExternalConstraints(P, res.speed_kph, rstop, rcap);

    // 입력 경로 복사 + z 에 목표속도[kph]
    nav_msgs::Path out = *msg;
    if (res.speed_kph.size() == out.poses.size())
      for (std::size_t i = 0; i < out.poses.size(); ++i)
        out.poses[i].pose.position.z = res.speed_kph[i];
    pub_path_.publish(out);

    if (publish_markers_) publishSpeedMarkers(out.header, P, res);

    ROS_INFO_THROTTLE(1.0, "[speed_profiler] min=%.1fkph follow=%.1fkph lead=%d pts=%zu",
                      res.min_kph, res.v_follow_kph, res.lead_id, out.poses.size());
  }

  // 속도 프로파일 시각화: (1) 속도별 색상 경로, (2) 속도=높이 스카이라인, (3) 요약/최저속도 텍스트.
  void publishSpeedMarkers(const std_msgs::Header& hdr, const std::vector<gpp::Vec2>& P,
                           const gpp::SpeedProfileResult& res) {
    if (P.size() != res.speed_kph.size() || P.size() < 2) return;
    const double vmax = std::max(1.0, planner_.cfg.max_speed_kph);
    visualization_msgs::MarkerArray arr;
    { visualization_msgs::Marker c; c.header = hdr; c.ns = "speed_profile";
      c.action = visualization_msgs::Marker::DELETEALL; arr.markers.push_back(c); }

    // 속도 → 색 (빨강 느림 → 노랑 → 초록 빠름)
    auto colorFor = [&](double kph) {
      std_msgs::ColorRGBA col; col.a = 1.0;
      const double t = std::max(0.0, std::min(1.0, kph / vmax));
      col.r = std::min(1.0, 2.0 * (1.0 - t));
      col.g = std::min(1.0, 2.0 * t);
      col.b = 0.0;
      return col;
    };
    auto mk = [&](int id, int type) {
      visualization_msgs::Marker m; m.header = hdr; m.ns = "speed_profile";
      m.id = id; m.type = type; m.action = visualization_msgs::Marker::ADD;
      m.lifetime = ros::Duration(0.5); m.pose.orientation.w = 1.0;
      return m;
    };

    // (1) 속도별 색상 경로 (경로 위, 얇은 라인)
    visualization_msgs::Marker line = mk(0, visualization_msgs::Marker::LINE_STRIP);
    line.scale.x = 0.15;
    // (2) 속도 스카이라인 (z = 속도*scale)
    visualization_msgs::Marker sky = mk(1, visualization_msgs::Marker::LINE_STRIP);
    sky.scale.x = 0.08;
    std::size_t min_i = 0;
    for (std::size_t i = 0; i < P.size(); ++i) {
      const double kph = res.speed_kph[i];
      if (kph < res.speed_kph[min_i]) min_i = i;
      std_msgs::ColorRGBA col = colorFor(kph);
      geometry_msgs::Point p; p.x = P[i].x; p.y = P[i].y; p.z = 0.15;
      line.points.push_back(p); line.colors.push_back(col);
      geometry_msgs::Point ps; ps.x = P[i].x; ps.y = P[i].y; ps.z = kph * speed_height_scale_;
      sky.points.push_back(ps); sky.colors.push_back(col);
    }
    arr.markers.push_back(line);
    arr.markers.push_back(sky);

    // (3) 최저속도 지점 텍스트
    {
      visualization_msgs::Marker t = mk(2, visualization_msgs::Marker::TEXT_VIEW_FACING);
      t.pose.position.x = P[min_i].x; t.pose.position.y = P[min_i].y;
      t.pose.position.z = res.speed_kph[min_i] * speed_height_scale_ + 0.5;
      t.scale.z = 0.5; t.color.a = 1.0; t.color.r = 1.0; t.color.g = 0.3; t.color.b = 0.3;
      std::ostringstream ss; ss << std::fixed << std::setprecision(1) << "min " << res.speed_kph[min_i] << " kph";
      t.text = ss.str(); arr.markers.push_back(t);
    }
    // (4) 요약 텍스트 (차량 원점 위)
    {
      visualization_msgs::Marker t = mk(3, visualization_msgs::Marker::TEXT_VIEW_FACING);
      t.pose.position.x = 0.0; t.pose.position.y = 0.0; t.pose.position.z = 3.5;
      t.scale.z = 0.5; t.color.a = 1.0; t.color.r = t.color.g = t.color.b = 1.0;
      std::ostringstream ss; ss << std::fixed << std::setprecision(1)
         << "SPEED  min=" << res.min_kph << "kph";
      if (res.v_follow_kph >= 0.0) ss << "  follow=" << res.v_follow_kph << "kph(id=" << res.lead_id << ")";
      else ss << "  follow=off";
      t.text = ss.str(); arr.markers.push_back(t);
    }
    pub_markers_.publish(arr);
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_path_, sub_obs_, sub_ego_, sub_rstop_, sub_rcap_;
  ros::Publisher pub_path_, pub_markers_;
  double rotary_stop_{-1.0}, rotary_cap_{-1.0}, rotary_timeout_{0.5};
  ros::Time rotary_stop_time_, rotary_cap_time_;
  bool has_rotary_stop_{false}, has_rotary_cap_{false};
  std::string input_topic_, output_topic_, obstacle_topic_, ego_topic_;
  double obstacle_timeout_{0.5}, ego_timeout_{0.5};
  bool publish_markers_{true};
  double speed_height_scale_{0.05};

  std::mutex mtx_;
  katri_msgs::ObstacleInfoArray latest_obs_;
  double ego_speed_mps_{0.0};
  ros::Time last_obs_time_, last_ego_time_;
  bool has_obs_{false}, has_ego_{false};

  gpp::SpeedProfiler planner_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "speed_profiler");
  SpeedProfilerNode node;
  ros::spin();
  return 0;
}
