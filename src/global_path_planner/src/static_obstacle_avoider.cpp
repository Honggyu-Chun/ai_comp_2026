// static_obstacle_avoider.cpp
// 정적 장애물 회피(Path Shifting) ROS 어댑터 노드.
//   Sub: /local_path1        (nav_msgs/Path, base_link)
//   Sub: /obstacle_info      (katri_msgs/ObstacleInfoArray, base_link, SI)
//   Pub: /avoid_path         (nav_msgs/Path, base_link)  ← 회피 경로(무장애물 시 passthrough)
//   Pub: /avoid_debug_markers(visualization_msgs/MarkerArray)
// 계획 알고리즘은 순수 코어 gpp::PathShiftPlanner 가 담당(ROS 무관, 단위테스트 가능).

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/MarkerArray.h>
#include <katri_msgs/ObstacleInfoArray.h>

#include <algorithm>
#include <cmath>
#include <mutex>
#include <sstream>
#include <iomanip>
#include <string>
#include <utility>
#include <vector>

#include "global_path_planner/path_shift_planner.h"

class StaticObstacleAvoider {
 public:
  StaticObstacleAvoider() : nh_(), pnh_("~") {
    pnh_.param<std::string>("path_topic", path_topic_, "/local_path1");
    pnh_.param<std::string>("obstacle_topic", obstacle_topic_, "/obstacle_info");
    pnh_.param<std::string>("output_topic", output_topic_, "/avoid_path");
    pnh_.param<double>("timer_rate_hz", timer_rate_hz_, 20.0);
    pnh_.param<bool>("publish_debug_markers", publish_debug_markers_, true);
    pnh_.param<double>("marker_rate_hz", marker_rate_hz_, 50.0);
    // 입력 staleness: 이 시간 넘게 갱신 안 되면 유령 장애물/stale 경로 방지
    pnh_.param<double>("path_timeout", path_timeout_, 0.3);
    pnh_.param<double>("obstacle_timeout", obstacle_timeout_, 0.5);
    pnh_.param<std::string>("expected_frame", expected_frame_, "base_link");
    // 차량 footprint 마커(아이오닉5). base_link(0,0,0)=후륜축 중앙 기준.
    pnh_.param<double>("viz_veh_length", viz_len_, 4.635);
    pnh_.param<double>("viz_veh_width", viz_wid_, 1.892);
    pnh_.param<double>("viz_veh_height", viz_hgt_, 1.605);
    pnh_.param<double>("viz_rear_overhang", viz_rear_, 0.79);  // 후륜축→뒷범퍼

    // 코어 파라미터(gpp::PathShiftConfig) 주입 — 기본값은 config 구조체와 동일.
    gpp::PathShiftConfig& c = planner_.cfg;
    pnh_.param("veh_half_width", c.veh_half_width, c.veh_half_width);
    pnh_.param("margin_obs", c.margin_obs, c.margin_obs);
    pnh_.param("x_front_min", c.x_front_min, c.x_front_min);
    pnh_.param("x_plan_max", c.x_plan_max, c.x_plan_max);
    pnh_.param("s_plan_max", c.s_plan_max, c.s_plan_max);
    pnh_.param("v_static_max", c.v_static_max, c.v_static_max);
    pnh_.param("side_deadband", c.side_deadband, c.side_deadband);
    pnh_.param("max_shift", c.max_shift, c.max_shift);
    pnh_.param("prefer_right", c.prefer_right, c.prefer_right);
    pnh_.param("right_switch_side", c.right_switch_side, c.right_switch_side);
    pnh_.param("avoid_offset", c.avoid_offset, c.avoid_offset);
    pnh_.param("lane_half_width", c.lane_half_width, c.lane_half_width);
    pnh_.param("pre_dist", c.pre_dist, c.pre_dist);
    pnh_.param("post_dist", c.post_dist, c.post_dist);
    pnh_.param("ramp_in_len", c.ramp_in_len, c.ramp_in_len);
    pnh_.param("ramp_out_len", c.ramp_out_len, c.ramp_out_len);
    pnh_.param("soften_margin", c.soften_margin, c.soften_margin);
    pnh_.param("cluster_s_eps", c.cluster_s_eps, c.cluster_s_eps);
    pnh_.param("solver_iters", c.solver_iters, c.solver_iters);
    pnh_.param("smooth_window", c.smooth_window, c.smooth_window);
    pnh_.param("max_doffset_ds", c.max_doffset_ds, c.max_doffset_ds);
    pnh_.param("hard_long_margin", c.hard_long_margin, c.hard_long_margin);
    pnh_.param("filter_k_avoid", c.filter_k_avoid, c.filter_k_avoid);
    pnh_.param("filter_k_return", c.filter_k_return, c.filter_k_return);
    pnh_.param("obs_radius_scale", c.obs_radius_scale, c.obs_radius_scale);
    pnh_.param("extra_clearance", c.extra_clearance, c.extra_clearance);
    pnh_.param("t_return_hold", c.t_return_hold, c.t_return_hold);
    pnh_.param("side_lock_sec", c.side_lock_sec, c.side_lock_sec);
    pnh_.param("track_lost_sec", c.track_lost_sec, c.track_lost_sec);
    pnh_.param("r_obs_min", c.r_obs_min, c.r_obs_min);
    pnh_.param("r_obs_max", c.r_obs_max, c.r_obs_max);
    pnh_.param("min_points", c.min_points, c.min_points);
    pnh_.param("min_path_len", c.min_path_len, c.min_path_len);

    sanitizeConfig(c);  // 음수/역전 등 비정상 파라미터 방어

    sub_path_ = nh_.subscribe(path_topic_, 1, &StaticObstacleAvoider::pathCb, this);
    sub_obs_ = nh_.subscribe(obstacle_topic_, 1, &StaticObstacleAvoider::obsCb, this);
    pub_path_ = nh_.advertise<nav_msgs::Path>(output_topic_, 1);
    pub_active_ = nh_.advertise<std_msgs::Bool>("/avoid_active", 1);
    pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("/avoid_debug_markers", 1);
    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, timer_rate_hz_)),
                             &StaticObstacleAvoider::timerCb, this);
    if (publish_debug_markers_)
      marker_timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, marker_rate_hz_)),
                                      &StaticObstacleAvoider::markerTimerCb, this);

    ROS_INFO("[static_obstacle_avoider] ready. path=%s obs=%s out=%s",
             path_topic_.c_str(), obstacle_topic_.c_str(), output_topic_.c_str());
  }

 private:
  // 비정상 파라미터 방어(음수/역전). 범위 밖이면 경고 후 클램프.
  static void sanitizeConfig(gpp::PathShiftConfig& c) {
    auto pos = [](double& v, double lo, const char* n) {
      if (v < lo) { ROS_WARN("[avoider] param %s=%.3f < %.3f -> clamped", n, v, lo); v = lo; }
    };
    pos(c.veh_half_width, 0.0, "veh_half_width");
    pos(c.margin_obs, 0.0, "margin_obs");
    pos(c.max_shift, 0.0, "max_shift");
    pos(c.pre_dist, 0.0, "pre_dist");
    pos(c.post_dist, 0.0, "post_dist");
    pos(c.ramp_in_len, 0.0, "ramp_in_len");
    pos(c.ramp_out_len, 0.0, "ramp_out_len");
    pos(c.soften_margin, 0.0, "soften_margin");
    pos(c.max_doffset_ds, 1e-3, "max_doffset_ds");
    pos(c.v_static_max, 0.0, "v_static_max");
    pos(c.obs_radius_scale, 0.0, "obs_radius_scale");
    pos(c.extra_clearance, 0.0, "extra_clearance");
    pos(c.side_lock_sec, 0.0, "side_lock_sec");
    pos(c.track_lost_sec, 0.0, "track_lost_sec");
    pos(c.t_return_hold, 0.0, "t_return_hold");
    pos(c.right_switch_side, 0.0, "right_switch_side");
    pos(c.avoid_offset, 0.0, "avoid_offset");
    pos(c.lane_half_width, 0.0, "lane_half_width");
    if (c.solver_iters < 1) { ROS_WARN("[avoider] solver_iters<1 -> 1"); c.solver_iters = 1; }
    if (c.smooth_window < 0) { ROS_WARN("[avoider] smooth_window<0 -> 0"); c.smooth_window = 0; }
    if (c.min_points < 2) c.min_points = 2;
    c.filter_k_avoid = std::max(0.0, std::min(1.0, c.filter_k_avoid));
    c.filter_k_return = std::max(0.0, std::min(1.0, c.filter_k_return));
    if (c.r_obs_min > c.r_obs_max) {
      ROS_WARN("[avoider] r_obs_min(%.2f) > r_obs_max(%.2f) -> swapped", c.r_obs_min, c.r_obs_max);
      std::swap(c.r_obs_min, c.r_obs_max);
    }
  }

  void pathCb(const nav_msgs::Path::ConstPtr& msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    latest_path_ = *msg;
    last_path_time_ = ros::Time::now();
    has_path_ = true;
  }
  void obsCb(const katri_msgs::ObstacleInfoArray::ConstPtr& msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    latest_obs_ = *msg;
    last_obs_time_ = ros::Time::now();
    has_obs_ = true;
  }

  void publishActive(bool a) {
    std_msgs::Bool m;
    m.data = a;
    pub_active_.publish(m);
  }
  static double maxAbs(const std::vector<double>& v) {
    double m = 0.0;
    for (double x : v) m = std::max(m, std::abs(x));
    return m;
  }

  void timerCb(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    if (now.isZero()) return;  // sim time: /clock 미도착 (side-lock/FSM 타이밍 오염 방지)

    nav_msgs::Path path;
    katri_msgs::ObstacleInfoArray obs_msg;
    ros::Time path_time, obs_time;
    bool have_path, have_obs;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      have_path = has_path_;
      have_obs = has_obs_;
      path = latest_path_;
      obs_msg = latest_obs_;
      path_time = last_path_time_;
      obs_time = last_obs_time_;
    }

    // 경로 없음 → 발행 안 함(선택기가 avoid_timeout 으로 lane fallback)
    if (!have_path) {
      ROS_WARN_THROTTLE(2.0, "[avoider] waiting for %s ...", path_topic_.c_str());
      publishActive(false);
      return;
    }
    // 경로 stale → 발행 중단 + planner 리셋(stale 경로 재발행/유령 회피 방지)
    if ((now - path_time).toSec() > path_timeout_) {
      ROS_WARN_THROTTLE(1.0, "[avoider] %s stale %.2fs -> hold", path_topic_.c_str(),
                        (now - path_time).toSec());
      planner_.reset();
      publishActive(false);
      return;
    }
    // 빈 경로 → 발행 안 함(빈 selected_path 전파 방지)
    if (path.poses.empty()) {
      ROS_WARN_THROTTLE(1.0, "[avoider] empty %s -> skip", path_topic_.c_str());
      publishActive(false);
      return;
    }
    if (!path.header.frame_id.empty() && path.header.frame_id != expected_frame_) {
      ROS_WARN_THROTTLE(2.0, "[avoider] path frame '%s' != expected '%s'",
                        path.header.frame_id.c_str(), expected_frame_.c_str());
    }

    // 장애물 stale → 무시(빈 것으로 취급 → 유령 장애물 무한 회피 방지)
    const bool obs_fresh = have_obs && (now - obs_time).toSec() <= obstacle_timeout_;
    if (have_obs && !obs_fresh)
      ROS_WARN_THROTTLE(1.0, "[avoider] %s stale -> ignoring obstacles", obstacle_topic_.c_str());

    std::vector<gpp::Vec2> P;
    P.reserve(path.poses.size());
    for (const auto& ps : path.poses) P.emplace_back(ps.pose.position.x, ps.pose.position.y);

    std::vector<gpp::Obstacle> obs;
    if (obs_fresh) {
      obs.reserve(obs_msg.obstacles.size());
      for (const auto& o : obs_msg.obstacles) {
        if (o.is_pedestrian) continue;  // 정적 게이팅(speed)은 코어가 처리
        gpp::Obstacle g;
        g.id = o.id; g.x = o.x_m; g.y = o.y_m; g.speed = o.speed_mps;
        g.width = o.width_m; g.length = o.length_m;
        obs.push_back(g);
      }
    }

    gpp::PathShiftResult res = planner_.plan(P, obs, now.toSec());

    // 출력: 입력 헤더(frame/stamp) 보존
    nav_msgs::Path out = path;
    if (res.valid && res.offset.size() == P.size()) {
      std::vector<gpp::Vec2> shifted = gpp::PathShiftPlanner::applyOffset(P, res.offset);
      for (std::size_t i = 0; i < out.poses.size() && i < shifted.size(); ++i) {
        out.poses[i].pose.position.x = shifted[i].x;
        out.poses[i].pose.position.y = shifted[i].y;
      }
    }
    pub_path_.publish(out);

    // avoid_active: 실제 shifting(AVOID/RETURN) 중일 때만 true.
    // NORMAL 에서는 false → selector 가 자기 로직(동적장애물 /moving_obs, 차선변경)을 계속 수행.
    publishActive(res.valid && res.state != gpp::PlannerState::NORMAL);

    if (res.blocked)
      ROS_WARN_THROTTLE(0.5, "[avoider] BLOCKED: passage narrower than max_shift(%.2f) -> needs STOP",
                        planner_.cfg.max_shift);
    ROS_INFO_THROTTLE(1.0, "[avoider] state=%s cand=%zu max|off|=%.2f%s",
                      res.state == gpp::PlannerState::NORMAL ? "NORMAL"
                          : res.state == gpp::PlannerState::AVOID ? "AVOID" : "RETURN",
                      res.candidates.size(), maxAbs(res.offset), res.blocked ? " [BLOCKED]" : "");

    if (publish_debug_markers_) buildDebugMarkers(out.header, res, P);  // 캐시만; 발행은 marker_timer_
  }

  // 시각화 마커를 독립적으로 marker_rate_hz(기본 50Hz)로 발행. 내용은 계획 루프가 갱신.
  void markerTimerCb(const ros::TimerEvent&) {
    if (!publish_debug_markers_ || !have_markers_) return;
    const ros::Time now = ros::Time::now();
    for (auto& m : last_markers_.markers) m.header.stamp = now;  // stamp 갱신(TF 정합)
    pub_markers_.publish(last_markers_);
  }

  void buildDebugMarkers(const std_msgs::Header& hdr, const gpp::PathShiftResult& res,
                         const std::vector<gpp::Vec2>& P) {
    visualization_msgs::MarkerArray arr;
    auto mk = [&](int id, int type) {
      visualization_msgs::Marker m;
      m.header = hdr;
      m.ns = "avoid_debug";
      m.id = id;
      m.type = type;
      m.action = visualization_msgs::Marker::ADD;
      m.lifetime = ros::Duration(0.5);
      m.pose.orientation.w = 1.0;
      return m;
    };
    // 매 프레임 잔상 제거
    { visualization_msgs::Marker c; c.header = hdr; c.ns = "avoid_debug";
      c.action = visualization_msgs::Marker::DELETEALL; arr.markers.push_back(c); }

    // === 차량 footprint (아이오닉5). base_link(0,0,0)=후륜축 중앙 → 차체 중심은 앞쪽으로 offset ===
    {
      visualization_msgs::Marker v = mk(50, visualization_msgs::Marker::CUBE);
      v.pose.position.x = viz_len_ / 2.0 - viz_rear_;  // 차체 중심 x (후륜축 기준)
      v.pose.position.y = 0.0;
      v.pose.position.z = viz_hgt_ / 2.0;
      v.scale.x = viz_len_; v.scale.y = viz_wid_; v.scale.z = viz_hgt_;
      v.color.a = 0.20; v.color.r = 0.55; v.color.g = 0.75; v.color.b = 0.95;  // 반투명 하늘색
      arr.markers.push_back(v);
    }
    { // 후륜축 원점(0,0,0) 표시(빨강 구)
      visualization_msgs::Marker o = mk(51, visualization_msgs::Marker::SPHERE);
      o.pose.position.x = 0.0; o.pose.position.y = 0.0; o.pose.position.z = 0.15;
      o.scale.x = o.scale.y = o.scale.z = 0.3;
      o.color.a = 1.0; o.color.r = 1.0; o.color.g = 0.0; o.color.b = 0.0;
      arr.markers.push_back(o);
      visualization_msgs::Marker ot = mk(52, visualization_msgs::Marker::TEXT_VIEW_FACING);
      ot.pose.position.x = 0.0; ot.pose.position.y = 0.0; ot.pose.position.z = 0.6;
      ot.scale.z = 0.3; ot.color.a = 1.0; ot.color.r = ot.color.g = ot.color.b = 1.0;
      ot.text = "rear axle (0,0,0)";
      arr.markers.push_back(ot);
    }

    const double lane_hw = planner_.cfg.lane_half_width;
    const double veh_hw = planner_.cfg.veh_half_width;
    const double lane_room = std::max(0.0, lane_hw - veh_hw);

    // 차선 경계선 (경로를 ±lane_half_width 만큼 법선 이동 = applyOffset 재사용)
    if (P.size() >= 2) {
      std::vector<double> plo(P.size(), +lane_hw), pri(P.size(), -lane_hw);
      auto edgeL = gpp::PathShiftPlanner::applyOffset(P, plo);
      auto edgeR = gpp::PathShiftPlanner::applyOffset(P, pri);
      for (int side = 0; side < 2; ++side) {
        visualization_msgs::Marker ln = mk(1 + side, visualization_msgs::Marker::LINE_STRIP);
        ln.scale.x = 0.08;
        ln.color.a = 0.9; ln.color.r = 1.0; ln.color.g = 1.0; ln.color.b = 0.0;  // 노랑 차선
        const auto& E = (side == 0) ? edgeL : edgeR;
        for (const auto& p : E) {
          geometry_msgs::Point gp; gp.x = p.x; gp.y = p.y; gp.z = 0.05;
          ln.points.push_back(gp);
        }
        arr.markers.push_back(ln);
      }
    }

    // 상태 텍스트
    {
      visualization_msgs::Marker m = mk(0, visualization_msgs::Marker::TEXT_VIEW_FACING);
      m.pose.position.x = 2.0; m.pose.position.z = 2.0; m.scale.z = 0.6;
      m.color.a = 1.0; m.color.r = 1.0; m.color.g = 1.0; m.color.b = 1.0;
      double max_abs = 0.0;
      for (double v : res.offset) max_abs = std::max(max_abs, std::abs(v));
      std::ostringstream ss;
      ss << (res.state == gpp::PlannerState::NORMAL ? "NORMAL"
             : res.state == gpp::PlannerState::AVOID ? "AVOID" : "RETURN")
         << (res.blocked ? " [BLOCKED]" : "") << "  cand=" << res.candidates.size()
         << "  |off|=" << std::fixed << std::setprecision(2) << max_abs
         << "  lane_room=" << lane_room << "m";
      m.text = ss.str();
      arr.markers.push_back(m);
    }

    // 후보별: d_block 안전영역(원판) + 판단 근거 텍스트
    int cnt = 0;
    for (const auto& c : res.candidates) {
      if (cnt >= 10) break;
      const bool right = (c.pass_sign < 0);

      // d_block 원판(CYLINDER) — 이 반경 안으로 경로가 들어오면 충돌
      visualization_msgs::Marker disk = mk(30 + cnt, visualization_msgs::Marker::CYLINDER);
      disk.pose.position.x = c.pos_x; disk.pose.position.y = c.pos_y; disk.pose.position.z = 0.05;
      disk.scale.x = disk.scale.y = 2.0 * c.d_block; disk.scale.z = 0.02;
      disk.color.a = 0.20;
      if (right) { disk.color.r = 0.2; disk.color.g = 0.6; disk.color.b = 1.0; }  // 우측=파랑
      else { disk.color.r = 1.0; disk.color.g = 0.4; disk.color.b = 0.1; }        // 좌측=주황
      arr.markers.push_back(disk);

      // 장애물 중심 구
      visualization_msgs::Marker m = mk(10 + cnt, visualization_msgs::Marker::SPHERE);
      m.pose.position.x = c.pos_x; m.pose.position.y = c.pos_y; m.pose.position.z = 0.4;
      m.scale.x = m.scale.y = m.scale.z = 0.35;
      m.color.a = 0.95;
      if (right) { m.color.r = 0.2; m.color.g = 0.6; m.color.b = 1.0; }
      else { m.color.r = 1.0; m.color.g = 0.4; m.color.b = 0.1; }
      arr.markers.push_back(m);

      // 판단 근거 텍스트: 방향 + right/left_need vs lane_room + 이유
      visualization_msgs::Marker t = mk(110 + cnt, visualization_msgs::Marker::TEXT_VIEW_FACING);
      t.pose.position.x = c.pos_x; t.pose.position.y = c.pos_y; t.pose.position.z = 1.2;
      t.scale.z = 0.35; t.color.a = 1.0;
      if (right) { t.color.r = 0.4; t.color.g = 0.8; t.color.b = 1.0; }
      else { t.color.r = 1.0; t.color.g = 0.6; t.color.b = 0.3; }
      std::ostringstream ss;
      ss << std::fixed << std::setprecision(2)
         << (right ? "-> RIGHT" : "-> LEFT ") << "  " << c.reason << "\n"
         << "id=" << c.id << " side=" << c.side_value << " clr=" << c.d_block << "\n"
         << "switch to LEFT when side < " << (-planner_.cfg.right_switch_side);
      if (c.cluster_barrier) ss << " [barrier]";
      t.text = ss.str();
      arr.markers.push_back(t);
      cnt++;
    }
    last_markers_ = arr;      // 캐시 (marker_timer_ 가 발행)
    have_markers_ = true;
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_path_, sub_obs_;
  ros::Publisher pub_path_, pub_active_, pub_markers_;
  ros::Timer timer_;

  std::string path_topic_, obstacle_topic_, output_topic_, expected_frame_;
  double timer_rate_hz_{20.0};
  double path_timeout_{0.3}, obstacle_timeout_{0.5};
  bool publish_debug_markers_{true};
  double marker_rate_hz_{50.0};
  double viz_len_{4.635}, viz_wid_{1.892}, viz_hgt_{1.605}, viz_rear_{0.79};
  ros::Timer marker_timer_;
  visualization_msgs::MarkerArray last_markers_;  // 계획 루프가 갱신, 마커 타이머가 발행
  bool have_markers_{false};

  std::mutex mtx_;
  nav_msgs::Path latest_path_;
  katri_msgs::ObstacleInfoArray latest_obs_;
  ros::Time last_path_time_, last_obs_time_;
  bool has_path_{false}, has_obs_{false};

  gpp::PathShiftPlanner planner_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "static_obstacle_avoider");
  StaticObstacleAvoider node;
  ros::spin();
  return 0;
}
