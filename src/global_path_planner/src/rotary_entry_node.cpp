// rotary_entry_node.cpp
// 로터리 진입 판단 ROS 어댑터.
//   Sub: /odometry/filtered(nav_msgs/Odometry, EKF 융합, UTM/map 월드), /obstacle_info(base_link 상대)
//   Pub: /rotary/stop_wall_s(Float64,m), /rotary/speed_cap_kph(Float64,kph), /rotary/debug_markers
//   장애물을 자차 pose로 월드 복원 → 순수 코어 gpp::RotaryGate 로 진입 판단.
//   프레임: /odometry/filtered 는 /gps_utm_odom(UTM)+IMU EKF 융합 → 로터리 기하도 UTM 으로 측정할 것.

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/MarkerArray.h>
#include <katri_msgs/ObstacleInfoArray.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

#include <algorithm>
#include <cmath>
#include <mutex>
#include <sstream>
#include <iomanip>
#include <string>
#include <vector>

#include "global_path_planner/rotary_gate.h"

class RotaryEntryNode {
 public:
  RotaryEntryNode() : nh_(), pnh_("~") {
    pnh_.param<std::string>("ego_topic", ego_topic_, "/odometry/filtered");
    pnh_.param<std::string>("obstacle_topic", obstacle_topic_, "/obstacle_info");
    pnh_.param<double>("timer_rate_hz", timer_rate_hz_, 30.0);
    pnh_.param<double>("ego_timeout", ego_timeout_, 0.5);
    pnh_.param<bool>("publish_markers", publish_markers_, true);

    gpp::RotaryGateConfig& c = gate_.cfg;
    pnh_.param("center_x", c.center_x, c.center_x);
    pnh_.param("center_y", c.center_y, c.center_y);
    pnh_.param("ring_radius_m", c.ring_radius_m, c.ring_radius_m);
    pnh_.param("ring_band_m", c.ring_band_m, c.ring_band_m);
    pnh_.param("theta_entry_rad", c.theta_entry_rad, c.theta_entry_rad);
    pnh_.param("theta_exit_rad", c.theta_exit_rad, c.theta_exit_rad);
    pnh_.param("dir", c.dir, c.dir);
    pnh_.param("s_stopline_to_conflict_m", c.s_stopline_to_conflict_m, c.s_stopline_to_conflict_m);
    pnh_.param("a_launch_mps2", c.a_launch_mps2, c.a_launch_mps2);
    pnh_.param("v_ring_kph", c.v_ring_kph, c.v_ring_kph);
    pnh_.param("ego_length_m", c.ego_length_m, c.ego_length_m);
    pnh_.param("tau_lead_s", c.tau_lead_s, c.tau_lead_s);
    pnh_.param("tau_lag_s", c.tau_lag_s, c.tau_lag_s);
    pnh_.param("v_agent_min_mps", c.v_agent_min_mps, c.v_agent_min_mps);
    pnh_.param("go_confirm_frames", c.go_confirm_frames, c.go_confirm_frames);
    pnh_.param("deadlock_timeout_s", c.deadlock_timeout_s, c.deadlock_timeout_s);
    pnh_.param("relaxed_scale", c.relaxed_scale, c.relaxed_scale);
    pnh_.param("activate_dist_m", c.activate_dist_m, c.activate_dist_m);
    pnh_.param("commit_dist_m", c.commit_dist_m, c.commit_dist_m);
    pnh_.param("approach_cap_kph", c.approach_cap_kph, c.approach_cap_kph);
    pnh_.param("allow_rearm", c.allow_rearm, c.allow_rearm);
    sanitize(c);

    sub_ego_ = nh_.subscribe(ego_topic_, 1, &RotaryEntryNode::egoCb, this);
    sub_obs_ = nh_.subscribe(obstacle_topic_, 1, &RotaryEntryNode::obsCb, this);
    pub_stop_ = nh_.advertise<std_msgs::Float64>("/rotary/stop_wall_s", 1);
    pub_cap_ = nh_.advertise<std_msgs::Float64>("/rotary/speed_cap_kph", 1);
    pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("/rotary/debug_markers", 1);
    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, timer_rate_hz_)),
                             &RotaryEntryNode::timerCb, this);
    ROS_INFO("[rotary_entry] ready. ego=%s obs=%s C=(%.1f,%.1f) R=%.1f dir=%d",
             ego_topic_.c_str(), obstacle_topic_.c_str(), c.center_x, c.center_y, c.ring_radius_m, c.dir);
  }

 private:
  static void sanitize(gpp::RotaryGateConfig& c) {
    c.ring_radius_m = std::max(0.5, c.ring_radius_m);
    c.ring_band_m = std::max(0.1, c.ring_band_m);
    c.a_launch_mps2 = std::max(0.1, c.a_launch_mps2);
    c.v_ring_kph = std::max(1.0, c.v_ring_kph);
    c.s_stopline_to_conflict_m = std::max(0.0, c.s_stopline_to_conflict_m);
    c.go_confirm_frames = std::max(1, c.go_confirm_frames);
    c.relaxed_scale = std::max(0.1, std::min(1.0, c.relaxed_scale));
    if (c.dir >= 0) c.dir = +1; else c.dir = -1;
  }

  void egoCb(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    ego_.x = msg->pose.pose.position.x;                       // UTM x
    ego_.y = msg->pose.pose.position.y;                       // UTM y
    ego_.yaw_rad = tf::getYaw(msg->pose.pose.orientation);    // ENU yaw[rad]
    ego_.v_mps = std::abs(msg->twist.twist.linear.x);         // 전방속도[m/s]
    last_ego_time_ = ros::Time::now();
    has_ego_ = true;
  }
  void obsCb(const katri_msgs::ObstacleInfoArray::ConstPtr& msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    latest_obs_ = *msg;
    has_obs_ = true;
  }

  void timerCb(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    if (now.isZero()) return;
    gpp::EgoState ego;
    katri_msgs::ObstacleInfoArray obs;
    bool ego_fresh;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      ego = ego_;
      obs = latest_obs_;
      ego_fresh = has_ego_ && (now - last_ego_time_).toSec() <= ego_timeout_;
    }
    if (!ego_fresh) {
      ROS_WARN_THROTTLE(2.0, "[rotary_entry] waiting for %s ...", ego_topic_.c_str());
      return;  // 발행 보류 → speed_profiler 타임아웃이 해제
    }

    // 장애물 base_link → 월드 복원 (obstacle_info_publisher 회전의 역변환)
    const double c_ = std::cos(ego.yaw_rad), s_ = std::sin(ego.yaw_rad);
    std::vector<gpp::AgentState> agents;
    agents.reserve(obs.obstacles.size());
    for (const auto& o : obs.obstacles) {
      if (o.is_pedestrian) continue;  // 보행자 제외
      gpp::AgentState a; a.id = o.id;
      a.x = ego.x + (c_ * o.x_m - s_ * o.y_m);
      a.y = ego.y + (s_ * o.x_m + c_ * o.y_m);
      const double head_abs = ego.yaw_rad + o.yaw_rad;
      a.vx = o.speed_mps * std::cos(head_abs);
      a.vy = o.speed_mps * std::sin(head_abs);
      agents.push_back(a);
    }

    // 진입 충돌점 월드 → base_link 전방거리 s
    const gpp::RotaryGateConfig& cfg = gate_.cfg;
    const double pcx = cfg.center_x + cfg.ring_radius_m * std::cos(cfg.theta_entry_rad);
    const double pcy = cfg.center_y + cfg.ring_radius_m * std::sin(cfg.theta_entry_rad);
    const double dx = pcx - ego.x, dy = pcy - ego.y;
    const double s_conflict = c_ * dx + s_ * dy;  // 전방(+x) 종거리. ponytail: 진입로 곡률 크면 경로 호길이로 교체

    gpp::RotaryDecision d = gate_.update(ego, agents, s_conflict, now.toSec());

    std_msgs::Float64 mstop; mstop.data = d.stop_wall_s; pub_stop_.publish(mstop);
    std_msgs::Float64 mcap; mcap.data = d.speed_cap_kph; pub_cap_.publish(mcap);

    ROS_INFO_THROTTLE(1.0,
        "[rotary_entry] phase=%d s_conf=%.1f stop=%.1f cap=%.1f eta=%.1f block=%d ring=%d all=%zu",
        (int)d.phase, s_conflict, d.stop_wall_s, d.speed_cap_kph, d.nearest_agent_eta_s,
        d.blocking_agent_id, d.n_ring_agents, agents.size());

    if (publish_markers_) publishMarkers(ego, agents, d, s_conflict);
  }

  // base_link 프레임 마커 (다른 노드 마커와 동일 프레임)
  void publishMarkers(const gpp::EgoState& ego, const std::vector<gpp::AgentState>& agents,
                      const gpp::RotaryDecision& d, double s_conflict) {
    const gpp::RotaryGateConfig& cfg = gate_.cfg;
    const double c_ = std::cos(ego.yaw_rad), s_ = std::sin(ego.yaw_rad);
    // 월드점 → base_link
    auto toBL = [&](double wx, double wy, double& bx, double& by) {
      const double dx = wx - ego.x, dy = wy - ego.y;
      bx = c_ * dx + s_ * dy; by = -s_ * dx + c_ * dy;
    };
    visualization_msgs::MarkerArray arr;
    std_msgs::Header hdr; hdr.frame_id = "base_link"; hdr.stamp = ros::Time::now();
    { visualization_msgs::Marker cl; cl.header=hdr; cl.ns="rotary"; cl.action=visualization_msgs::Marker::DELETEALL; arr.markers.push_back(cl); }
    auto mk=[&](int id,int type){ visualization_msgs::Marker m; m.header=hdr; m.ns="rotary"; m.id=id; m.type=type;
      m.action=visualization_msgs::Marker::ADD; m.lifetime=ros::Duration(0.3); m.pose.orientation.w=1.0; return m; };

    // 링 원
    visualization_msgs::Marker ring = mk(0, visualization_msgs::Marker::LINE_STRIP);
    ring.scale.x=0.15; ring.color.a=0.9; ring.color.r=0.4; ring.color.g=0.7; ring.color.b=1.0;
    for (int i=0;i<=48;++i){ double th=k2pi()*i/48.0; double bx,by;
      toBL(cfg.center_x+cfg.ring_radius_m*std::cos(th), cfg.center_y+cfg.ring_radius_m*std::sin(th), bx,by);
      geometry_msgs::Point p; p.x=bx;p.y=by;p.z=0.1; ring.points.push_back(p); }
    arr.markers.push_back(ring);
    // 진입/진출점
    for (int e=0;e<2;++e){ double th=(e==0)?cfg.theta_entry_rad:cfg.theta_exit_rad; double bx,by;
      toBL(cfg.center_x+cfg.ring_radius_m*std::cos(th), cfg.center_y+cfg.ring_radius_m*std::sin(th), bx,by);
      visualization_msgs::Marker m=mk(1+e, visualization_msgs::Marker::SPHERE);
      m.pose.position.x=bx;m.pose.position.y=by;m.pose.position.z=0.4; m.scale.x=m.scale.y=m.scale.z=1.0;
      m.color.a=0.9; if(e==0){m.color.r=1;m.color.g=1;} else {m.color.g=0.7;m.color.b=1;} arr.markers.push_back(m); }
    // 에이전트: 링 안(순환) 만 표시. 월드좌표 → base_link
    int id=10;
    for (const auto& a : agents){ if(!gate_.isRingAgent(a)) continue;
      double bx,by; toBL(a.x,a.y,bx,by);
      visualization_msgs::Marker m=mk(id, visualization_msgs::Marker::SPHERE);
      m.pose.position.x=bx;m.pose.position.y=by;m.pose.position.z=0.5; m.scale.x=m.scale.y=m.scale.z=0.8;
      m.color.a=0.9; if(a.id==d.blocking_agent_id){m.color.r=1;} else {m.color.g=1;} arr.markers.push_back(m); id++; }
    // 상태 텍스트 (자차 위)
    { visualization_msgs::Marker t=mk(5, visualization_msgs::Marker::TEXT_VIEW_FACING);
      t.pose.position.z=2.5; t.scale.z=0.6; t.color.a=1; t.color.r=t.color.g=t.color.b=1;
      const char* ph[]={"FAR","APPROACH","HOLD","COMMIT","INSIDE","DONE"};
      std::ostringstream ss; ss<<std::fixed<<std::setprecision(1)<<"ROTARY "<<ph[(int)d.phase]
        <<" s="<<s_conflict<<" t_arr="<<d.t_ego_arrive_s<<" eta="<<d.nearest_agent_eta_s
        <<" cap="<<d.speed_cap_kph<<(d.stop_wall_s>=0?" [STOP]":""); t.text=ss.str(); arr.markers.push_back(t); }
    // 정지벽
    if (d.stop_wall_s>=0.0){ visualization_msgs::Marker w=mk(6, visualization_msgs::Marker::CUBE);
      w.pose.position.x=d.stop_wall_s; w.pose.position.z=0.75; w.scale.x=0.2;w.scale.y=6.0;w.scale.z=1.5;
      w.color.a=0.4; w.color.r=1.0; arr.markers.push_back(w); }
    pub_markers_.publish(arr);
  }
  static double k2pi(){ return 2.0*M_PI; }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_ego_, sub_obs_;
  ros::Publisher pub_stop_, pub_cap_, pub_markers_;
  ros::Timer timer_;
  std::string ego_topic_, obstacle_topic_;
  double timer_rate_hz_{30.0}, ego_timeout_{0.5};
  bool publish_markers_{true};

  std::mutex mtx_;
  gpp::EgoState ego_;
  katri_msgs::ObstacleInfoArray latest_obs_;
  ros::Time last_ego_time_;
  bool has_ego_{false}, has_obs_{false};

  gpp::RotaryGate gate_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "rotary_entry");
  RotaryEntryNode node;
  ros::spin();
  return 0;
}
