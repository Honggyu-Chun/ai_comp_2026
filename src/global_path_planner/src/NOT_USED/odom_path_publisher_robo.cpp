/*
================================================================================
파일명: odom_path_publisher.cpp
================================================================================
기능:
1. Global Path(UTM) -> Local Path(Base_Link) 로버스트 슬라이싱 및 변환
   - Windowed Search, Teleport 감지, Watchdog 적용
2. (제외) Avoidance Path 변환은 본 노드에서 수행하지 않음
3. 현재 경로(/current_path) 결정부에 overlap lock + hysteresis 강화 적용
================================================================================
*/

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32.h>
#include <tf/tf.h>

#include <cmath>
#include <algorithm>
#include <limits>
#include <string>
#include <vector>

class OdomPathPublisher
{
public:
  struct TrackerState {
    int last_index = 0;
    bool is_first_run = true;
    ros::Time last_reset_time;
  };

  struct LocalPathResult {
    nav_msgs::Path path;   // base_link frame
    double best_dist = std::numeric_limits<double>::infinity();
    bool valid = false;
  };

  OdomPathPublisher()
  : nh_(), pnh_("~")
  {
    // =========================
    // Parameters
    // =========================
    // Topics
    pnh_.param<std::string>("odom_topic",         odom_topic_,         "/gps_utm_odom");
    pnh_.param<std::string>("global_path1_topic", global_path1_topic_, "/global_path1");
    pnh_.param<std::string>("global_path2_topic", global_path2_topic_, "/global_path2");
    
    pnh_.param<std::string>("local_path1_topic",  local_path1_topic_,  "/local_path1");
    pnh_.param<std::string>("local_path2_topic",  local_path2_topic_,  "/local_path2");
    
    pnh_.param<std::string>("current_path_topic", current_path_topic_, "/current_path");

    // Frames
    pnh_.param<std::string>("local_frame_id", local_frame_id_, "base_link");

    // Loop Rate
    pnh_.param<double>("rate_hz", rate_hz_, 50.0);

    // Local slicing params
    pnh_.param<double>("waypoint_interval", waypoint_interval_, 0.1);
    pnh_.param<double>("back_distance",     back_distance_,     0.5); 
    pnh_.param<double>("front_distance",    front_distance_,    20.0);

    // Window search params
    pnh_.param<int>("window_back", window_back_, 200);
    pnh_.param<int>("window_fwd",  window_fwd_,  800);

    // Teleport/reset params
    pnh_.param<double>("reset_cooldown_sec",           reset_cooldown_sec_,           1.0);
    pnh_.param<double>("teleport_ego_dist_thresh",     teleport_ego_dist_thresh_,     5.0);
    pnh_.param<double>("teleport_nearest_dist_thresh", teleport_nearest_dist_thresh_, 30.0);

    // Wrap policy
    pnh_.param("is_circular_path", is_circular_path_, true); 

    // Watchdog
    pnh_.param("require_fresh_path", require_fresh_path_, false);
    pnh_.param<double>("path_timeout_sec", path_timeout_sec_, 2.0);
    pnh_.param<double>("odom_timeout_sec", odom_timeout_sec_, 0.5);

    // Stability params
    pnh_.param<double>("switch_margin_m", switch_margin_m_, 0.3);
    pnh_.param<double>("min_switch_interval_s", min_switch_interval_s_, 0.5);
    pnh_.param<int>("better_count_required", better_count_required_, 3);
    pnh_.param<double>("force_switch_dist_m", force_switch_dist_m_, 2.0);
    pnh_.param<double>("force_switch_margin_m", force_switch_margin_m_, 1.0);
    pnh_.param<int>("min_path_points", min_path_points_, 5);

    pnh_.param<bool>("use_overlap_lock", use_overlap_lock_, true);
    pnh_.param<int>("overlap_check_points", overlap_check_points_, 30);
    pnh_.param<int>("overlap_ahead_offset_pts", overlap_ahead_offset_pts_, 0);
    pnh_.param<double>("overlap_enter_m", overlap_enter_m_, 0.25);
    pnh_.param<double>("overlap_exit_m", overlap_exit_m_, 0.60);
    pnh_.param<int>("overlap_enter_confirm_frames", overlap_enter_confirm_frames_, 3);
    pnh_.param<int>("overlap_exit_confirm_frames", overlap_exit_confirm_frames_, 10);
    pnh_.param<double>("unlock_cooldown_s", unlock_cooldown_s_, 0.5);
    pnh_.param<double>("overlap_unlock_hold_s", overlap_unlock_hold_s_, 0.0);

    // Optional LPF
    pnh_.param<bool>("use_lpf", use_lpf_, false);
    pnh_.param<double>("lpf_tau_s", lpf_tau_s_, 0.2);

    // =========================
    // Subscribers / Publishers
    // =========================
    gps_sub_          = nh_.subscribe(odom_topic_,         10, &OdomPathPublisher::GPSCallBack,         this);
    global_path_1_sub_= nh_.subscribe(global_path1_topic_,  1, &OdomPathPublisher::GlobalPath1CallBack, this);
    global_path_2_sub_= nh_.subscribe(global_path2_topic_,  1, &OdomPathPublisher::GlobalPath2CallBack, this);
    
    local_path1_pub_  = nh_.advertise<nav_msgs::Path>(local_path1_topic_, 1);
    local_path2_pub_  = nh_.advertise<nav_msgs::Path>(local_path2_topic_, 1);
    
    current_path_pub_ = nh_.advertise<std_msgs::Int32>(current_path_topic_, 1);

    // =========================
    // State init
    // =========================
    current_position_.x = current_position_.y = current_position_.z = 0.0;
    vehicle_yaw_ = 0.0;

    gps_state_ = false;
    path_state_1_ = false;
    path_state_2_ = false;

    has_prev_ego_ = false;
    prev_ego_x_ = prev_ego_y_ = 0.0;

    last_odom_stamp_ = ros::Time(0);

    stable_lane_ = 0;
    last_switch_time_ = ros::Time(0);
    pending_lane_ = 0;
    pending_count_ = 0;
    overlap_lock_ = false;
    overlap_enter_cnt_ = 0;
    overlap_exit_cnt_ = 0;
    d1_f_ = std::numeric_limits<double>::quiet_NaN();
    d2_f_ = std::numeric_limits<double>::quiet_NaN();
    lpf_inited_ = false;
    last_filter_time_ = ros::Time(0);
    last_overlap_sep_ = std::numeric_limits<double>::infinity();
    last_overlap_unlock_time_ = ros::Time(0);

    ROS_INFO("[OdomPathPublisher] Initialized.");
    ROS_INFO("  - Converting Global Paths -> Local Paths");
    ROS_INFO("  - Avoidance Path conversion disabled in this node");
    ROS_INFO("  - Current path selection: overlap-lock + hysteresis (test node)");
  }

  void spin()
  {
    ros::Rate rate(rate_hz_);
    while (ros::ok()) {
      ros::spinOnce();

      const ros::Time now = ros::Time::now();

      // --------------------------
      // 1) Odom watchdog
      // --------------------------
      if (!gps_state_) {
        ROS_INFO_THROTTLE(2.0, "[OdomPathPublisher] Waiting for Odom...");
        rate.sleep();
        continue;
      }
      if (!last_odom_stamp_.isZero()) {
        const double odom_age = (now - last_odom_stamp_).toSec();
        if (odom_age > odom_timeout_sec_) {
          ROS_WARN_THROTTLE(1.0, "[OdomPathPublisher] Odom timeout: age=%.3f", odom_age);
          // Publish empty paths to stop the car safely
          local_path1_pub_.publish(makeEmptyLocalPath(now));
          local_path2_pub_.publish(makeEmptyLocalPath(now));
          rate.sleep();
          continue;
        }
      }

      // --------------------------
      // 2) Teleport handling
      // --------------------------
      handleEgoTeleport();

      // --------------------------
      // 3) Build Global -> Local paths
      // --------------------------
      LocalPathResult r1, r2;

      // Process Path 1
      if (path_state_1_ && global_path_1_ptr_) {
        if (isPathFresh(*global_path_1_ptr_, now)) {
          r1 = buildLocalPath(*global_path_1_ptr_, path1_tracker_, now);
          if (!r1.valid) resetTracker(path1_tracker_);
          local_path1_pub_.publish(r1.path);
        } else {
          local_path1_pub_.publish(makeEmptyLocalPath(now));
        }
      } else {
        local_path1_pub_.publish(makeEmptyLocalPath(now));
      }

      // Process Path 2
      if (path_state_2_ && global_path_2_ptr_) {
        if (isPathFresh(*global_path_2_ptr_, now)) {
          r2 = buildLocalPath(*global_path_2_ptr_, path2_tracker_, now);
          if (!r2.valid) resetTracker(path2_tracker_);
          local_path2_pub_.publish(r2.path);
        } else {
          local_path2_pub_.publish(makeEmptyLocalPath(now));
        }
      } else {
        local_path2_pub_.publish(makeEmptyLocalPath(now));
      }

      // --------------------------
      // 4) Identify Nearest Path (Current Path) - Stabilized
      // --------------------------
      const int decided_lane = decideStableCurrentLane(r1, r2, now);
      publishCurrentPath(decided_lane);

      const double debug_d1 = use_lpf_ && lpf_inited_ ? d1_f_ : r1.best_dist;
      const double debug_d2 = use_lpf_ && lpf_inited_ ? d2_f_ : r2.best_dist;
      // ROS_INFO_THROTTLE(
      //   1.0,
      //   "[OdomPathPublisher] d1=%.2f d2=%.2f sep=%.2f lock=%d stable=%d pending=%d/%d", 
      //   debug_d1,
      //   debug_d2,
      //   last_overlap_sep_,
      //   overlap_lock_ ? 1 : 0,
      //   decided_lane,
      //   pending_count_,
      //   pending_lane_
      // );

      rate.sleep();
    }
  }

private:
  // ============ Callbacks ============
  void GPSCallBack(const nav_msgs::Odometry::ConstPtr &msg)
  {
    gps_state_ = true;
    last_odom_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;

    tf::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);

    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    vehicle_yaw_ = yaw;

    current_position_.x = msg->pose.pose.position.x;
    current_position_.y = msg->pose.pose.position.y;
    current_position_.z = msg->pose.pose.position.z;
  }

  void GlobalPath1CallBack(const nav_msgs::Path::ConstPtr &msg)
  {
    path_state_1_ = true;
    global_path_1_ptr_ = msg;
    resetTracker(path1_tracker_);
  }

  void GlobalPath2CallBack(const nav_msgs::Path::ConstPtr &msg)
  {
    path_state_2_ = true;
    global_path_2_ptr_ = msg;
    resetTracker(path2_tracker_);
  }

  // ============ Watchdog helpers ============
  bool isPathFresh(const nav_msgs::Path &path, const ros::Time &now) const
  {
    if (!require_fresh_path_) return true;
    if (path.header.stamp.isZero()) return true;

    const double age = (now - path.header.stamp).toSec();
    if (age > path_timeout_sec_) {
      ROS_WARN_THROTTLE(1.0, "[OdomPathPublisher] Path stale: age=%.3f", age);
      return false;
    }
    return true;
  }

  nav_msgs::Path makeEmptyLocalPath(const ros::Time &stamp) const
  {
    nav_msgs::Path p;
    p.header.frame_id = local_frame_id_;
    p.header.stamp = stamp;
    return p;
  }

  void publishCurrentPath(int lane)
  {
    std_msgs::Int32 msg;
    if (lane != 1 && lane != 2) lane = 1;
    msg.data = lane;
    current_path_pub_.publish(msg);
  }

  // ============ Teleport / reset ============
  void handleEgoTeleport()
  {
    const double ego_x = current_position_.x;
    const double ego_y = current_position_.y;

    if (!has_prev_ego_) {
      prev_ego_x_ = ego_x;
      prev_ego_y_ = ego_y;
      has_prev_ego_ = true;
      return;
    }

    const double dx = ego_x - prev_ego_x_;
    const double dy = ego_y - prev_ego_y_;
    const double d2 = dx*dx + dy*dy;

    if (d2 > teleport_ego_dist_thresh_ * teleport_ego_dist_thresh_) {
      resetTracker(path1_tracker_);
      resetTracker(path2_tracker_);
      ROS_WARN_THROTTLE(0.5, "[OdomPathPublisher] EGO TELEPORT DETECTED! Resetting trackers.");
    }

    prev_ego_x_ = ego_x;
    prev_ego_y_ = ego_y;
  }

  void resetTracker(TrackerState &tracker)
  {
    const ros::Time now = ros::Time::now();
    if (!tracker.last_reset_time.isZero()) {
      if ((now - tracker.last_reset_time).toSec() < reset_cooldown_sec_) {
        return;
      }
    }
    tracker.last_index = 0;
    tracker.is_first_run = true;
    tracker.last_reset_time = now;
  }

  // ============ Math helpers ============
  inline double normalizeYaw(double yaw) const
  {
    return std::atan2(std::sin(yaw), std::cos(yaw));
  }

  // base_link 변환 (직접 계산)
  inline void transformPoint2D(double ego_x, double ego_y, double yaw_norm,
                               double gx, double gy,
                               double &bx, double &by) const
  {
    const double c = std::cos(yaw_norm);
    const double s = std::sin(yaw_norm);
    const double dx = gx - ego_x;
    const double dy = gy - ego_y;
    bx =  c*dx + s*dy;
    by = -s*dx + c*dy;
  }

  // ============ Nearest search (windowed + fallback) ============
  int findNearestIndexWindowed(const nav_msgs::Path &path,
                               double ego_x, double ego_y,
                               TrackerState &tracker,
                               double &out_best_dist) const
  {
    const int n = static_cast<int>(path.poses.size());
    if (n <= 0) { out_best_dist = std::numeric_limits<double>::infinity(); return 0; }

    int start_idx = 0;
    int end_idx = n - 1;
    if (!tracker.is_first_run) {
      start_idx = std::max(0, tracker.last_index - window_back_);
      end_idx   = std::min(n - 1, tracker.last_index + window_fwd_);
    }

    double best_d2 = std::numeric_limits<double>::infinity();
    int best_idx = start_idx;

    for (int i = start_idx; i <= end_idx; ++i) {
      const double gx = path.poses[i].pose.position.x;
      const double gy = path.poses[i].pose.position.y;
      const double dx = ego_x - gx;
      const double dy = ego_y - gy;
      const double d2 = dx*dx + dy*dy;
      if (d2 < best_d2) { best_d2 = d2; best_idx = i; }
    }

    out_best_dist = std::sqrt(best_d2);
    tracker.last_index = best_idx;
    tracker.is_first_run = false;
    return best_idx;
  }

  int findNearestIndexFullScan(const nav_msgs::Path &path,
                               double ego_x, double ego_y,
                               double &out_best_dist) const
  {
    const int n = static_cast<int>(path.poses.size());
    if (n <= 0) { out_best_dist = std::numeric_limits<double>::infinity(); return 0; }

    double best_d2 = std::numeric_limits<double>::infinity();
    int best_idx = 0;

    for (int i = 0; i < n; ++i) {
      const double gx = path.poses[i].pose.position.x;
      const double gy = path.poses[i].pose.position.y;
      const double dx = ego_x - gx;
      const double dy = ego_y - gy;
      const double d2 = dx*dx + dy*dy;
      if (d2 < best_d2) { best_d2 = d2; best_idx = i; }
    }

    out_best_dist = std::sqrt(best_d2);
    return best_idx;
  }

  // ============ Core builder ============
  LocalPathResult buildLocalPath(const nav_msgs::Path &global_path,
                                 TrackerState &tracker,
                                 const ros::Time &stamp)
  {
    LocalPathResult res;
    res.path.header.frame_id = local_frame_id_;
    res.path.header.stamp = stamp;

    const int n = static_cast<int>(global_path.poses.size());
    if (n <= 0) { res.valid = false; return res; }

    const double ego_x = current_position_.x;
    const double ego_y = current_position_.y;
    const double yaw_norm = normalizeYaw(vehicle_yaw_);

    // 1. Windowed Nearest Search
    double best_dist = 0.0;
    int nearest_idx = findNearestIndexWindowed(global_path, ego_x, ego_y, tracker, best_dist);

    // 2. Fallback Full Scan (if local minimum or teleport)
    if (best_dist > teleport_nearest_dist_thresh_) {
      double fs_dist = 0.0;
      int fs_idx = findNearestIndexFullScan(global_path, ego_x, ego_y, fs_dist);
      nearest_idx = fs_idx;
      best_dist = fs_dist;
      tracker.last_index = nearest_idx;
      tracker.is_first_run = false;

      if (best_dist > teleport_nearest_dist_thresh_) {
        res.best_dist = best_dist;
        res.valid = false;
        return res; // Path is too far
      }
    }

    res.best_dist = best_dist;
    res.valid = true;

    // 3. Slicing Indices
    const int back_count  = static_cast<int>(back_distance_  / waypoint_interval_);
    const int front_count = static_cast<int>(front_distance_ / waypoint_interval_);

    int start_waypoint = std::max(0, nearest_idx - back_count);
    int end_waypoint   = nearest_idx + front_count;

    if (!is_circular_path_) end_waypoint = std::min(end_waypoint, n);

    const int out_count = std::max(0, end_waypoint - start_waypoint);
    res.path.poses.reserve(static_cast<size_t>(out_count));

    // 4. Generate Local Path (Slicing + Transform)
    for (int i = start_waypoint; i < end_waypoint; ++i) {
      int index = i;

      if (is_circular_path_) {
        index = (n > 0) ? (i % n) : 0;
      } else {
        if (i < 0 || i >= n) break;
      }

      const double gx = global_path.poses[index].pose.position.x;
      const double gy = global_path.poses[index].pose.position.y;
      const double gz = global_path.poses[index].pose.position.z;

      double bx, by;
      transformPoint2D(ego_x, ego_y, yaw_norm, gx, gy, bx, by);

      geometry_msgs::PoseStamped p;
      p.header.frame_id = local_frame_id_;
      p.header.stamp = stamp;
      p.pose.position.x = bx;
      p.pose.position.y = by;
      p.pose.position.z = gz;
      p.pose.orientation.w = 1.0;

      res.path.poses.push_back(p);
    }

    return res;
  }

  // ============ Decision helpers ============
  double computeOverlapSeparation(const LocalPathResult &r1, const LocalPathResult &r2) const
  {
    const int sz1 = static_cast<int>(r1.path.poses.size());
    const int sz2 = static_cast<int>(r2.path.poses.size());
    const int min_size = std::min(sz1, sz2);

    if (min_size <= 0) {
      return std::numeric_limits<double>::infinity();
    }

    const int back_count = static_cast<int>(
      back_distance_ / std::max(waypoint_interval_, std::numeric_limits<double>::epsilon())
    );

    int start = back_count + overlap_ahead_offset_pts_;
    if (start < 0) start = 0;
    if (start > min_size - 1) start = min_size - 1;

    const int avail = min_size - start;
    const int n = std::min(overlap_check_points_, avail);
    if (n <= 0) {
      return std::numeric_limits<double>::infinity();
    }

    std::vector<double> seps;
    seps.reserve(static_cast<size_t>(n));

    for (int i = 0; i < n; ++i) {
      const auto &p1 = r1.path.poses[start + i].pose.position;
      const auto &p2 = r2.path.poses[start + i].pose.position;
      const double dx = p1.x - p2.x;
      const double dy = p1.y - p2.y;
      seps.push_back(std::hypot(dx, dy));
    }

    if (seps.empty()) {
      return std::numeric_limits<double>::infinity();
    }

    std::sort(seps.begin(), seps.end());
    const int mid = static_cast<int>(seps.size()) / 2;
    if ((seps.size() & 1) == 1) {
      return seps[mid];
    }

    return (seps[mid - 1] + seps[mid]) * 0.5;
  }

  void updateOverlapLock(double sep, bool valid1, bool valid2, const ros::Time &now)
  {
    const bool lock_ready = use_overlap_lock_ && valid1 && valid2 && std::isfinite(sep);

    if (!lock_ready) {
      overlap_lock_ = false;
      overlap_enter_cnt_ = 0;
      overlap_exit_cnt_ = 0;
      last_overlap_unlock_time_ = ros::Time(0);
      return;
    }

    const int enter_needed = std::max(1, overlap_enter_confirm_frames_);
    const int exit_needed  = std::max(1, overlap_exit_confirm_frames_);

    if (!overlap_lock_) {
      if (sep < overlap_enter_m_) {
        if (overlap_enter_cnt_ < enter_needed) {
          ++overlap_enter_cnt_;
        }
        if (overlap_enter_cnt_ >= enter_needed) {
          overlap_lock_ = true;
          overlap_enter_cnt_ = 0;
          overlap_exit_cnt_ = 0;
          last_overlap_unlock_time_ = ros::Time(0);
          // ROS_INFO_THROTTLE(0.5, "[OdomPathPublisher] Enter overlap lock (sep=%.2f)", sep);
        }
      } else {
        overlap_enter_cnt_ = 0;
      }
    } else {
      if (sep > overlap_exit_m_) {
        if (overlap_exit_cnt_ < exit_needed) {
          ++overlap_exit_cnt_;
        }
        if (overlap_exit_cnt_ >= exit_needed) {
          overlap_lock_ = false;
          overlap_exit_cnt_ = 0;
          overlap_enter_cnt_ = 0;
          last_overlap_unlock_time_ = now;
          last_switch_time_ = now;
          // ROS_INFO_THROTTLE(0.5, "[OdomPathPublisher] Exit overlap lock (sep=%.2f)", sep);
        }
      } else {
        overlap_exit_cnt_ = 0;
      }
    }
  }

  void updateFilteredDistances(const ros::Time &now, double raw_d1, double raw_d2, double &d1, double &d2)
  {
    if (!use_lpf_) {
      d1 = raw_d1;
      d2 = raw_d2;
      return;
    }

    if (!lpf_inited_) {
      d1_f_ = raw_d1;
      d2_f_ = raw_d2;
      lpf_inited_ = true;
      last_filter_time_ = now;
      d1 = d1_f_;
      d2 = d2_f_;
      return;
    }

    const double dt = (now - last_filter_time_).toSec();
    const double tau = std::max(1e-6, lpf_tau_s_);
    const double alpha = dt <= 0.0 ? 1.0 : std::min(1.0, dt / tau);

    d1_f_ = d1_f_ + (raw_d1 - d1_f_) * alpha;
    d2_f_ = d2_f_ + (raw_d2 - d2_f_) * alpha;
    last_filter_time_ = now;

    d1 = d1_f_;
    d2 = d2_f_;
  }

  int decideStableCurrentLane(const LocalPathResult &r1, const LocalPathResult &r2, const ros::Time &now)
  {
    const bool valid1 = r1.valid && static_cast<int>(r1.path.poses.size()) >= min_path_points_;
    const bool valid2 = r2.valid && static_cast<int>(r2.path.poses.size()) >= min_path_points_;

    double d1 = r1.best_dist;
    double d2 = r2.best_dist;
    updateFilteredDistances(now, r1.best_dist, r2.best_dist, d1, d2);

    last_overlap_sep_ = computeOverlapSeparation(r1, r2);
    updateOverlapLock(last_overlap_sep_, valid1, valid2, now);

    if (stable_lane_ == 0) {
      if (valid1 && valid2) {
        stable_lane_ = (d1 <= d2) ? 1 : 2;
      } else if (valid1) {
        stable_lane_ = 1;
      } else if (valid2) {
        stable_lane_ = 2;
      } else {
        stable_lane_ = 1;
      }
      last_switch_time_ = now;
      return stable_lane_;
    }

    int current = stable_lane_;
    int decided = current;

    if (!valid1 && valid2) {
      decided = 2;
    } else if (!valid2 && valid1) {
      decided = 1;
    } else if (!valid1 && !valid2) {
      decided = current;
    } else {
      if (current != 1 && current != 2) {
        current = (d1 <= d2) ? 1 : 2;
      }
      decided = current;

      // overlap lock이 막 풀린 직후에는 기존 lane을 잠깐 유지해
      // 분기 시작점의 1~2프레임 거리 튐이 current_path로 즉시 전파되지 않게 한다.
      const bool current_valid = (current == 1) ? valid1 : valid2;
      const bool in_unlock_hold = !overlap_lock_ &&
                                  current_valid &&
                                  overlap_unlock_hold_s_ > 0.0 &&
                                  !last_overlap_unlock_time_.isZero() &&
                                  ((now - last_overlap_unlock_time_).toSec() < overlap_unlock_hold_s_);
      if (in_unlock_hold) {
        pending_lane_ = 0;
        pending_count_ = 0;
        return stable_lane_;
      }

      const int other = 3 - current;
      const double d_cur = (current == 1) ? d1 : d2;
      const double d_other = (current == 1) ? d2 : d1;
      const double hold_time = (now - last_switch_time_).toSec();
      const bool hold_ok = hold_time >= min_switch_interval_s_;
      const bool force_switch = (d_cur > force_switch_dist_m_) ||
                               (d_other + force_switch_margin_m_ < d_cur);

      if (force_switch) {
        decided = other;
      } else if (!overlap_lock_) {
        const bool strong_better = (d_other + switch_margin_m_ < d_cur);
        if (strong_better && hold_ok) {
          const int need_cnt = std::max(1, better_count_required_);
          if (pending_lane_ == other) {
            ++pending_count_;
          } else {
            pending_lane_ = other;
            pending_count_ = 1;
          }

          if (pending_count_ >= need_cnt) {
            decided = other;
          }
        } else {
          pending_lane_ = 0;
          pending_count_ = 0;
        }
      }
    }

    if (decided == 0) decided = 1;

    if (decided != stable_lane_) {
      // unlock event immediately applies cooldown effect through last_switch_time_
      stable_lane_ = decided;
      last_switch_time_ = now;
      pending_lane_ = 0;
      pending_count_ = 0;
    }

    // unlock cooldown: keep hold time after unlock even without switching
    if (!overlap_lock_) {
      const double unlock_age = (now - last_switch_time_).toSec();
      if (unlock_age < unlock_cooldown_s_) {
        pending_lane_ = 0;
        pending_count_ = 0;
      }
    }

    return stable_lane_;
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // topics/frames
  std::string odom_topic_, global_path1_topic_, global_path2_topic_;
  std::string local_path1_topic_, local_path2_topic_, current_path_topic_;
  
  std::string local_frame_id_;

  // loop
  double rate_hz_;

  // params
  double waypoint_interval_, back_distance_, front_distance_;
  int window_back_, window_fwd_;
  double reset_cooldown_sec_, teleport_ego_dist_thresh_, teleport_nearest_dist_thresh_;
  bool is_circular_path_;

  // watchdog
  bool require_fresh_path_;
  double path_timeout_sec_;
  double odom_timeout_sec_;

  // stability params
  double switch_margin_m_;
  double min_switch_interval_s_;
  int better_count_required_;
  double force_switch_dist_m_;
  double force_switch_margin_m_;
  int min_path_points_;

  bool use_overlap_lock_;
  int overlap_check_points_;
  int overlap_ahead_offset_pts_;
  double overlap_enter_m_;
  double overlap_exit_m_;
  int overlap_enter_confirm_frames_;
  int overlap_exit_confirm_frames_;
  double unlock_cooldown_s_;
  double overlap_unlock_hold_s_;

  bool use_lpf_;
  double lpf_tau_s_;

  // subs/pubs
  ros::Subscriber gps_sub_, global_path_1_sub_, global_path_2_sub_;
  ros::Publisher local_path1_pub_, local_path2_pub_, current_path_pub_;

  // states
  geometry_msgs::Point current_position_;
  double vehicle_yaw_;
  bool gps_state_, path_state_1_, path_state_2_;

  // pointers (deep copy 제거)
  nav_msgs::PathConstPtr global_path_1_ptr_;
  nav_msgs::PathConstPtr global_path_2_ptr_;

  // trackers
  TrackerState path1_tracker_;
  TrackerState path2_tracker_;

  // ego teleport tracking
  bool has_prev_ego_;
  double prev_ego_x_, prev_ego_y_;

  // current path selection state
  int stable_lane_ = 0;
  ros::Time last_switch_time_;
  int pending_lane_ = 0;
  int pending_count_ = 0;

  bool overlap_lock_ = false;
  int overlap_enter_cnt_ = 0;
  int overlap_exit_cnt_ = 0;
  double last_overlap_sep_;
  ros::Time last_overlap_unlock_time_;

  // LPF states
  double d1_f_ = std::numeric_limits<double>::quiet_NaN();
  double d2_f_ = std::numeric_limits<double>::quiet_NaN();
  bool lpf_inited_ = false;
  ros::Time last_filter_time_;

  // timestamps
  ros::Time last_odom_stamp_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_path_publisher");
  OdomPathPublisher node;
  node.spin();
  return 0;
}
