#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int32.h>
#include <tf/tf.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

class OdomPathPublisher
{
public:
    OdomPathPublisher()
        : nh_(),
          pnh_("~"),
          gps_state_(false),
          path_state_1_(false),
          path_state_2_(false),
          vehicle_yaw_(0.0),
          vehicle_heading_x_(1.0),
          vehicle_heading_y_(0.0),
          has_prev_ego_(false),
          prev_ego_x_(0.0),
          prev_ego_y_(0.0),
          stable_current_path_(0),
          pending_path_(0),
          pending_count_(0)
    {
        pnh_.param<std::string>("odom_topic", odom_topic_, std::string("/gps_utm_odom"));
        pnh_.param<std::string>("global_path1_topic", global_path1_topic_, std::string("/global_path1"));
        pnh_.param<std::string>("global_path2_topic", global_path2_topic_, std::string("/global_path2"));
        pnh_.param<std::string>("local_path1_topic", local_path1_topic_, std::string("/local_path1"));
        pnh_.param<std::string>("local_path2_topic", local_path2_topic_, std::string("/local_path2"));
        pnh_.param<std::string>("current_path_topic", current_path_topic_, std::string("/current_path"));
        pnh_.param<std::string>("local_frame_id", local_frame_id_, std::string("base_link"));

        pnh_.param("rate_hz", rate_hz_, 50.0);
        pnh_.param("waypoint_interval", waypoint_interval_, 0.1);
        pnh_.param("back_distance", back_distance_, 2.0);
        pnh_.param("front_distance", front_distance_, 50.0);

        pnh_.param("window_back", window_back_, 10);
        pnh_.param("window_fwd", window_fwd_, 100);
        pnh_.param("is_circular_path", is_circular_path_, false);

        pnh_.param("teleport_ego_dist_thresh", teleport_ego_dist_thresh_, 5.0);
        pnh_.param("max_valid_nearest_dist", max_valid_nearest_dist_, 30.0);

        pnh_.param("distance_weight", distance_weight_, 1.0);
        pnh_.param("angle_weight", angle_weight_, 5.0);
        pnh_.param("forward_dot_min", forward_dot_min_, 0.0);
        pnh_.param("path_heading_dot_min", path_heading_dot_min_, 0.0);

        pnh_.param("switch_margin_cost", switch_margin_cost_, 1.0);
        pnh_.param("min_switch_interval_s", min_switch_interval_s_, 0.5);
        pnh_.param("better_count_required", better_count_required_, 3);

        rate_hz_ = std::max(1.0, rate_hz_);
        waypoint_interval_ = std::max(1e-3, waypoint_interval_);
        window_back_ = std::max(0, window_back_);
        window_fwd_ = std::max(0, window_fwd_);
        teleport_ego_dist_thresh_ = std::max(0.0, teleport_ego_dist_thresh_);
        max_valid_nearest_dist_ = std::max(0.0, max_valid_nearest_dist_);
        distance_weight_ = std::max(0.0, distance_weight_);
        angle_weight_ = std::max(0.0, angle_weight_);
        better_count_required_ = std::max(1, better_count_required_);
        min_switch_interval_s_ = std::max(0.0, min_switch_interval_s_);
        back_steps_ = static_cast<int>(std::lround(back_distance_ / waypoint_interval_));
        front_steps_ = static_cast<int>(std::lround(front_distance_ / waypoint_interval_));
        teleport_ego_dist_thresh_sq_ = teleport_ego_dist_thresh_ * teleport_ego_dist_thresh_;
        max_valid_nearest_dist_sq_ = max_valid_nearest_dist_ * max_valid_nearest_dist_;

        current_position_.x = 0.0;
        current_position_.y = 0.0;
        current_position_.z = 0.0;

        gps_sub_ = nh_.subscribe(odom_topic_, 10, &OdomPathPublisher::gpsCallback, this);
        global_path_1_sub_ = nh_.subscribe(global_path1_topic_, 1, &OdomPathPublisher::globalPath1Callback, this);
        global_path_2_sub_ = nh_.subscribe(global_path2_topic_, 1, &OdomPathPublisher::globalPath2Callback, this);

        local_path1_pub_ = nh_.advertise<nav_msgs::Path>(local_path1_topic_, 1);
        local_path2_pub_ = nh_.advertise<nav_msgs::Path>(local_path2_topic_, 1);
        current_path_pub_ = nh_.advertise<std_msgs::Int32>(current_path_topic_, 1);

        ROS_INFO("[odom_path_publisher] initialized: window=[-%d,+%d], circular=%s",
                 window_back_, window_fwd_, is_circular_path_ ? "true" : "false");
    }

    void spin()
    {
        ros::Rate rate(rate_hz_);
        while (ros::ok()) {
            ros::spinOnce();

            const ros::Time now = ros::Time::now();
            if (!gps_state_) {
                ROS_INFO_THROTTLE(2.0, "[odom_path_publisher] waiting for odom");
                rate.sleep();
                continue;
            }

            handleEgoTeleport();

            resetPathMessage(local_path1_msg_, now);
            resetPathMessage(local_path2_msg_, now);

            LocalPathResult path1_result;
            LocalPathResult path2_result;

            if (path_state_1_ && global_path_1_cache_.path) {
                path1_result = buildLocalPath(global_path_1_cache_, path1_tracker_, now, local_path1_msg_);
            }
            if (path_state_2_ && global_path_2_cache_.path) {
                path2_result = buildLocalPath(global_path_2_cache_, path2_tracker_, now, local_path2_msg_);
            }

            local_path1_pub_.publish(local_path1_msg_);
            local_path2_pub_.publish(local_path2_msg_);
            publishCurrentPath(decideCurrentPath(path1_result, path2_result, now));

            ROS_INFO_THROTTLE(1.0,
                              "[odom_path_publisher] valid=[%d,%d] idx=[%d,%d] cost=[%.2f,%.2f] current=%d",
                              path1_result.valid ? 1 : 0,
                              path2_result.valid ? 1 : 0,
                              path1_result.best_index,
                              path2_result.best_index,
                              path1_result.cost,
                              path2_result.cost,
                              stable_current_path_);

            rate.sleep();
        }
    }

private:
    struct TrackerState
    {
        int last_index = 0;
        bool is_first_run = true;
    };

    struct CachedTangent
    {
        bool valid = false;
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
    };

    struct CachedPath
    {
        nav_msgs::PathConstPtr path;
        std::vector<CachedTangent> tangents;
    };

    struct SearchResult
    {
        bool valid = false;
        int index = 0;
        double distance = std::numeric_limits<double>::infinity();
        double distance_sq = std::numeric_limits<double>::infinity();
        double cost = std::numeric_limits<double>::infinity();
        double angle_diff = 0.0;
    };

    struct LocalPathResult
    {
        bool valid = false;
        int best_index = -1;
        double best_distance = std::numeric_limits<double>::infinity();
        double cost = std::numeric_limits<double>::infinity();
    };

    void gpsCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        gps_state_ = true;

        tf::Quaternion q(msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z,
                         msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        m.getRPY(roll, pitch, yaw);

        vehicle_yaw_ = normalizeAngle(yaw);
        vehicle_heading_x_ = std::cos(vehicle_yaw_);
        vehicle_heading_y_ = std::sin(vehicle_yaw_);
        current_position_.x = msg->pose.pose.position.x;
        current_position_.y = msg->pose.pose.position.y;
        current_position_.z = msg->pose.pose.position.z;
    }

    void globalPath1Callback(const nav_msgs::Path::ConstPtr& msg)
    {
        setCachedPath(msg, global_path_1_cache_);
        path_state_1_ = true;
        resetTracker(path1_tracker_);
    }

    void globalPath2Callback(const nav_msgs::Path::ConstPtr& msg)
    {
        setCachedPath(msg, global_path_2_cache_);
        path_state_2_ = true;
        resetTracker(path2_tracker_);
    }

    void resetTracker(TrackerState& tracker)
    {
        tracker.last_index = 0;
        tracker.is_first_run = true;
    }

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
        if (dx * dx + dy * dy > teleport_ego_dist_thresh_sq_) {
            resetTracker(path1_tracker_);
            resetTracker(path2_tracker_);
            ROS_WARN_THROTTLE(0.5, "[odom_path_publisher] ego teleport detected; reset search trackers");
        }

        prev_ego_x_ = ego_x;
        prev_ego_y_ = ego_y;
    }

    void resetPathMessage(nav_msgs::Path& path, const ros::Time& stamp) const
    {
        path.header.frame_id = local_frame_id_;
        path.header.stamp = stamp;
        path.poses.clear();
    }

    double normalizeAngle(const double angle) const
    {
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    int wrapIndex(const int index, const int size) const
    {
        if (size <= 0) {
            return 0;
        }
        const int wrapped = index % size;
        return wrapped < 0 ? wrapped + size : wrapped;
    }

    bool computePathTangent(const nav_msgs::Path& path,
                            const int index,
                            CachedTangent& tangent) const
    {
        const int size = static_cast<int>(path.poses.size());
        if (size < 2 || index < 0 || index >= size) {
            return false;
        }

        int prev_index = index;
        int next_index = index;
        if (is_circular_path_ && size > 2) {
            prev_index = wrapIndex(index - 1, size);
            next_index = wrapIndex(index + 1, size);
        } else if (index == 0) {
            next_index = 1;
        } else if (index == size - 1) {
            prev_index = size - 2;
        } else {
            prev_index = index - 1;
            next_index = index + 1;
        }

        const double dx = path.poses[next_index].pose.position.x - path.poses[prev_index].pose.position.x;
        const double dy = path.poses[next_index].pose.position.y - path.poses[prev_index].pose.position.y;
        const double norm = std::hypot(dx, dy);
        if (norm < 1e-6) {
            return false;
        }

        tangent.valid = true;
        tangent.x = dx / norm;
        tangent.y = dy / norm;
        tangent.yaw = std::atan2(tangent.y, tangent.x);
        return true;
    }

    void setCachedPath(const nav_msgs::Path::ConstPtr& msg, CachedPath& cached_path)
    {
        cached_path.path = msg;
        cached_path.tangents.clear();

        if (!msg) {
            return;
        }

        const int size = static_cast<int>(msg->poses.size());
        cached_path.tangents.resize(static_cast<std::size_t>(size));
        for (int i = 0; i < size; ++i) {
            computePathTangent(*msg, i, cached_path.tangents[static_cast<std::size_t>(i)]);
        }
    }

    bool evaluateCandidate(const CachedPath& cached_path,
                           const int index,
                           const bool strict_path_heading_filter,
                           SearchResult& best) const
    {
        if (!cached_path.path ||
            index < 0 ||
            index >= static_cast<int>(cached_path.tangents.size())) {
            return false;
        }

        const auto& point = cached_path.path->poses[static_cast<std::size_t>(index)].pose.position;
        const double dx = point.x - current_position_.x;
        const double dy = point.y - current_position_.y;
        const double forward_dot = dx * vehicle_heading_x_ + dy * vehicle_heading_y_;
        if (forward_dot < forward_dot_min_) {
            return false;
        }

        const CachedTangent& tangent = cached_path.tangents[static_cast<std::size_t>(index)];
        if (!tangent.valid) {
            return false;
        }

        const double path_heading_dot = tangent.x * vehicle_heading_x_ + tangent.y * vehicle_heading_y_;
        if (strict_path_heading_filter && path_heading_dot < path_heading_dot_min_) {
            return false;
        }

        const double distance_sq = dx * dx + dy * dy;
        const double angle_diff = normalizeAngle(tangent.yaw - vehicle_yaw_);
        const double cost = distance_weight_ * distance_sq + angle_weight_ * angle_diff * angle_diff;
        if (cost < best.cost) {
            best.valid = true;
            best.index = index;
            best.distance_sq = distance_sq;
            best.cost = cost;
            best.angle_diff = angle_diff;
        }
        return true;
    }

    SearchResult searchRange(const CachedPath& cached_path,
                             int start_index,
                             int end_index,
                             const bool use_wrap,
                             const bool strict_path_heading_filter) const
    {
        SearchResult best;
        if (!cached_path.path) {
            return best;
        }

        const int size = static_cast<int>(cached_path.path->poses.size());
        if (size < 2 || end_index < start_index) {
            return best;
        }

        if (!use_wrap) {
            start_index = std::max(0, start_index);
            end_index = std::min(size - 1, end_index);
            if (end_index < start_index) {
                return best;
            }
            for (int i = start_index; i <= end_index; ++i) {
                evaluateCandidate(cached_path, i, strict_path_heading_filter, best);
            }
            return best;
        }

        const int requested_count = end_index - start_index + 1;
        const int search_count = std::min(size, requested_count);
        for (int offset = 0; offset < search_count; ++offset) {
            const int index = wrapIndex(start_index + offset, size);
            evaluateCandidate(cached_path, index, strict_path_heading_filter, best);
        }
        return best;
    }

    SearchResult findBestIndex(const CachedPath& cached_path, TrackerState& tracker)
    {
        if (!cached_path.path) {
            return SearchResult();
        }

        const int size = static_cast<int>(cached_path.path->poses.size());
        if (size < 2) {
            return SearchResult();
        }

        SearchResult best;
        if (tracker.is_first_run) {
            best = searchRange(cached_path, 0, size - 1, false, true);
        } else {
            const int start_index = tracker.last_index - window_back_;
            const int end_index = tracker.last_index + window_fwd_;
            best = searchRange(cached_path, start_index, end_index, is_circular_path_, true);
        }

        if (!best.valid || best.distance_sq > max_valid_nearest_dist_sq_) {
            best = searchRange(cached_path, 0, size - 1, false, false);
        }

        if (!best.valid || best.distance_sq > max_valid_nearest_dist_sq_) {
            return SearchResult();
        }

        best.distance = std::sqrt(best.distance_sq);
        tracker.last_index = best.index;
        tracker.is_first_run = false;
        return best;
    }

    void transformPointToBaseLink(const double global_x,
                                  const double global_y,
                                  double& base_x,
                                  double& base_y) const
    {
        const double dx = global_x - current_position_.x;
        const double dy = global_y - current_position_.y;
        base_x = vehicle_heading_x_ * dx + vehicle_heading_y_ * dy;
        base_y = -vehicle_heading_y_ * dx + vehicle_heading_x_ * dy;
    }

    LocalPathResult buildLocalPath(const CachedPath& cached_path,
                                   TrackerState& tracker,
                                   const ros::Time& stamp,
                                   nav_msgs::Path& local_path)
    {
        LocalPathResult result;
        if (!cached_path.path) {
            return result;
        }

        const nav_msgs::Path& global_path = *cached_path.path;
        const int size = static_cast<int>(global_path.poses.size());
        if (size < 2) {
            return result;
        }

        const SearchResult nearest = findBestIndex(cached_path, tracker);
        if (!nearest.valid) {
            return result;
        }

        result.valid = true;
        result.best_index = nearest.index;
        result.best_distance = nearest.distance;
        result.cost = nearest.cost;

        int start_index = nearest.index - back_steps_;
        int end_index = nearest.index + front_steps_;

        int output_count = std::max(0, end_index - start_index);
        if (!is_circular_path_) {
            start_index = std::max(0, start_index);
            end_index = std::min(size, end_index);
            output_count = std::max(0, end_index - start_index);
        } else {
            output_count = std::min(size, output_count);
        }

        local_path.poses.reserve(static_cast<std::size_t>(output_count));
        for (int offset = 0; offset < output_count; ++offset) {
            const int raw_index = start_index + offset;
            const int index = is_circular_path_ ? wrapIndex(raw_index, size) : raw_index;
            if (index < 0 || index >= size) {
                break;
            }

            const auto& global_pose = global_path.poses[index].pose.position;
            double base_x = 0.0;
            double base_y = 0.0;
            transformPointToBaseLink(global_pose.x, global_pose.y, base_x, base_y);

            local_path.poses.emplace_back();
            geometry_msgs::PoseStamped& pose = local_path.poses.back();
            pose.header.frame_id = local_frame_id_;
            pose.header.stamp = stamp;
            pose.pose.position.x = base_x;
            pose.pose.position.y = base_y;
            pose.pose.position.z = global_pose.z;
            pose.pose.orientation.w = 1.0;
        }

        return result;
    }

    bool isLaneValid(const int lane, const LocalPathResult& path1, const LocalPathResult& path2) const
    {
        if (lane == 1) {
            return path1.valid;
        }
        if (lane == 2) {
            return path2.valid;
        }
        return false;
    }

    double laneCost(const int lane, const LocalPathResult& path1, const LocalPathResult& path2) const
    {
        if (lane == 1) {
            return path1.cost;
        }
        if (lane == 2) {
            return path2.cost;
        }
        return std::numeric_limits<double>::infinity();
    }

    int decideCurrentPath(const LocalPathResult& path1,
                          const LocalPathResult& path2,
                          const ros::Time& now)
    {
        if (!path1.valid && !path2.valid) {
            stable_current_path_ = 0;
            pending_path_ = 0;
            pending_count_ = 0;
            return 0;
        }

        int candidate_path = 0;
        if (path1.valid && path2.valid) {
            candidate_path = (path1.cost <= path2.cost) ? 1 : 2;
        } else {
            candidate_path = path1.valid ? 1 : 2;
        }

        if (!isLaneValid(stable_current_path_, path1, path2)) {
            stable_current_path_ = candidate_path;
            last_switch_time_ = now;
            pending_path_ = 0;
            pending_count_ = 0;
            return stable_current_path_;
        }

        if (candidate_path == stable_current_path_) {
            pending_path_ = 0;
            pending_count_ = 0;
            return stable_current_path_;
        }

        const double current_cost = laneCost(stable_current_path_, path1, path2);
        const double candidate_cost = laneCost(candidate_path, path1, path2);
        const bool hold_time_ok =
            last_switch_time_.isZero() ||
            (now - last_switch_time_).toSec() >= min_switch_interval_s_;
        const bool candidate_better = candidate_cost + switch_margin_cost_ < current_cost;

        if (candidate_better && hold_time_ok) {
            if (pending_path_ == candidate_path) {
                ++pending_count_;
            } else {
                pending_path_ = candidate_path;
                pending_count_ = 1;
            }

            if (pending_count_ >= better_count_required_) {
                stable_current_path_ = candidate_path;
                last_switch_time_ = now;
                pending_path_ = 0;
                pending_count_ = 0;
            }
        } else {
            pending_path_ = 0;
            pending_count_ = 0;
        }

        return stable_current_path_;
    }

    void publishCurrentPath(const int current_path)
    {
        std_msgs::Int32 msg;
        msg.data = current_path;
        current_path_pub_.publish(msg);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    std::string odom_topic_;
    std::string global_path1_topic_;
    std::string global_path2_topic_;
    std::string local_path1_topic_;
    std::string local_path2_topic_;
    std::string current_path_topic_;
    std::string local_frame_id_;

    double rate_hz_;
    double waypoint_interval_;
    double back_distance_;
    double front_distance_;
    int back_steps_;
    int front_steps_;
    int window_back_;
    int window_fwd_;
    bool is_circular_path_;

    double teleport_ego_dist_thresh_;
    double teleport_ego_dist_thresh_sq_;
    double max_valid_nearest_dist_;
    double max_valid_nearest_dist_sq_;
    double distance_weight_;
    double angle_weight_;
    double forward_dot_min_;
    double path_heading_dot_min_;

    double switch_margin_cost_;
    double min_switch_interval_s_;
    int better_count_required_;

    ros::Subscriber gps_sub_;
    ros::Subscriber global_path_1_sub_;
    ros::Subscriber global_path_2_sub_;
    ros::Publisher local_path1_pub_;
    ros::Publisher local_path2_pub_;
    ros::Publisher current_path_pub_;

    geometry_msgs::Point current_position_;
    double vehicle_yaw_;
    double vehicle_heading_x_;
    double vehicle_heading_y_;
    bool gps_state_;
    bool path_state_1_;
    bool path_state_2_;

    CachedPath global_path_1_cache_;
    CachedPath global_path_2_cache_;
    nav_msgs::Path local_path1_msg_;
    nav_msgs::Path local_path2_msg_;
    TrackerState path1_tracker_;
    TrackerState path2_tracker_;

    bool has_prev_ego_;
    double prev_ego_x_;
    double prev_ego_y_;

    int stable_current_path_;
    ros::Time last_switch_time_;
    int pending_path_;
    int pending_count_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_path_publisher");
    OdomPathPublisher node;
    node.spin();
    return 0;
}
