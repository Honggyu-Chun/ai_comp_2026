#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#include <katri_msgs/Objects.h>   // /obstacle_path_info (배열)

#include <vector>
#include <cmath>
#include <iostream>
#include <limits>
#include <iomanip>   // for std::setprecision
#include <algorithm> // for std::min

class PathSelect
{
public:
    // Subscribers
    ros::Subscriber lane_path_sub_, local_path1_sub_, local_path2_sub_;
    ros::Subscriber current_path_sub_, state_sub_, sign_state_sub_;
    ros::Subscriber objects_sub_;  // katri_msgs/Objects

    // Publishers
    ros::Publisher selected_path_pub_;
    ros::Publisher moving_obs_pub_;     // 동적 장애물 속도 지시 퍼블리셔

    // Paths
    nav_msgs::Path local_path1_, local_path2_, lane_path_;

    // State/flags
    std::string frame_id_ = "base_link";
    std::string state, sign_state;
    int current_path_ = 1;          // 외부 /current_path 로부터만 갱신
    int selected_path_number = 1;

    // 의도/명령 래치
    int  desired_path_        = 1;   // 정책적으로 가고 싶은 차선(1/2)
    int  commanded_path_      = -1;  // 방금 명령한 목표 차선(1/2). -1이면 없음
    bool hold_until_match_    = false; // /current_path가 commanded와 같아질 때까지 다른 로직 무시
    bool prev_static_         = false;

    // --- 정적 장애물 회피 경로(/avoid_path) 연동 ---
    // static_obstacle_avoider 가 발행하는 회피 경로. fresh 하면 selected_path 로 그대로 전달한다.
    // (avoid 노드는 무장애물 시 local_path1 을 passthrough 하므로 항상 유효한 주행 경로다.)
    ros::Subscriber avoid_path_sub_;
    nav_msgs::Path  avoid_path_;
    ros::Time       avoid_path_stamp_;
    bool            has_avoid_       = false;
    // avoid 노드가 "실제 shifting 중"임을 알리는 플래그(/avoid_active). NORMAL(passthrough)에는 false 라
    // selector 의 동적장애물(/moving_obs)·차선변경 로직을 죽이지 않는다.
    ros::Subscriber avoid_active_sub_;
    bool            avoid_active_    = false;
    ros::Time       avoid_active_stamp_;

    // 차선별 장애물 유무/최근접거리 (static_obs용 판단)
    bool obstacle_lane1_ = false;
    bool obstacle_lane2_ = false;
    double lane1_nearest_ = std::numeric_limits<double>::infinity();
    double lane2_nearest_ = std::numeric_limits<double>::infinity();

    // (디버그) 가장 가까운 장애물
    bool obstacle = false;
    int  obstacle_path_ = -1;
    double obstacle_distance_ = 1e9;

    // Objects 메시지 저장
    katri_msgs::Objects objects_msg_;

    // Simple pose (optional: 실제 odom/gps로 갱신 가능)
    geometry_msgs::Point current_pose_;

    // Params
    std::string objects_topic_ = "/obstacle_path_info";
    std::string lane_path_topic_ = "/lane_path";
    std::string local_path1_topic_ = "/local_path1";
    std::string local_path2_topic_ = "/local_path2";
    std::string current_path_topic_ = "/current_path";
    std::string state_topic_ = "/state";
    std::string sign_state_topic_ = "/sign_state";
    std::string selected_path_topic_ = "/selected_path";
    std::string moving_obs_topic_ = "/moving_obs";
    std::string avoid_path_topic_ = "/avoid_path";
    std::string avoid_active_topic_ = "/avoid_active";
    double obs_dist_threshold_  = 20.0; // static_obs 회피 판단 거리(차선 변경용)
    double rate_hz_ = 20.0;
    double lane_change_interpolate_length_ = 2.0;
    int lane_change_interpolate_points_ = 10;
    int subscribe_queue_size_ = 10;
    int publish_queue_size_ = 1;
    bool debug_print_ = true;
    bool   use_avoid_path_  = true;
    double avoid_timeout_   = 0.5;   // [s] 이 시간 넘게 안 오면 lane 로직 fallback

    // ==== 동적 장애물 속도 지시용 파라미터/디버그 ====
    double slow_down_dist_      = 16.0; // 동적: 감속 시작
    double stop_dist_           = 9.0;  // 동적: 정지
    bool   has_front_obj_       = false;
    double front_obj_dist_      = std::numeric_limits<double>::infinity();
    std::string last_moving_cmd_= "none";

    PathSelect()
    {
        ros::NodeHandle pnh("~");
        pnh.param("objects_topic",       objects_topic_,       objects_topic_);
        pnh.param("lane_path_topic",     lane_path_topic_,     lane_path_topic_);
        pnh.param("local_path1_topic",   local_path1_topic_,   local_path1_topic_);
        pnh.param("local_path2_topic",   local_path2_topic_,   local_path2_topic_);
        pnh.param("current_path_topic",  current_path_topic_,  current_path_topic_);
        pnh.param("state_topic",         state_topic_,         state_topic_);
        pnh.param("sign_state_topic",    sign_state_topic_,    sign_state_topic_);
        pnh.param("selected_path_topic", selected_path_topic_, selected_path_topic_);
        pnh.param("moving_obs_topic",    moving_obs_topic_,    moving_obs_topic_);
        pnh.param("avoid_path_topic",    avoid_path_topic_,    avoid_path_topic_);
        pnh.param("avoid_active_topic",  avoid_active_topic_,  avoid_active_topic_);
        pnh.param("obs_dist_threshold",  obs_dist_threshold_,  obs_dist_threshold_);
        pnh.param("slow_down_dist",      slow_down_dist_,      slow_down_dist_);
        pnh.param("stop_dist",           stop_dist_,           stop_dist_);
        pnh.param("rate_hz",             rate_hz_,             rate_hz_);
        pnh.param("lane_change_interpolate_length", lane_change_interpolate_length_, lane_change_interpolate_length_);
        pnh.param("lane_change_interpolate_points", lane_change_interpolate_points_, lane_change_interpolate_points_);
        pnh.param("subscribe_queue_size", subscribe_queue_size_, subscribe_queue_size_);
        pnh.param("publish_queue_size",   publish_queue_size_,   publish_queue_size_);
        pnh.param("debug_print",          debug_print_,          debug_print_);
        pnh.param("use_avoid_path",       use_avoid_path_,       use_avoid_path_);
        pnh.param("avoid_timeout",        avoid_timeout_,        avoid_timeout_);

        rate_hz_ = std::max(1.0, rate_hz_);
        lane_change_interpolate_length_ = std::max(0.1, lane_change_interpolate_length_);
        lane_change_interpolate_points_ = std::max(2, lane_change_interpolate_points_);
        subscribe_queue_size_ = std::max(1, subscribe_queue_size_);
        publish_queue_size_ = std::max(1, publish_queue_size_);

        ros::NodeHandle nh;

        lane_path_sub_    = nh.subscribe(lane_path_topic_,    subscribe_queue_size_, &PathSelect::LanePathCB, this);
        local_path1_sub_  = nh.subscribe(local_path1_topic_,  subscribe_queue_size_, &PathSelect::LocalPath1CB, this);
        local_path2_sub_  = nh.subscribe(local_path2_topic_,  subscribe_queue_size_, &PathSelect::LocalPath2CB, this);
        current_path_sub_ = nh.subscribe(current_path_topic_, subscribe_queue_size_, &PathSelect::CurrentPathCB, this);
        state_sub_        = nh.subscribe(state_topic_,        subscribe_queue_size_, &PathSelect::StateCB, this);
        sign_state_sub_   = nh.subscribe(sign_state_topic_,   subscribe_queue_size_, &PathSelect::SignStateCB, this);

        objects_sub_      = nh.subscribe(objects_topic_,   subscribe_queue_size_, &PathSelect::ObjectsCB, this);

        avoid_path_sub_   = nh.subscribe(avoid_path_topic_,   subscribe_queue_size_, &PathSelect::AvoidPathCB, this);
        avoid_active_sub_ = nh.subscribe(avoid_active_topic_, subscribe_queue_size_, &PathSelect::AvoidActiveCB, this);

        selected_path_pub_ = nh.advertise<nav_msgs::Path>(selected_path_topic_, publish_queue_size_);
        moving_obs_pub_    = nh.advertise<std_msgs::String>(moving_obs_topic_, publish_queue_size_);
    }

    // --- Callbacks ---
    void AvoidPathCB(const nav_msgs::Path::ConstPtr &msg) {
        avoid_path_ = *msg;
        avoid_path_stamp_ = ros::Time::now();
        has_avoid_ = true;
    }
    void AvoidActiveCB(const std_msgs::Bool::ConstPtr &msg) {
        avoid_active_ = msg->data;
        avoid_active_stamp_ = ros::Time::now();
    }
    // avoid 노드가 "실제 회피 중(active)"이고 fresh·비어있지 않은 /avoid_path 가 있으면 selected_path 로
    // 전달하고 true. NORMAL(passthrough)·미실행·stale·빈경로면 false → 기존 lane 로직 fallback.
    bool tryPublishAvoidPath() {
        if (!use_avoid_path_ || !has_avoid_) return false;
        if (avoid_path_.poses.empty()) return false;                      // 빈 경로 전파 방지
        const ros::Time now = ros::Time::now();
        if ((now - avoid_path_stamp_).toSec() > avoid_timeout_) return false;
        if (!avoid_active_ || (now - avoid_active_stamp_).toSec() > avoid_timeout_) return false;
        avoid_path_.header.stamp = now;
        selected_path_pub_.publish(avoid_path_);
        return true;
    }
    void LanePathCB(const nav_msgs::Path::ConstPtr &msg)   { lane_path_  = *msg; }
    void LocalPath1CB(const nav_msgs::Path::ConstPtr &msg) { local_path1_ = *msg; }
    void LocalPath2CB(const nav_msgs::Path::ConstPtr &msg) { local_path2_ = *msg; }

    void CurrentPathCB(const std_msgs::Int32::ConstPtr &msg)
    {
        current_path_ = msg->data;

        // 차선 변경 완료 확인: commanded_path와 현재가 일치하면 hold 해제
        if (hold_until_match_ && commanded_path_ == current_path_)
        {
            hold_until_match_ = false;
            desired_path_     = current_path_; // 의도도 현재에 맞춰 동기화
            commanded_path_   = -1;
        }
    }

    void StateCB(const std_msgs::String::ConstPtr &msg)    { state = msg->data; }
    void SignStateCB(const std_msgs::String::ConstPtr &msg){ sign_state = msg->data; }

    void ObjectsCB(const katri_msgs::Objects::ConstPtr &msg)
    {
        objects_msg_ = *msg;
    }

    // --- Helper: 현재 차선 전방(+)의 "가장 가까운" 객체 거리 계산 (동적용) ---
    bool getFrontObjectDistance(double &nearest_dist) const
    {
        nearest_dist = std::numeric_limits<double>::infinity();
        bool found = false;

        if (objects_msg_.objects.empty()) return false;

        for (const auto& obj : objects_msg_.objects)
        {
            const bool same_lane = (obj.path_number == current_path_);
            const bool is_front  = (obj.distance > 0.0) || (obj.obj_position == "front");
            if (debug_print_) {
                std::cout<<"obj.distance\t"<<obj.distance<<"\n";
                std::cout << "is_front\t" << std::boolalpha << is_front << "\n";
                std::cout << "same_lane\t" << std::boolalpha << same_lane << "\n";
            }

            if (!same_lane || !is_front) continue;

            if (obj.distance > 0.0 && obj.distance < nearest_dist)
            {
                nearest_dist = obj.distance;
                found = true;
            }
        }
        return found;
    }

    // --- Obstacle check for static_obs (차선별 판단/토글 근거) ---
    void obstacle_check_static()
    {
        // 디버그 필드 초기화
        obstacle = false;
        obstacle_path_ = -1;
        obstacle_distance_ = 1e9;

        // 차선별 초기화
        obstacle_lane1_ = false;
        obstacle_lane2_ = false;
        lane1_nearest_  = std::numeric_limits<double>::infinity();
        lane2_nearest_  = std::numeric_limits<double>::infinity();

        if (state != "static_obs") return;
        if (objects_msg_.objects.empty()) return;

        for (const auto& obj : objects_msg_.objects)
        {
            const bool is_front  = (obj.distance > 0.0) || (obj.obj_position == "front");
            const bool within_th = (obj.distance > 0.0 && obj.distance <= obs_dist_threshold_);
            if (!is_front) continue;

            if (within_th && obj.distance < obstacle_distance_) {
                obstacle = true;
                obstacle_path_ = obj.path_number;
                obstacle_distance_ = obj.distance;
            }

            if (within_th)
            {
                if (obj.path_number == 1) {
                    obstacle_lane1_ = true;
                    lane1_nearest_  = std::min(lane1_nearest_, obj.distance);
                } else if (obj.path_number == 2) {
                    obstacle_lane2_ = true;
                    lane2_nearest_  = std::min(lane2_nearest_, obj.distance);
                }
            }
        }
    }

    // --- [동적] 동적 장애물(비 static_obs) 속도지시 퍼블리시 ---
    void publish_moving_obs_cmd()
    {
        // gps_fail이나 static_obs 중엔 퍼블리시하지 않음
        if (state == "gps_fail" || state == "static_obs") return;

        std_msgs::String cmd;
        double d;
        bool found = getFrontObjectDistance(d);

        has_front_obj_  = found;
        front_obj_dist_ = found ? d : std::numeric_limits<double>::infinity();
        if (debug_print_) {
            std::cout<<"front_obj_dist_\t"<<front_obj_dist_<<"\n";
        }
        if (!found) {
            cmd.data = "go";
        } else {
            if (d < stop_dist_)           cmd.data = "stop";
            else if (d < slow_down_dist_) cmd.data = "slow_down";
            else                          cmd.data = "go";
        }
        last_moving_cmd_ = cmd.data;  // 상태 저장
        moving_obs_pub_.publish(cmd);
    }

    // --- Publish helpers ---
    void publishPathByNumber(int num)
    {
        nav_msgs::Path out;
        if (num == 1) {
            out = local_path1_;
            selected_path_number = 1;
        } else if (num == 2) {
            out = local_path2_;
            selected_path_number = 2;
        } else {
            ROS_WARN_THROTTLE(1.0, "publishPathByNumber: invalid path %d", num);
            return;
        }
        out.header.frame_id = frame_id_;
        out.header.stamp = ros::Time::now();
        selected_path_pub_.publish(out);
    }

    void publishLanePath()
    {
        nav_msgs::Path out = lane_path_;
        out.header.frame_id = frame_id_;
        out.header.stamp = ros::Time::now();
        selected_path_pub_.publish(out);
        selected_path_number = 0; // 디버그용(0 = lane_path)
    }

    // 스플라인으로 목표 차선으로 전환 명령 + 홀드 래치 ON
    void command_change_to(int target_path)
    {
        nav_msgs::Path path;
        if (target_path == 1) {
            path.poses = cubicInterpolatePath(local_path1_, lane_change_interpolate_points_);
            selected_path_number = 1;
        } else if (target_path == 2) {
            path.poses = cubicInterpolatePath(local_path2_, lane_change_interpolate_points_);
            selected_path_number = 2;
        } else {
            ROS_WARN("command_change_to: invalid target_path: %d", target_path);
            return;
        }

        path.header.frame_id = frame_id_;
        path.header.stamp = ros::Time::now();
        selected_path_pub_.publish(path);

        // 래치: 실제 current_path가 바뀌었다고 올 때까지 다른 로직 무시
        commanded_path_   = target_path;
        hold_until_match_ = true;

        // 의도도 목표에 맞춰 둠
        desired_path_ = target_path;
    }

    // --- Simple cubic interpolation to make a smooth lane change segment ---
    std::vector<geometry_msgs::PoseStamped> cubicInterpolatePath(const nav_msgs::Path &path, int num_points)
    {
        std::vector<geometry_msgs::PoseStamped> out;
        const double interpolate_length = lane_change_interpolate_length_;

        if (path.poses.empty()) {
            ROS_WARN_THROTTLE(1.0, "cubicInterpolatePath: empty path");
            return out;
        }

        int end_idx = -1;
        for (size_t i = 0; i < path.poses.size(); ++i)
        {
            double dx = path.poses[i].pose.position.x - current_pose_.x;
            if (dx >= interpolate_length) { end_idx = static_cast<int>(i); break; }
        }
        if (end_idx <= 0) {
            ROS_WARN_THROTTLE(1.0, "cubicInterpolatePath: too short path or no forward point");
            return out;
        }

        // start at (0,0) in base_link; end at end_idx point
        double x0 = 0.0, y0 = 0.0;
        double x1 = path.poses[end_idx].pose.position.x;
        double y1 = path.poses[end_idx].pose.position.y;

        double dx = x1 - x0; if (dx <= 0.0) dx = std::fabs(dx) + 1e-3;
        double end_slope = 0.0;
        if (end_idx + 1 < (int)path.poses.size())
        {
            double nx = path.poses[end_idx + 1].pose.position.x;
            double ny = path.poses[end_idx + 1].pose.position.y;
            double ndx = (nx - x1);
            end_slope = (std::fabs(ndx) > 1e-6) ? (ny - y1) / ndx : 0.0;
        }

        const double c0 = y0;
        const double c1 = 0.0; // start slope
        const double c2 = (3 * (y1 - y0) / (dx * dx)) - ((end_slope + 2 * c1) / dx);
        const double c3 = (2 * (y0 - y1) / (dx * dx * dx)) + ((end_slope + c1) / (dx * dx));

        out.reserve(num_points + (path.poses.size() - end_idx));
        for (int i = 0; i < num_points; ++i)
        {
            double t = (num_points == 1) ? 0.0 : (double)i / (num_points - 1);
            double x = x0 + t * dx;
            double y = c0 + c1 * (x - x0) + c2 * std::pow((x - x0), 2) + c3 * std::pow((x - x0), 3);

            geometry_msgs::PoseStamped p;
            p.header.frame_id = frame_id_;
            p.header.stamp = ros::Time::now();
            p.pose.position.x = x;
            p.pose.position.y = y;
            p.pose.orientation.w = 1.0;
            out.push_back(p);
        }
        for (size_t i = end_idx; i < path.poses.size(); ++i)
        {
            auto p = path.poses[i];
            p.header.frame_id = frame_id_;
            out.push_back(p);
        }
        return out;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_path_selector");
    PathSelect ps;
    ros::Rate rate(ps.rate_hz_);

    while (ros::ok())
    {
        ros::spinOnce();

        const bool cur_static     = (ps.state == "static_obs");
        const bool entered_static = (cur_static && !ps.prev_static_);
        const bool left_static    = (!cur_static && ps.prev_static_);
        ps.prev_static_ = cur_static;

        // === gps_fail 우선 처리: 무조건 lane_path 추종, 홀드 해제, 다른 로직 무시 ===
        if (ps.state == "gps_fail")
        {
            ps.hold_until_match_ = false;
            ps.commanded_path_   = -1;
            ps.publishLanePath();
            rate.sleep();
            continue;
        }

        // === 정적 장애물 회피 경로 우선: static_obstacle_avoider 의 /avoid_path 가 fresh 하면
        //     그대로 selected_path 로 전달(무장애물 시 local_path1 passthrough라 안전). ===
        if (ps.tryPublishAvoidPath())
        {
            rate.sleep();
            continue;
        }

        // --- 변경 진행 중이면 다른 로직은 무시하고 목표 차선 경로만 계속 퍼블리시 ---
        if (ps.hold_until_match_)
        {
            ps.publishPathByNumber(ps.commanded_path_);
            rate.sleep();
            continue;
        }

        // --- 상태 전이 처리 ---
        if (entered_static)
        {
            // static_obs에 들어갈 때 기본 의도는 현재 차선을 유지
            ps.desired_path_ = ps.current_path_;
        }
        if (left_static)
        {
            // static_obs가 끝나면 무조건 1번 차선으로 복귀
            if (ps.current_path_ != 1) {
                ps.desired_path_ = 1;
                ps.command_change_to(1);
                ps.publishPathByNumber(1);
                rate.sleep();
                continue; // 변경 완료 전까지 hold에서 처리
            } else {
                ps.desired_path_ = 1;
                ps.publishPathByNumber(1);
                ps.publish_moving_obs_cmd();
                rate.sleep();
                continue;
            }
        }

        // --- static_obs 중 로직 ---
        ps.obstacle_check_static();

        if (cur_static)
        {
            // 1번 주행 중 1번 차선에 장애물이 있으면 2번 차선으로 변경하고 hold
            if (ps.current_path_ == 1 && ps.obstacle_lane1_)
            {
                ps.desired_path_ = 2;
                ps.command_change_to(2);
                ps.publishPathByNumber(2);
                rate.sleep();
                continue; // 변경 완료 전까지 hold
            }

            // 2번 주행 중 2번 차선에 장애물이 있으면 1번 차선으로 변경하고 hold
            if (ps.current_path_ == 2 && ps.obstacle_lane2_)
            {
                ps.desired_path_ = 1;
                ps.command_change_to(1);
                ps.publishPathByNumber(1);
                rate.sleep();
                continue; // 변경 완료 전까지 hold
            }

            // 장애물이 없으면 현재 차선 유지
            ps.desired_path_ = ps.current_path_;
            ps.publishPathByNumber(ps.current_path_);
            // static_obs 중에는 /moving_obs를 퍼블리시하지 않음
        }
        else
        {
            // static_obs가 아니면 무조건 1번 차선 추종
            if (ps.current_path_ != 1)
            {
                ps.desired_path_ = 1;
                ps.command_change_to(1);
                ps.publishPathByNumber(1);
            }
            else
            {
                ps.desired_path_ = 1;
                ps.publishPathByNumber(1);
            }

            // [동적] 전방 거리 기반 속도 지시 퍼블리시 (go/slow_down/stop)
            ps.publish_moving_obs_cmd();
        }

        if (ps.debug_print_) {
            std::cout << "State=" << ps.state
                      << " | current=" << ps.current_path_
                      << std::endl;
                    //   << " | desired=" << ps.desired_path_
                    //   << " | commanded=" << ps.commanded_path_
            std::cout << "hold=" << std::boolalpha << ps.hold_until_match_
                      << " | selected=" << ps.selected_path_number
                      << std::endl;

            std::cout << "Lane1 obs=" << ps.obstacle_lane1_
                      << " (nearest=" << (std::isinf(ps.lane1_nearest_) ? -1.0 : ps.lane1_nearest_) << " m)"
                      << " | Lane2 obs=" << ps.obstacle_lane2_
                      << " (nearest=" << (std::isinf(ps.lane2_nearest_) ? -1.0 : ps.lane2_nearest_) << " m)"
                      << std::endl;

            std::cout << "moving_obj_dist=";
            if (ps.has_front_obj_) {
                std::cout << std::fixed << std::setprecision(2) << ps.front_obj_dist_ << " m";
            } else {
                std::cout << "INF";
            }
            std::cout << " | obs state=" << ps.last_moving_cmd_ << std::endl;

            std::cout << "Obstacle(static): " << std::boolalpha << ps.obstacle
                      << " (path=" << ps.obstacle_path_
                      << ", dist=" << ps.obstacle_distance_ << ")" << std::endl;

            std::cout << "=========================================== " << std::endl;
        }

        rate.sleep();
    }
    return 0;
}
