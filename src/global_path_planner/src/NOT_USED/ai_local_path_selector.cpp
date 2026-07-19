// // file: src/ai_local_path_selector.cpp

// #include <ros/ros.h>
// #include <nav_msgs/Path.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/Point.h>
// #include <std_msgs/Int32.h>
// #include <std_msgs/String.h>

// #include <katri_msgs/Objects.h>   // /obstacle_path_info (배열)

// #include <vector>
// #include <cmath>
// #include <iostream>
// #include <limits>
// #include <iomanip>   // for std::setprecision

// class PathSelect
// {
// public:
//     // Subscribers
//     ros::Subscriber lane_path_sub_, local_path1_sub_, local_path2_sub_;
//     ros::Subscriber current_path_sub_, state_sub_, sign_state_sub_;
//     ros::Subscriber objects_sub_;  // katri_msgs/Objects

//     // Publishers
//     ros::Publisher selected_path_pub_;
//     ros::Publisher moving_obs_pub_;     // 동적 장애물 속도 지시 퍼블리셔

//     // Paths
//     nav_msgs::Path local_path1_, local_path2_, lane_path_;

//     // State/flags
//     std::string frame_id_ = "base_link";
//     std::string state, sign_state;
//     int current_path_ = 1;
//     int selected_path_number = 1;

//     // Obstacle info (from katri_msgs/Objects)
//     katri_msgs::Objects objects_msg_;
//     bool obstacle = false;          // 정지장애물(static_obs) 판단용
//     int obstacle_path_ = -1;
//     double obstacle_distance_ = 1e9;

//     // Dynamic obstacle debug states
//     bool   has_front_obj_ = false;                                    // 전방 객체 유무
//     double front_obj_dist_ = std::numeric_limits<double>::infinity(); // 전방 최근접 거리
//     std::string last_moving_cmd_ = "none";                            // 마지막 /moving_obs 명령

//     // Simple pose (optional: 실제 odom/gps로 갱신 가능)
//     geometry_msgs::Point current_pose_;

//     // Params
//     std::string objects_topic_ = "/obstacle_path_info";
//     double obs_dist_threshold_  = 15.0; // static_obs 회피 판단 거리(차선 변경용)
//     double slow_down_dist_      = 15.0; // 동적: 감속 시작
//     double stop_dist_           = 8.0;  // 동적: 정지

//     PathSelect()
//     {
//         // private nh로 파라미터 로드
//         ros::NodeHandle pnh("~");
//         pnh.param("objects_topic",       objects_topic_,       objects_topic_);
//         pnh.param("obs_dist_threshold",  obs_dist_threshold_,  obs_dist_threshold_);
//         pnh.param("slow_down_dist",      slow_down_dist_,      slow_down_dist_);
//         pnh.param("stop_dist",           stop_dist_,           stop_dist_);

//         ros::NodeHandle nh;

//         lane_path_sub_    = nh.subscribe("/lane_path",    10, &PathSelect::LanePathCB, this);
//         local_path1_sub_  = nh.subscribe("/local_path1",  10, &PathSelect::LocalPath1CB, this);
//         local_path2_sub_  = nh.subscribe("/local_path2",  10, &PathSelect::LocalPath2CB, this);
//         current_path_sub_ = nh.subscribe("/current_path", 10, &PathSelect::CurrentPathCB, this);
//         state_sub_        = nh.subscribe("/state",        10, &PathSelect::StateCB, this);
//         sign_state_sub_   = nh.subscribe("/sign_state",   10, &PathSelect::SignStateCB, this);

//         // katri_msgs/Objects 배열 구독
//         objects_sub_      = nh.subscribe(objects_topic_,   10, &PathSelect::ObjectsCB, this);

//         selected_path_pub_ = nh.advertise<nav_msgs::Path>("/selected_path", 1);
//         moving_obs_pub_    = nh.advertise<std_msgs::String>("/moving_obs", 1);
//     }

//     // --- Callbacks ---
//     void LanePathCB(const nav_msgs::Path::ConstPtr &msg)   { lane_path_  = *msg; }
//     void LocalPath1CB(const nav_msgs::Path::ConstPtr &msg) { local_path1_ = *msg; }
//     void LocalPath2CB(const nav_msgs::Path::ConstPtr &msg) { local_path2_ = *msg; }
//     void CurrentPathCB(const std_msgs::Int32::ConstPtr &msg){ current_path_ = msg->data; }
//     void StateCB(const std_msgs::String::ConstPtr &msg)    { state = msg->data; }
//     void SignStateCB(const std_msgs::String::ConstPtr &msg){ sign_state = msg->data; }

//     void ObjectsCB(const katri_msgs::Objects::ConstPtr &msg)
//     {
//         objects_msg_ = *msg;
//     }

//     // --- Helper: 현재 차선 전방(+)의 "가장 가까운" 객체 거리 계산 (있으면 true/거리 반환) ---
//     bool getFrontObjectDistance(double &nearest_dist) const
//     {
//         nearest_dist = std::numeric_limits<double>::infinity();
//         bool found = false;

//         if (objects_msg_.objects.empty()) return false;

//         for (const auto& obj : objects_msg_.objects)
//         {
//             const bool same_lane = (obj.path_number == current_path_);
//             const bool is_front  = (obj.distance > 0.0) || (obj.obj_position == "front");
//             if (!same_lane || !is_front) continue;

//             if (obj.distance > 0.0 && obj.distance < nearest_dist)
//             {
//                 nearest_dist = obj.distance;
//                 found = true;
//             }
//         }
//         return found;
//     }

//     // --- Obstacle check for static_obs (차선 변경 여부 판단) ---
//     void obstacle_check_static()
//     {
//         obstacle = false;
//         obstacle_path_ = -1;
//         obstacle_distance_ = 1e9;

//         if (state != "static_obs") return;             // 정지장애물 상황에서만 회피
//         if (objects_msg_.objects.empty()) return;

//         // 같은 차선 + 전방 + obs_dist_threshold_ 이내 중 가장 가까운 것 선택
//         for (const auto& obj : objects_msg_.objects)
//         {
//             const bool same_lane = (obj.path_number == current_path_);
//             const bool is_front  = (obj.distance > 0.0) || (obj.obj_position == "front");
//             const bool within_th = (obj.distance > 0.0 && obj.distance <= obs_dist_threshold_);

//             if (same_lane && is_front && within_th)
//             {
//                 if (obj.distance < obstacle_distance_)
//                 {
//                     obstacle = true;
//                     obstacle_path_ = obj.path_number;
//                     obstacle_distance_ = obj.distance;
//                 }
//             }
//         }
//     }

//     // --- Publish helper ---
//     void publishSelectedPath(nav_msgs::Path& path)
//     {
//         path.header.frame_id = frame_id_;
//         path.header.stamp = ros::Time::now();
//         selected_path_pub_.publish(path);
//     }

//     // --- Simple cubic interpolation to make a smooth lane change segment ---
//     std::vector<geometry_msgs::PoseStamped> cubicInterpolatePath(const nav_msgs::Path &path, int num_points)
//     {
//         std::vector<geometry_msgs::PoseStamped> out;
//         const double interpolate_length = 2.0;

//         if (path.poses.empty()) {
//             ROS_WARN_THROTTLE(1.0, "cubicInterpolatePath: empty path");
//             return out;
//         }

//         int end_idx = -1;
//         for (size_t i = 0; i < path.poses.size(); ++i)
//         {
//             double dx = path.poses[i].pose.position.x - current_pose_.x;
//             if (dx >= interpolate_length) { end_idx = static_cast<int>(i); break; }
//         }
//         if (end_idx <= 0) {
//             ROS_WARN_THROTTLE(1.0, "cubicInterpolatePath: too short path or no forward point");
//             return out;
//         }

//         // start at (0,0) in base_link; end at end_idx point
//         double x0 = 0.0, y0 = 0.0;
//         double x1 = path.poses[end_idx].pose.position.x;
//         double y1 = path.poses[end_idx].pose.position.y;

//         double dx = x1 - x0; if (dx <= 0.0) dx = std::fabs(dx) + 1e-3;
//         double end_slope = 0.0;
//         if (end_idx + 1 < (int)path.poses.size())
//         {
//             double nx = path.poses[end_idx + 1].pose.position.x;
//             double ny = path.poses[end_idx + 1].pose.position.y;
//             double ndx = (nx - x1);
//             end_slope = (std::fabs(ndx) > 1e-6) ? (ny - y1) / ndx : 0.0;
//         }

//         const double c0 = y0;
//         const double c1 = 0.0; // start slope
//         const double c2 = (3 * (y1 - y0) / (dx * dx)) - ((end_slope + 2 * c1) / dx);
//         const double c3 = (2 * (y0 - y1) / (dx * dx * dx)) + ((end_slope + c1) / (dx * dx));

//         out.reserve(num_points + (path.poses.size() - end_idx));
//         for (int i = 0; i < num_points; ++i)
//         {
//             double t = (num_points == 1) ? 0.0 : (double)i / (num_points - 1);
//             double x = x0 + t * dx;
//             double y = c0 + c1 * (x - x0) + c2 * std::pow((x - x0), 2) + c3 * std::pow((x - x0), 3);

//             geometry_msgs::PoseStamped p;
//             p.header.frame_id = frame_id_;
//             p.header.stamp = ros::Time::now();
//             p.pose.position.x = x;
//             p.pose.position.y = y;
//             p.pose.orientation.w = 1.0;
//             out.push_back(p);
//         }
//         for (size_t i = end_idx; i < path.poses.size(); ++i)
//         {
//             auto p = path.poses[i];
//             p.header.frame_id = frame_id_;
//             out.push_back(p);
//         }
//         return out;
//     }

//     // current_path 기준 반대 차선으로 스플라인 전환
//     void lane_change(int current_path)
//     {
//         current_path_ = current_path;

//         nav_msgs::Path path;
//         int next_path = (current_path_ == 1) ? 2 :
//                         (current_path_ == 2) ? 1 : -1;

//         if (next_path == 1)
//         {
//             path.poses = cubicInterpolatePath(local_path1_, 10);
//             selected_path_number = 1;
//         }
//         else if (next_path == 2)
//         {
//             path.poses = cubicInterpolatePath(local_path2_, 10);
//             selected_path_number = 2;
//         }
//         else
//         {
//             ROS_WARN("lane_change: invalid current_path_: %d", current_path_);
//             return;
//         }
//         publishSelectedPath(path);
//     }

//     // --- 동적 장애물(비 static_obs) 속도지시 퍼블리시 ---
//     void publish_moving_obs_cmd()
//     {
//         state = "go";
//         return ;
//         if(state != "gps_fail")
//         {
//             std_msgs::String cmd;
//             double d;
//             bool found = getFrontObjectDistance(d);

//             has_front_obj_  = found;
//             front_obj_dist_ = found ? d : std::numeric_limits<double>::infinity();

//             if (!found) {
//                 cmd.data = "go";
//             } else {
//                 if (d < stop_dist_)           cmd.data = "stop";
//                 else if (d < slow_down_dist_) cmd.data = "slow_down";
//                 else                          cmd.data = "go";
//             }
//             last_moving_cmd_ = cmd.data;  // 상태 저장
//             moving_obs_pub_.publish(cmd);
//         }
//         else
//         {
//             state = "go";
//         }
//     }
// };

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "ai_local_path_selector");
//     PathSelect ps;
//     ros::Rate rate(20);

//     while (ros::ok())
//     {
//         ros::spinOnce();

//         // 1) static_obs일 때: 정지장애물 회피 판단 & 경로 선택
//         ps.obstacle_check_static();

//         if (ps.state == "gps_fail")
//         {
//             ps.publishSelectedPath(ps.lane_path_);
//         }
//         else if (ps.state == "static_obs")
//         {
//             // 정지장애물 회피: 1번 차선 주행 중이면 2번으로 변경
//             if (ps.obstacle)
//             {
//                 if (ps.current_path_ == 1)
//                 {
//                     ps.lane_change(1);   // 1 -> 2
//                 }
//                 else if(ps.current_path_ == 2)
//                 {
//                     ps.lane_change(2);
//                 }
//                 else
//                 {
//                     // 이미 2번이면 2번 경로 유지
//                     ps.publishSelectedPath(ps.local_path2_);
//                     ps.selected_path_number = 2;
//                 }
//             }
//             else
//             {
//                 // 장애물 플래그 없으면 현재 차선 유지(보수적)
//                 if (ps.current_path_ == 2)
//                 {
//                     ps.publishSelectedPath(ps.local_path2_);
//                     ps.selected_path_number = 2;
//                 }
//                 else
//                 {
//                     ps.publishSelectedPath(ps.local_path1_);
//                     ps.selected_path_number = 1;
//                 }
//             }
//             // 요구사항: static_obs 구간이 아닐 때만 moving_obs 지시를 보냄 → 여기선 안 보냄
//         }
//         else
//         {
//             // 2) static_obs가 아닐 때: 기본 주행 경로 유지(또는 go 로직) + 동적 장애물 속도지시 퍼블리시
//             if (ps.state == "go")
//             {
//                 ps.publishSelectedPath(ps.local_path1_);
//                 ps.selected_path_number = 1;
//             }
//             else if (ps.current_path_ == 2)
//             {
//                 // 필요 시 2->1 복귀 (원래 로직에 맞춤)
//                 ps.lane_change(2); // 2 -> 1 복귀 가정
//             }
//             else
//             {
//                 ps.publishSelectedPath(ps.local_path1_);
//                 ps.selected_path_number = 1;
//             }

//             // 동적 장애물 거리 유지 지시: /moving_obs = stop/slow_down/go
//             ps.publish_moving_obs_cmd();
//         }

//         // Debug print
//         std::cout << "Current Path : " << ps.current_path_
//                   << " | Selected Path : " << ps.selected_path_number << std::endl;
//         std::cout << "Obstacle(static): " << std::boolalpha << ps.obstacle
//                   << " (path=" << ps.obstacle_path_
//                   << ", dist=" << ps.obstacle_distance_ << ")" <<std::endl;
//         std::cout << "State : " << ps.state << std::endl;

//         // --- 추가: 동적 장애물 정보 ---
//         std::cout << "moving_obj_dist=";
//         if (ps.has_front_obj_) {
//             std::cout << std::fixed << std::setprecision(2) << ps.front_obj_dist_ << " m";
//         } else {
//             std::cout << "INF";
//         }
//         std::cout << " | obs state=" << ps.last_moving_cmd_ << std::endl;

//         std::cout << "=========================================== " << std::endl;

//         rate.sleep();
//     }
//     return 0;
// }

// // file: src/ai_local_path_selector.cpp

// #include <ros/ros.h>
// #include <nav_msgs/Path.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/Point.h>
// #include <std_msgs/Int32.h>
// #include <std_msgs/String.h>

// #include <katri_msgs/Objects.h>   // /obstacle_path_info (배열)

// #include <vector>
// #include <cmath>
// #include <iostream>
// #include <limits>
// #include <iomanip>   // for std::setprecision
// #include <algorithm> // for std::min

// class PathSelect
// {
// public:
//     // Subscribers
//     ros::Subscriber lane_path_sub_, local_path1_sub_, local_path2_sub_;
//     ros::Subscriber current_path_sub_, state_sub_, sign_state_sub_;
//     ros::Subscriber objects_sub_;  // katri_msgs/Objects

//     // Publishers
//     ros::Publisher selected_path_pub_;
//     ros::Publisher moving_obs_pub_;     // 동적 장애물 속도 지시 퍼블리셔

//     // Paths
//     nav_msgs::Path local_path1_, local_path2_, lane_path_;

//     // State/flags
//     std::string frame_id_ = "base_link";
//     std::string state, sign_state;
//     int current_path_ = 1;          // 외부 /current_path 로부터만 갱신
//     int selected_path_number = 1;

//     // 의도/명령 래치
//     int  desired_path_        = 1;   // 정책적으로 가고 싶은 차선(1/2)
//     int  commanded_path_      = -1;  // 방금 명령한 목표 차선(1/2). -1이면 없음
//     bool hold_until_match_    = false; // /current_path가 commanded와 같아질 때까지 다른 로직 무시
//     bool prev_static_         = false;

//     // 차선별 장애물 유무/최근접거리 (static_obs용 판단)
//     bool obstacle_lane1_ = false;
//     bool obstacle_lane2_ = false;
//     double lane1_nearest_ = std::numeric_limits<double>::infinity();
//     double lane2_nearest_ = std::numeric_limits<double>::infinity();

//     // (디버그) 가장 가까운 장애물 (임계거리 내)
//     bool obstacle = false;
//     int  obstacle_path_ = -1;
//     double obstacle_distance_ = 1e9;

//     // Objects 메시지 저장
//     katri_msgs::Objects objects_msg_;

//     // Simple pose (optional: 실제 odom/gps로 갱신 가능)
//     geometry_msgs::Point current_pose_;

//     // Params
//     std::string objects_topic_ = "/obstacle_path_info";
//     double obs_dist_threshold_  = 20.0; // static_obs 회피 판단 거리(차선 변경용)

//     // ==== 동적 장애물 속도 지시용 파라미터/디버그 ====
//     double slow_down_dist_      = 16.0; // 동적: 감속 시작
//     double stop_dist_           = 9.0;  // 동적: 정지
//     bool   has_front_obj_       = false;
//     double front_obj_dist_      = std::numeric_limits<double>::infinity();
//     std::string last_moving_cmd_= "none";

//     PathSelect()
//     {
//         ros::NodeHandle pnh("~");
//         pnh.param("objects_topic",       objects_topic_,       objects_topic_);
//         pnh.param("obs_dist_threshold",  obs_dist_threshold_,  obs_dist_threshold_);
//         pnh.param("slow_down_dist",      slow_down_dist_,      slow_down_dist_);
//         pnh.param("stop_dist",           stop_dist_,           stop_dist_);

//         ros::NodeHandle nh;

//         lane_path_sub_    = nh.subscribe("/lane_path",    10, &PathSelect::LanePathCB, this);
//         local_path1_sub_  = nh.subscribe("/local_path1",  10, &PathSelect::LocalPath1CB, this);
//         local_path2_sub_  = nh.subscribe("/local_path2",  10, &PathSelect::LocalPath2CB, this);
//         current_path_sub_ = nh.subscribe("/current_path", 10, &PathSelect::CurrentPathCB, this);
//         state_sub_        = nh.subscribe("/state",        10, &PathSelect::StateCB, this);
//         sign_state_sub_   = nh.subscribe("/sign_state",   10, &PathSelect::SignStateCB, this);

//         objects_sub_      = nh.subscribe(objects_topic_,   10, &PathSelect::ObjectsCB, this);

//         selected_path_pub_ = nh.advertise<nav_msgs::Path>("/selected_path", 1);
//         moving_obs_pub_    = nh.advertise<std_msgs::String>("/moving_obs", 1);
//     }

//     // --- Callbacks ---
//     void LanePathCB(const nav_msgs::Path::ConstPtr &msg)   { lane_path_  = *msg; }
//     void LocalPath1CB(const nav_msgs::Path::ConstPtr &msg) { local_path1_ = *msg; }
//     void LocalPath2CB(const nav_msgs::Path::ConstPtr &msg) { local_path2_ = *msg; }

//     void CurrentPathCB(const std_msgs::Int32::ConstPtr &msg)
//     {
//         current_path_ = msg->data;

//         // 차선 변경 완료 확인: commanded_path와 현재가 일치하면 hold 해제
//         if (hold_until_match_ && commanded_path_ == current_path_)
//         {
//             hold_until_match_ = false;
//             desired_path_     = current_path_; // 의도도 현재에 맞춰 동기화
//             commanded_path_   = -1;
//         }
//     }

//     void StateCB(const std_msgs::String::ConstPtr &msg)    { state = msg->data; }
//     void SignStateCB(const std_msgs::String::ConstPtr &msg){ sign_state = msg->data; }

//     void ObjectsCB(const katri_msgs::Objects::ConstPtr &msg)
//     {
//         objects_msg_ = *msg;
//     }

//     // --- Helper: 현재 차선 전방(+)의 "가장 가까운" 객체 거리 계산 (동적용) ---
//     bool getFrontObjectDistance(double &nearest_dist) const
//     {
//         nearest_dist = std::numeric_limits<double>::infinity();
//         bool found = false;

//         if (objects_msg_.objects.empty()) return false;

//         for (const auto& obj : objects_msg_.objects)
//         {
//             const bool same_lane = (obj.path_number == current_path_);
//             const bool is_front  = (obj.distance > 0.0) || (obj.obj_position == "front");
//             if (!same_lane || !is_front) continue;

//             if (obj.distance > 0.0 && obj.distance < nearest_dist)
//             {
//                 nearest_dist = obj.distance;
//                 found = true;
//             }
//         }
//         return found;
//     }

//     // --- Obstacle check for static_obs (차선별 판단/토글 근거) ---
//     void obstacle_check_static()
//     {
//         // 디버그 필드 초기화
//         obstacle = false;
//         obstacle_path_ = -1;
//         obstacle_distance_ = 1e9;

//         // 차선별 초기화
//         obstacle_lane1_ = false;
//         obstacle_lane2_ = false;
//         lane1_nearest_  = std::numeric_limits<double>::infinity();
//         lane2_nearest_  = std::numeric_limits<double>::infinity();

//         if (state != "static_obs") return;
//         if (objects_msg_.objects.empty()) return;

//         for (const auto& obj : objects_msg_.objects)
//         {
//             const bool is_front  = (obj.distance > 0.0) || (obj.obj_position == "front");
//             const bool within_th = (obj.distance > 0.0 && obj.distance <= obs_dist_threshold_);
//             if (!is_front) continue;

//             if (within_th && obj.distance < obstacle_distance_) {
//                 obstacle = true;
//                 obstacle_path_ = obj.path_number;
//                 obstacle_distance_ = obj.distance;
//             }

//             if (within_th)
//             {
//                 if (obj.path_number == 1) {
//                     obstacle_lane1_ = true;
//                     lane1_nearest_  = std::min(lane1_nearest_, obj.distance);
//                 } else if (obj.path_number == 2) {
//                     obstacle_lane2_ = true;
//                     lane2_nearest_  = std::min(lane2_nearest_, obj.distance);
//                 }
//             }
//         }
//     }

//     // --- [동적] 동적 장애물(비 static_obs) 속도지시 퍼블리시 ---
//     void publish_moving_obs_cmd()
//     {
//         // gps_fail이나 static_obs 중엔 퍼블리시하지 않음
//         if (state == "gps_fail" || state == "static_obs") return;

//         std_msgs::String cmd;
//         double d;
//         bool found = getFrontObjectDistance(d);

//         has_front_obj_  = found;
//         front_obj_dist_ = found ? d : std::numeric_limits<double>::infinity();

//         if (!found) {
//             cmd.data = "go";
//         } else {
//             if (d < stop_dist_)           cmd.data = "stop";
//             else if (d < slow_down_dist_) cmd.data = "slow_down";
//             else                          cmd.data = "go";
//         }
//         last_moving_cmd_ = cmd.data;  // 상태 저장
//         moving_obs_pub_.publish(cmd);
//     }

//     // --- Publish helpers ---
//     void publishPathByNumber(int num)
//     {
//         nav_msgs::Path out;
//         if (num == 1) {
//             out = local_path1_;
//             selected_path_number = 1;
//         } else if (num == 2) {
//             out = local_path2_;
//             selected_path_number = 2;
//         } else {
//             ROS_WARN_THROTTLE(1.0, "publishPathByNumber: invalid path %d", num);
//             return;
//         }
//         out.header.frame_id = frame_id_;
//         out.header.stamp = ros::Time::now();
//         selected_path_pub_.publish(out);
//     }

//     void publishLanePath()
//     {
//         nav_msgs::Path out = lane_path_;
//         out.header.frame_id = frame_id_;
//         out.header.stamp = ros::Time::now();
//         selected_path_pub_.publish(out);
//         selected_path_number = 0; // 디버그용(0 = lane_path)
//     }

//     // 스플라인으로 목표 차선으로 전환 명령 + 홀드 래치 ON
//     void command_change_to(int target_path)
//     {
//         nav_msgs::Path path;
//         if (target_path == 1) {
//             path.poses = cubicInterpolatePath(local_path1_, 10);
//             selected_path_number = 1;
//         } else if (target_path == 2) {
//             path.poses = cubicInterpolatePath(local_path2_, 10);
//             selected_path_number = 2;
//         } else {
//             ROS_WARN("command_change_to: invalid target_path: %d", target_path);
//             return;
//         }

//         path.header.frame_id = frame_id_;
//         path.header.stamp = ros::Time::now();
//         selected_path_pub_.publish(path);

//         // 래치: 실제 current_path가 바뀌었다고 올 때까지 다른 로직 무시
//         commanded_path_   = target_path;
//         hold_until_match_ = true;

//         // 의도도 목표에 맞춰 둠
//         desired_path_ = target_path;
//     }

//     // --- Simple cubic interpolation to make a smooth lane change segment ---
//     std::vector<geometry_msgs::PoseStamped> cubicInterpolatePath(const nav_msgs::Path &path, int num_points)
//     {
//         std::vector<geometry_msgs::PoseStamped> out;
//         const double interpolate_length = 2.0;

//         if (path.poses.empty()) {
//             ROS_WARN_THROTTLE(1.0, "cubicInterpolatePath: empty path");
//             return out;
//         }

//         int end_idx = -1;
//         for (size_t i = 0; i < path.poses.size(); ++i)
//         {
//             double dx = path.poses[i].pose.position.x - current_pose_.x;
//             if (dx >= interpolate_length) { end_idx = static_cast<int>(i); break; }
//         }
//         if (end_idx <= 0) {
//             ROS_WARN_THROTTLE(1.0, "cubicInterpolatePath: too short path or no forward point");
//             return out;
//         }

//         // start at (0,0) in base_link; end at end_idx point
//         double x0 = 0.0, y0 = 0.0;
//         double x1 = path.poses[end_idx].pose.position.x;
//         double y1 = path.poses[end_idx].pose.position.y;

//         double dx = x1 - x0; if (dx <= 0.0) dx = std::fabs(dx) + 1e-3;
//         double end_slope = 0.0;
//         if (end_idx + 1 < (int)path.poses.size())
//         {
//             double nx = path.poses[end_idx + 1].pose.position.x;
//             double ny = path.poses[end_idx + 1].pose.position.y; // (FIX) poses
//             double ndx = (nx - x1);
//             end_slope = (std::fabs(ndx) > 1e-6) ? (ny - y1) / ndx : 0.0;
//         }

//         const double c0 = y0;
//         const double c1 = 0.0; // start slope
//         const double c2 = (3 * (y1 - y0) / (dx * dx)) - ((end_slope + 2 * c1) / dx);
//         const double c3 = (2 * (y0 - y1) / (dx * dx * dx)) + ((end_slope + c1) / (dx * dx));

//         out.reserve(num_points + (path.poses.size() - end_idx));
//         for (int i = 0; i < num_points; ++i)
//         {
//             double t = (num_points == 1) ? 0.0 : (double)i / (num_points - 1);
//             double x = x0 + t * dx;
//             double y = c0 + c1 * (x - x0) + c2 * std::pow((x - x0), 2) + c3 * std::pow((x - x0), 3);

//             geometry_msgs::PoseStamped p;
//             p.header.frame_id = frame_id_;
//             p.header.stamp = ros::Time::now();
//             p.pose.position.x = x;
//             p.pose.position.y = y;
//             p.pose.orientation.w = 1.0;
//             out.push_back(p);
//         }
//         for (size_t i = end_idx; i < path.poses.size(); ++i)
//         {
//             auto p = path.poses[i];
//             p.header.frame_id = frame_id_;
//             out.push_back(p);
//         }
//         return out;
//     }
// };

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "ai_local_path_selector");
//     PathSelect ps;
//     ros::Rate rate(20);

//     while (ros::ok())
//     {
//         ros::spinOnce();

//         const bool cur_static     = (ps.state == "static_obs");
//         const bool entered_static = (cur_static && !ps.prev_static_);
//         const bool left_static    = (!cur_static && ps.prev_static_);
//         ps.prev_static_ = cur_static;

//         // === gps_fail 우선 처리: 무조건 lane_path 추종, 홀드 해제, 다른 로직 무시 ===
//         if (ps.state == "gps_fail")
//         {
//             ps.hold_until_match_ = false;
//             ps.commanded_path_   = -1;
//             ps.publishLanePath();
//             rate.sleep();
//             continue;
//         }

//         // --- 변경 진행 중이면 다른 로직 전부 무시: 목표 차선 경로만 계속 퍼블리시 ---
//         if (ps.hold_until_match_)
//         {
//             ps.publishPathByNumber(ps.commanded_path_);
//             rate.sleep();
//             continue;
//         }

//         // --- 상태 전이 처리 ---
//         if (entered_static)
//         {
//             // static_obs 들어갈 때 기본 의도는 현재 차선을 유지
//             ps.desired_path_ = ps.current_path_;
//         }
//         if (left_static)
//         {
//             // static_obs 끝나면 무조건 1번으로 복귀
//             if (ps.current_path_ != 1) {
//                 ps.desired_path_ = 1;
//                 ps.command_change_to(1);
//                 ps.publishPathByNumber(1);
//                 rate.sleep();
//                 continue; // 변경 완료될 때까지 hold에서 처리
//             } else {
//                 ps.desired_path_ = 1;
//                 ps.publishPathByNumber(1);
//                 // static_obs 해제 직후 동적 지시 퍼블리시 가능
//                 ps.publish_moving_obs_cmd();
//                 rate.sleep();
//                 continue;
//             }
//         }

//         // --- static_obs 중 로직 ---
//         ps.obstacle_check_static();

//         if (cur_static)
//         {
//             // 1번 주행 중 1번에 장애물 → 무조건 2번으로 변경(그리고 hold)
//             if (ps.current_path_ == 1 && ps.obstacle_lane1_)
//             {
//                 ps.desired_path_ = 2;
//                 ps.command_change_to(2);
//                 ps.publishPathByNumber(2);
//                 rate.sleep();
//                 continue; // 변경 완료될 때까지 hold
//             }

//             // 2번 주행 중 2번에 장애물 → 무조건 1번으로 변경(그리고 hold)
//             if (ps.current_path_ == 2 && ps.obstacle_lane2_)
//             {
//                 ps.desired_path_ = 1;
//                 ps.command_change_to(1);
//                 ps.publishPathByNumber(1);
//                 rate.sleep();
//                 continue; // 변경 완료될 때까지 hold
//             }

//             // 장애물 없으면 현재 차선 유지
//             ps.desired_path_ = ps.current_path_;
//             ps.publishPathByNumber(ps.current_path_);
//             // static_obs 중에는 /moving_obs 퍼블리시 안 함
//         }
//         else
//         {
//             // static_obs가 아니면 무조건 1번 추종
//             if (ps.current_path_ != 1)
//             {
//                 ps.desired_path_ = 1;
//                 ps.command_change_to(1);
//                 ps.publishPathByNumber(1);
//             }
//             else
//             {
//                 ps.desired_path_ = 1;
//                 ps.publishPathByNumber(1);
//             }

//             // [동적] 전방 거리 기반 속도 지시 퍼블리시 (go/slow_down/stop)
//             ps.publish_moving_obs_cmd();
//         }

//         // Debug
//         std::cout << "State=" << ps.state
//                   << " | current=" << ps.current_path_
//                   << std::endl;
//         std::cout << "hold=" << std::boolalpha << ps.hold_until_match_
//                   << " | selected=" << ps.selected_path_number
//                   << std::endl;

//         std::cout << "Lane1 obs=" << ps.obstacle_lane1_
//                   << " (nearest=" << (std::isinf(ps.lane1_nearest_) ? -1.0 : ps.lane1_nearest_) << " m)"
//                   << " | Lane2 obs=" << ps.obstacle_lane2_
//                   << " (nearest=" << (std::isinf(ps.lane2_nearest_) ? -1.0 : ps.lane2_nearest_) << " m)"
//                   << std::endl;

//         std::cout << "moving_obj_dist=";
//         if (ps.has_front_obj_) {
//             std::cout << std::fixed << std::setprecision(2) << ps.front_obj_dist_ << " m";
//         } else {
//             std::cout << "INF";
//         }
//         std::cout << " | obs state=" << ps.last_moving_cmd_ << std::endl;

//         std::cout << "Obstacle(static): " << std::boolalpha << ps.obstacle
//                   << " (path=" << ps.obstacle_path_
//                   << ", dist=" << ps.obstacle_distance_ << ")" << std::endl;

//         std::cout << "=========================================== " << std::endl;

//         rate.sleep();
//     }
//     return 0;
// }

// file: src/local_path_selector.cpp

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

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
    ros::Publisher lane_change_state_pub_;
    
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
    double obs_dist_threshold_  = 20.0; // static_obs 회피 판단 거리(차선 변경용)

    // ==== 동적 장애물 속도 지시용 파라미터/디버그 ====
    double slow_down_dist_      = 20.0; // 동적: 감속 시작
    double stop_dist_           = 3.0;  // 동적: 정지
    bool   has_front_obj_       = false;
    double front_obj_dist_      = std::numeric_limits<double>::infinity();
    std::string last_moving_cmd_= "none";

    PathSelect()
    {
        ros::NodeHandle pnh("~");
        pnh.param("objects_topic",       objects_topic_,       objects_topic_);
        pnh.param("obs_dist_threshold",  obs_dist_threshold_,  obs_dist_threshold_);
        pnh.param("slow_down_dist",      slow_down_dist_,      slow_down_dist_);
        pnh.param("stop_dist",           stop_dist_,           stop_dist_);

        ros::NodeHandle nh;

        lane_path_sub_    = nh.subscribe("/lane_path",    10, &PathSelect::LanePathCB, this);
        local_path1_sub_  = nh.subscribe("/local_path1",  10, &PathSelect::LocalPath1CB, this);
        local_path2_sub_  = nh.subscribe("/local_path2",  10, &PathSelect::LocalPath2CB, this);
        current_path_sub_ = nh.subscribe("/current_path", 10, &PathSelect::CurrentPathCB, this);
        state_sub_        = nh.subscribe("/state",        10, &PathSelect::StateCB, this);
        sign_state_sub_   = nh.subscribe("/sign_state",   10, &PathSelect::SignStateCB, this);

        objects_sub_      = nh.subscribe(objects_topic_,   10, &PathSelect::ObjectsCB, this);

        selected_path_pub_ = nh.advertise<nav_msgs::Path>("/selected_path", 1);
        moving_obs_pub_    = nh.advertise<std_msgs::String>("/moving_obs", 1);
        lane_change_state_pub_  = nh.advertise<std_msgs::Int32>("/lane_change_state", 1);
    }

    // --- Callbacks ---
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
            path.poses = cubicInterpolatePath(local_path1_, 10);
            selected_path_number = 1;
        } else if (target_path == 2) {
            path.poses = cubicInterpolatePath(local_path2_, 10);
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
        const double interpolate_length = 2.0;

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
    ros::init(argc, argv, "ai_local_path_selector");
    PathSelect ps;
    ros::Rate rate(20);
    std_msgs::Int32 lane_change_state_msg;

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

        // --- 변경 진행 중이면 다른 로직 전부 무시: 목표 차선 경로만 계속 퍼블리시 ---
        if (ps.hold_until_match_)
        {
            lane_change_state_msg.data=20;
            ps.lane_change_state_pub_.publish(lane_change_state_msg);
            ps.publishPathByNumber(ps.commanded_path_);
            rate.sleep();
            continue;
        }
        else if (lane_change_state_msg.data>0){
            lane_change_state_msg.data-=1;
        }
        ps.lane_change_state_pub_.publish(lane_change_state_msg);
        // --- 상태 전이 처리 ---
        if (entered_static)
        {
            // static_obs 들어갈 때 기본 의도는 현재 차선을 유지
            ps.desired_path_ = ps.current_path_;
        }
        if (left_static)
        {
            // static_obs 끝나면 무조건 1번으로 복귀
            if (ps.current_path_ != 1) {
                ps.desired_path_ = 1;
                ps.command_change_to(1);
                ps.publishPathByNumber(1);
                rate.sleep();
                continue; // 변경 완료될 때까지 hold에서 처리
            } else {
                ps.desired_path_ = 1;
                ps.publishPathByNumber(1);
                // static_obs 해제 직후 동적 지시 퍼블리시 가능
                ps.publish_moving_obs_cmd();
                rate.sleep();
                continue;
            }
        }

        // --- static_obs 중 로직 ---
        ps.obstacle_check_static();

        if (cur_static)
        {
            // 1번 주행 중 1번에 장애물 → 무조건 2번으로 변경(그리고 hold)
            if (ps.current_path_ == 1 && ps.obstacle_lane1_)
            {
                ps.desired_path_ = 2;
                ps.command_change_to(2);
                ps.publishPathByNumber(2);
                rate.sleep();
                continue; // 변경 완료될 때까지 hold
            }

            // 2번 주행 중 2번에 장애물 → 무조건 1번으로 변경(그리고 hold)
            if (ps.current_path_ == 2 && ps.obstacle_lane2_)
            {
                ps.desired_path_ = 1;
                ps.command_change_to(1);
                ps.publishPathByNumber(1);
                rate.sleep();
                continue; // 변경 완료될 때까지 hold
            }

            // 장애물 없으면 현재 차선 유지
            ps.desired_path_ = ps.current_path_;
            ps.publishPathByNumber(ps.current_path_);
            // static_obs 중에는 /moving_obs 퍼블리시 안 함
        }
        else
        {
            // static_obs가 아니면 무조건 1번 추종
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

        // Debug
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

        rate.sleep();
    }
    return 0;
}
