#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <iomanip>

class HybridControl {
public:
    HybridControl();
    void run();
    void speed_control(); 
    void steer_control();

    double getCrossTrackError(const geometry_msgs::Point& front_axle_position, const geometry_msgs::Point& closest_point);
    double getPathYawFromPoint(const nav_msgs::Path& path, const geometry_msgs::Point& closest_point);  

    geometry_msgs::Point calculateFrontAxlePosition(double front_axle_offset);
    geometry_msgs::Point findClosestPathPoint(const nav_msgs::Path& path, const geometry_msgs::Point& front_axle_position);

private:
    ros::NodeHandle nh;
    ros::Publisher ctrl_cmd_pub;
    ros::Subscriber gps_sub, ego_vehicle_sub, local_path_sub, speed_control_sub, state_sub;

    nav_msgs::Path localpath;
    geometry_msgs::Point current_position;
    
    geometry_msgs::Twist ctrl_cmd_msg;

    double LD;
    double target_vel, current_vel;
    double vehicle_yaw;
    double vehicle_length;
    double front_axle_offset;
    std::string map_state = "go";
    std::string car_state = "car_go";

    bool is_look_forward_point;
    bool gps_state, path_state, vehicle_info;
    int frequency;
    double dt_;
    double localpath_yaw;
    double k = 1.6;
    double k_p;
    double k_s;

    void GPSCallBack(const nav_msgs::Odometry::ConstPtr& msg);
    void EgoVehicleCallBack(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg);
    void LocalPathCallBack(const nav_msgs::Path::ConstPtr& msg);
    void SpeedControlCallBack(const std_msgs::String::ConstPtr& msg);
    void StateCallBack(const std_msgs::String::ConstPtr& msg);
};

HybridControl::HybridControl() 
    : nh("~"),
       
        LD(5), // init
        target_vel(650), // init
        
        current_vel(0),
        vehicle_yaw(0.0),
        vehicle_length(1.04),
        is_look_forward_point(false),
        gps_state(false),
        path_state(false),
        vehicle_info(false),
        frequency(50),
        dt_(1.0 / frequency) { 
        
        ros::NodeHandle nh;

        gps_sub = nh.subscribe("/gps_utm_odom", 1, &HybridControl::GPSCallBack, this);
        ego_vehicle_sub = nh.subscribe("/gps_data/fix_velocity", 1, &HybridControl::EgoVehicleCallBack, this);
        local_path_sub = nh.subscribe("/selected_path", 1, &HybridControl::LocalPathCallBack, this);
        speed_control_sub = nh.subscribe("/state", 1, &HybridControl::SpeedControlCallBack, this);
        state_sub = nh.subscribe("/speed_control", 1, &HybridControl::StateCallBack, this);
        
        ctrl_cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        ctrl_cmd_msg = geometry_msgs::Twist();
    }

void HybridControl::GPSCallBack(const nav_msgs::Odometry::ConstPtr& msg) {
    gps_state = true;

    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    vehicle_yaw = yaw;
    current_position.x = msg->pose.pose.position.x;
    current_position.y = msg->pose.pose.position.y;
    current_position.z = msg->pose.pose.position.z;
}

void HybridControl::EgoVehicleCallBack(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg) {
    vehicle_info = true;
    current_vel = sqrt(pow(msg->twist.twist.linear.x, 2) + pow(msg->twist.twist.linear.y, 2)) * 3.6;
}

void HybridControl::LocalPathCallBack(const nav_msgs::Path::ConstPtr& msg) {
    path_state = true;
    localpath = *msg;
}

void HybridControl::SpeedControlCallBack(const std_msgs::String::ConstPtr& msg) {
    map_state = msg->data;
}

void HybridControl::StateCallBack(const std_msgs::String::ConstPtr& msg) {
    car_state = msg->data;
}

///////////////////// <<<<튜닝>>>> ///////////////////////////////////////
void HybridControl::speed_control() {
    constexpr int MAX = 900; 
    constexpr int MIDDLE = 500;
    constexpr int SLOW = 500;
    constexpr int SLOW_2 = 650;
    constexpr int SLOW_3 = 435;
    constexpr int STOP = 0;

    if (map_state == "go") { // 차선 변경 X
        if (car_state == "car_go") target_vel = MAX;
        else if (car_state == "car_slow") target_vel = SLOW; // 기존 slow보다 속도 더 낮게
        else if (car_state == "stop") target_vel = STOP;
        else if (car_state == "car_lane_change") target_vel = SLOW; //
    } 
    else if (map_state == "slow") { 
        if (car_state == "car_go") target_vel = SLOW;
        else if (car_state == "car_slow") target_vel = SLOW_2;
        else if (car_state == "stop") target_vel = STOP;
        else if (car_state == "car_lane_change") target_vel = SLOW_2;
    }
    else if (map_state == "slow2") { 
        if (car_state == "car_go") target_vel = SLOW_2;
        else if (car_state == "car_slow") target_vel = SLOW_3;
        else if (car_state == "stop") target_vel = STOP;
        else if (car_state == "car_lane_change") target_vel = SLOW_3;
    }
    else if (map_state == "slow3") { 
        if (car_state == "car_go") target_vel = SLOW_3;
        else if (car_state == "car_slow") target_vel = SLOW_3;
        else if (car_state == "stop") target_vel = STOP;
        else if (car_state == "car_lane_change") target_vel = 400;
    }
    else if (map_state == "slow4") { 
        if (car_state == "car_go") target_vel = SLOW_2;
        else if (car_state == "car_slow") target_vel = SLOW_3;
        else if (car_state == "stop") target_vel = STOP;
        else if (car_state == "car_lane_change") target_vel = SLOW_3;
    }
    else if (map_state == "overtake") {  // 차선 변경 O
        if (car_state == "car_go") target_vel = MAX;
        else if (car_state == "car_slow") target_vel = SLOW;
        else if (car_state == "stop") target_vel = STOP;
        else if (car_state == "car_lane_change") target_vel = MIDDLE;
    }

    // LD 값 설정 (속도에 따라 다르게 적용)
    if (target_vel == MAX) LD = 7;
    else if (target_vel == MIDDLE) LD = 6;
    else if (target_vel == SLOW) LD = 5;
    else if (target_vel == SLOW_2) LD = 5;
    else if (target_vel == SLOW_3) LD = 4;
    else LD = 6;  // STOP or 기타 속도
}

void HybridControl::steer_control() {
    constexpr double MAX = 0.9;
    constexpr double MIDDLE = 0.8;
    constexpr double SLOW = 0.55;
    constexpr double SLOW_2 = 0.7;
    constexpr double SLOW_3 = 0.7;
    constexpr double STOP = 1.0;

    if (map_state == "go") { // 차선 변경 X
        if (car_state == "car_go") k_p = MAX;
        else if (car_state == "car_slow") k_p = SLOW;
        else if (car_state == "car_stop") k_p = STOP;
        else if (car_state == "car_lane_change") k_p = MIDDLE; //
    } 
    else if (map_state == "slow") {  
        if (car_state == "car_go") k_p = SLOW;
        else if (car_state == "car_slow") k_p = SLOW_2; // 기존 slow보다 속도 더 낮게
        else if (car_state == "car_stop") k_p = STOP;
        else if (car_state == "car_lane_change") k_p = SLOW; //
    } 
    else if (map_state == "slow4") {  
        if (car_state == "car_go") k_p = SLOW;
        else if (car_state == "car_slow") k_p = SLOW_2; // 기존 slow보다 속도 더 낮게
        else if (car_state == "car_stop") k_p = STOP;
        else if (car_state == "car_lane_change") k_p = SLOW; //
    } 
    else if (map_state == "slow2") {  
        if (car_state == "car_go") k_p = SLOW_2;
        else if (car_state == "car_slow") k_p = SLOW_2; // 기존 slow보다 속도 더 낮게
        else if (car_state == "car_stop") k_p = STOP;
        else if (car_state == "car_lane_change") k_p = SLOW_2; //
    } 
    else if (map_state == "slow3") {  
        if (car_state == "car_go") k_p = SLOW_3;
        else if (car_state == "car_slow") k_p = SLOW_3; // 기존 slow보다 속도 더 낮게
        else if (car_state == "car_stop") k_p = STOP;
        else if (car_state == "car_lane_change") k_p = SLOW_3; //
    } 
    else if (map_state == "overtake") {  // 차선 변경 O
        if (car_state == "car_go") k_p = MAX;
        else if (car_state == "car_slow") k_p = SLOW;
        else if (car_state == "car_stop") k_p = STOP;
        else if (car_state == "car_lane_change") k_p= MIDDLE;
    }

    k_s = 1.0 - k_p;
}
///////////////////////////////////////////////////////////////////////////

double HybridControl::getPathYawFromPoint(const nav_msgs::Path& path, const geometry_msgs::Point& closest_point) 
{
    auto it = std::find_if(path.poses.begin(), path.poses.end(), [&closest_point](const auto& pose_stamped) {
        return pose_stamped.pose.position.x == closest_point.x && pose_stamped.pose.position.y == closest_point.y;
    });

    if (it != path.poses.end() && std::next(it) != path.poses.end()) {
        const auto& next_point = std::next(it)->pose.position;
        double dx = next_point.x - closest_point.x;
        double dy = next_point.y - closest_point.y;
        return atan2(dy, dx);
    } else {
        return 0.0;
    }
}

double HybridControl::getCrossTrackError(const geometry_msgs::Point& front_axle_position, const geometry_msgs::Point& closest_point)
{
    double dx = closest_point.x - front_axle_position.x;
    double dy = closest_point.y - front_axle_position.y;

    double angle = atan2(dy, dx);
    double error_distance = -sqrt(dx * dx + dy * dy) * sin(angle);

    return error_distance;
}

geometry_msgs::Point HybridControl::findClosestPathPoint(const nav_msgs::Path& path, const geometry_msgs::Point& front_axle_position)
{
    double min_distance = std::numeric_limits<double>::max();
    geometry_msgs::Point closest_point;

    for (const auto& pose_stamped : path.poses)
    {
        double distance = std::hypot(front_axle_position.x - pose_stamped.pose.position.x, 
                                     front_axle_position.y - pose_stamped.pose.position.y);
        if (distance < min_distance)
        {
            min_distance = distance;
            closest_point = pose_stamped.pose.position;
        }
    }

    return closest_point;
}

geometry_msgs::Point HybridControl::calculateFrontAxlePosition(double front_axle_offset) 
{
    geometry_msgs::Point front_axle_position;
    
    front_axle_offset = 0.1;

    front_axle_position.x = front_axle_offset;

    return front_axle_position;
}

void HybridControl::run() {
    ros::Rate rate(frequency);

    while (ros::ok()) {
        if (gps_state && path_state && vehicle_info) {
            is_look_forward_point = false;

            for (const auto& pose : localpath.poses) {
                double x = pose.pose.position.x;
                double y = pose.pose.position.y;
                double dis = sqrt(x * x + y * y);


                if (LD <= dis && dis < LD + 10) {
                    is_look_forward_point = true;

                    double alpha = atan2(y, x);  // 목표점과 차량의 상대 각도
                    double steer_p = atan2((2 * vehicle_length * sin(alpha)), LD);

                    geometry_msgs::Point front_axle_position = calculateFrontAxlePosition(front_axle_offset);
                    geometry_msgs::Point closest_point = findClosestPathPoint(this->localpath, front_axle_position);

                    double pathYaw = getPathYawFromPoint(this->localpath, closest_point);
                    double crossTrackError = getCrossTrackError(closest_point, front_axle_position);
                    double cte_term = atan2(k * crossTrackError, this->current_vel);
                    double steer_s = pathYaw + cte_term;

                    while (steer_s > M_PI){
                        steer_s -= 2 * M_PI;
                    }
                    while (steer_s < -M_PI){
                        steer_s +=  2 * M_PI;
                    }

                    speed_control();
                    steer_control();

                    double steer = steer_p * k_p + steer_s * k_s;
                    

                    if (target_vel < 0) {
                        target_vel = 1;
                    }
            
                    ctrl_cmd_msg.linear.x = std::max(0.0, target_vel);
                    ctrl_cmd_msg.angular.z = - steer;
          
                    std::cout << std::fixed << std::setprecision(2);
                    std::cout << "k_p : " << k_p << " | k_s : " << k_s << std::endl;
                    std::cout << "pathYaw : " << pathYaw << " | CTE : " << crossTrackError << " | steer_s : " << steer_s * 180 / M_PI << " | steer_p : " << steer_p * 180 / M_PI << " | steer : " << steer * 180 / M_PI<< std::endl;
                    std::cout << "LD : " << LD << " | Target Vel : " << target_vel << " | Current Vel : " << current_vel  << std::endl;
                    break;
                }
            }
        }

        if (!is_look_forward_point) {
            ctrl_cmd_msg.linear.x = 0;
            ctrl_cmd_msg.angular.z = 0;
    
        }
        ctrl_cmd_pub.publish(ctrl_cmd_msg);
        std::cout << "Steer [deg] :" << ctrl_cmd_msg.angular.z * 180 / M_PI << " | Target Vel :" << target_vel << " | Current Vel :" << static_cast<int>(current_vel)  << " | LD [m] :" << LD << std::endl;
        std::cout << "Map_state :" << map_state << " | Car_state :" << car_state << std::endl;
        std::cout << "=================================================================" << std::endl;

        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "HybridControl_Robo");

    HybridControl hc;
    
    ros::Rate rate(50);

    while (ros::ok()) {
        hc.run();
        rate.sleep();
    }

    return 0;
}
