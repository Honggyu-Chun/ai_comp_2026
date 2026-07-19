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

class PurePursuit {
public:
    PurePursuit();
    void run();
    void speed_control(); 

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
    std::string map_state = "go";
    std::string car_state = "car_go";

    bool is_look_forward_point;
    bool gps_state, path_state, vehicle_info;
    int frequency;
    double dt_;
    double localpath_yaw;

    void GPSCallBack(const nav_msgs::Odometry::ConstPtr& msg);
    void EgoVehicleCallBack(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg);
    void LocalPathCallBack(const nav_msgs::Path::ConstPtr& msg);
    void SpeedControlCallBack(const std_msgs::String::ConstPtr& msg);
    void StateCallBack(const std_msgs::String::ConstPtr& msg);
};

PurePursuit::PurePursuit() 
    : nh("~"),
        LD(2), // init
        target_vel(300), // init
    
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

        gps_sub = nh.subscribe("/gps_utm_odom", 1, &PurePursuit::GPSCallBack, this);
        ego_vehicle_sub = nh.subscribe("/gps_data/fix_velocity", 1, &PurePursuit::EgoVehicleCallBack, this);
        local_path_sub = nh.subscribe("/selected_path", 1, &PurePursuit::LocalPathCallBack, this);
        speed_control_sub = nh.subscribe("/state", 1, &PurePursuit::SpeedControlCallBack, this);
        state_sub = nh.subscribe("/speed_control", 1, &PurePursuit::StateCallBack, this);
        
        ctrl_cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        ctrl_cmd_msg = geometry_msgs::Twist();
    }

void PurePursuit::GPSCallBack(const nav_msgs::Odometry::ConstPtr& msg) {
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

void PurePursuit::EgoVehicleCallBack(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg) {
    vehicle_info = true;
    double linear_x = msg->twist.twist.linear.x;
    // double linear_y = msg->twist.twist.linear.y;
    // current_vel = sqrt(linear_x * linear_x + linear_y * linear_y);
    current_vel = linear_x;
}

void PurePursuit::LocalPathCallBack(const nav_msgs::Path::ConstPtr& msg) {
    path_state = true;
    localpath = *msg;
}

void PurePursuit::SpeedControlCallBack(const std_msgs::String::ConstPtr& msg) {
    map_state = msg->data;
}

void PurePursuit::StateCallBack(const std_msgs::String::ConstPtr& msg) {
    car_state = msg->data;
}

///////////////////////// <<<<튜닝>>>> ////////////////////////////////////
void PurePursuit::speed_control() {
    constexpr int MAX = 800; 
    constexpr int MIDDLE = 800;
    constexpr int SLOW = 650;
    constexpr int SLOW_2 = 150;
    constexpr int STOP = 0;

    if (map_state == "go") { // 차선 변경 X
        if (car_state == "car_go") target_vel = MAX;
        else if (car_state == "car_slow") target_vel = SLOW;
        else if (car_state == "stop") target_vel = STOP;
        else if (car_state == "car_lane_change") target_vel = SLOW; //
    } 
    else if (map_state == "slow") {  
        if (car_state == "car_go") target_vel = SLOW;
        else if (car_state == "car_slow") target_vel = SLOW_2; // 기존 slow보다 속도 더 낮게
        else if (car_state == "stop") target_vel = STOP;
        else if (car_state == "car_lane_change") target_vel = SLOW; //
    } 
    else if (map_state == "overtake") {  // 차선 변경 O
        if (car_state == "car_go") target_vel = MIDDLE;
        else if (car_state == "car_slow") target_vel = SLOW;
        else if (car_state == "stop") target_vel = STOP;
        else if (car_state == "car_lane_change") target_vel = SLOW;
    }

    // LD 값 설정 (속도에 따라 다르게 적용)
    LD = ((0.2 * current_vel) + 2);
    // if (target_vel == MAX) LD = 6;
    // else if (target_vel == MIDDLE) LD = 6;
    // else if (target_vel == SLOW) LD = 6;
    // else if (target_vel == SLOW_2) LD = 3;
    // else LD = 4;  // STOP or 기타 속도
}
///////////////////////////////////////////////////////////////////////////

void PurePursuit::run() {
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

                    double alpha = atan2(y, x);  
                    double steer = atan2((2 * vehicle_length * sin(alpha)), LD);  

                    speed_control();

                    if (target_vel < 0) {
                        target_vel = 1;
                    }
            
                    ctrl_cmd_msg.linear.x = std::max(0.0, target_vel);
                    ctrl_cmd_msg.angular.z = -steer;
          
                    ctrl_cmd_pub.publish(ctrl_cmd_msg);
                    std::cout << std::fixed << std::setprecision(2);
                    break;
                }
            }
        }

        if (!is_look_forward_point) {
            ctrl_cmd_msg.linear.x = 0.0;
            ctrl_cmd_msg.angular.z = 0.0;
            ctrl_cmd_pub.publish(ctrl_cmd_msg);
    
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
    ros::init(argc, argv, "PurePursuit_Robo");

    PurePursuit pp;

    ros::Rate rate(50);

    while (ros::ok()) {
        pp.run();
        rate.sleep();
    }

    return 0;
}
