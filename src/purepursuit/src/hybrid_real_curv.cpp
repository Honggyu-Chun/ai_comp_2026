#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <iomanip> 

class HybridControl {
public:
    HybridControl();
    void run();
    void speed_control(const double curvature); 
    void steer_control(const double curvature);

    double getCrossTrackError(const geometry_msgs::Point& front_axle_position, const geometry_msgs::Point& closest_point);
    double getPathYawFromPoint(const nav_msgs::Path& path, const geometry_msgs::Point& closest_point);
    double calculateCurvature(const nav_msgs::Path& path, const geometry_msgs::Point& closest_point);
    double applyKalmanFilter(double measured_curvature);

    geometry_msgs::Point calculateFrontAxlePosition(double front_axle_offset);
    geometry_msgs::Point findClosestPathPoint(const nav_msgs::Path& path, const geometry_msgs::Point& front_axle_position);

private:
    ros::NodeHandle nh;
    ros::Publisher ctrl_cmd_pub, rmse_pub, curvature_pub, hde_pub, cte_pub, vel_pub, penalty_pub;
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
    double k = 1.5;
    double k_p;
    double k_s;
    double vel_penalty = 0.0;
    double steer_ratio = 0.0;

    double cte_sum;       // CTE의 제곱 합
    int cte_count;        // CTE 샘플 수

    // 칼만 필터 관련 변수
    double curvature_estimate;    // 추정된 곡률
    double estimate_uncertainty;   // 추정 불확실성
    double measurement_noise;      // 측정 노이즈
    double process_noise;         // 프로세스 노이즈

    void GPSCallBack(const nav_msgs::Odometry::ConstPtr& msg);
    void EgoVehicleCallBack(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg);
    void LocalPathCallBack(const nav_msgs::Path::ConstPtr& msg);
    void SpeedControlCallBack(const std_msgs::String::ConstPtr& msg);
    void StateCallBack(const std_msgs::String::ConstPtr& msg);
};

HybridControl::HybridControl() 
    : nh("~"),
       
        LD(10), // init
        target_vel(100), // init
        current_vel(0),
        vehicle_yaw(0.0),
        vehicle_length(1.04),
        is_look_forward_point(false),
        gps_state(false),
        path_state(false),
        vehicle_info(false),
        frequency(50),
        dt_(1.0 / frequency),
        curvature_estimate(0.0),
        estimate_uncertainty(1.0),
        measurement_noise(0.1),      // 조정 가능
        process_noise(0.01),         // 조정 가능
        cte_sum(0.0),
        cte_count(0)
    { 
        
        ros::NodeHandle nh;

        gps_sub = nh.subscribe("/gps_utm_odom", 1, &HybridControl::GPSCallBack, this);
        ego_vehicle_sub = nh.subscribe("/gps_data/fix_velocity", 1, &HybridControl::EgoVehicleCallBack, this);
        local_path_sub = nh.subscribe("/selected_path", 1, &HybridControl::LocalPathCallBack, this);
        speed_control_sub = nh.subscribe("/state", 1, &HybridControl::SpeedControlCallBack, this);
        state_sub = nh.subscribe("/speed_control", 1, &HybridControl::StateCallBack, this);
        
        // rmse_pub = nh.advertise<std_msgs::Float64>("/rmse", 10);
        // curvature_pub = nh.advertise<std_msgs::Float64>("/curvature", 10); // 곡률 publisher 초기화
        // hde_pub = nh.advertise<std_msgs::Float64>("/heading_error", 10); // heading error publisher 초기화
        // cte_pub = nh.advertise<std_msgs::Float64>("/cross_track_error", 10); // CTE publisher 초기화
        // vel_pub = nh.advertise<std_msgs::Float64>("/target_vel", 10); // target velocity publisher 초기화
        // penalty_pub = nh.advertise<std_msgs::Float64>("/penalty", 10); // 속도 감소 publisher 초기화

        // ctrl_cmd_msg = morai_msgs::CtrlCmd();
        // ctrl_cmd_msg.longlCmdType = 2;
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
void HybridControl::speed_control(const double curvature) {
    const double MAX = 600.0;
    const double CURVATURE_THRESHOLD = 0.02;  // 곡률 임계값
    const double CURVATURE_FACTOR = 1000.0;    // 곡률에 따른 감속 계수
    // 곡률에 따른 속도 계산
    if (curvature > CURVATURE_THRESHOLD) {
        vel_penalty = std::min(MAX * 0.4, curvature * CURVATURE_FACTOR);
        //vel_penalty = curvature * CURVATURE_FACTOR; // 곡률에 비례하여 속도 감소
        // ROS_INFO("Curve detected! Curvature: %.3f, Velocity penalty: %.1f", curvature, vel_penalty);
        if (curvature > 0.1) {
            target_vel = std::max(MAX * 0.4, MAX - vel_penalty);
        }
        else if (curvature > 0.05) {
            target_vel = std::max(MAX * 0.6, MAX - vel_penalty);
        }
        else {
            target_vel = std::max(MAX * 0.8, MAX - vel_penalty);
        }
    } else {
        vel_penalty = 0.0;
        target_vel = MAX;
    }

    // LD 값 설정
    const double BASE_LFW = 6.0;    // 기본 Look-ahead distance
    const double MIN_LFW = 5.0;     // 최소 Look-ahead distance
    const double MAX_LFW = 7.0;     // 최대 Look-ahead distance

    // 곡률에 따른 Lfw 계산
    if (curvature > CURVATURE_THRESHOLD) {
        // 곡률이 큰 경우 Lfw 감소
        double raw_lfw = BASE_LFW - (curvature * 70.0);
        this->LD = std::max(MIN_LFW, raw_lfw);
    } else {
        // 직선 구간에서는 속도에 비례
        double raw_lfw = BASE_LFW + (this->current_vel * 0.3);
        this->LD = std::min(MAX_LFW, raw_lfw);
    }
}

void HybridControl::steer_control(const double curvature) {
    // k_p 값 설정
    constexpr double MAX = 0.7;
    const double CURVATURE_THRESHOLD = 0.02;  // 곡률 임계값
    const double CURVATURE_FACTOR = 2.5;    // 곡률에 따른 감속 계수

    // if (curvature > CURVATURE_THRESHOLD) {
    //     steer_ratio = std::min(0.15, curvature * CURVATURE_FACTOR);
    //     if (curvature > 0.1) {
    //         k_p = std::max(MAX * 0.4, MAX - steer_ratio);
    //     }
    //     else if (curvature > 0.05) {
    //         k_p = std::max(MAX * 0.6, MAX - steer_ratio);
    //     }
    //     else {
    //         k_p = std::max(MAX * 0.8, MAX - steer_ratio);
    //     }
    // } else {
    //     steer_ratio = 0.0;
        k_p = MAX;
    // }

    k_s = 1.0 - k_p;
}
///////////////////////////////////////////////////////////////////////////

double HybridControl::calculateCurvature(const nav_msgs::Path& path, const geometry_msgs::Point& closest_point)
{
    auto it = std::find_if(path.poses.begin(), path.poses.end(), 
        [&closest_point](const auto& pose_stamped) {
            return pose_stamped.pose.position.x == closest_point.x && 
                   pose_stamped.pose.position.y == closest_point.y;
    });

    if (it == path.poses.end()) return 0.0;

    // 고정된 곡률 계산 구간 설정
    const double CURVATURE_CALC_DIST = 5.0;  // 3m 구간에서 곡률 계산
    
    // 현재 위치에서 일정 거리만큼의 점들 수집
    std::vector<geometry_msgs::Point> curve_points;
    double accumulated_dist = 0.0;
    auto current = it;
    
    while (current != path.poses.end() && accumulated_dist < CURVATURE_CALC_DIST) {
        curve_points.push_back(current->pose.position);
        
        if (std::next(current) != path.poses.end()) {
            double dx = std::next(current)->pose.position.x - current->pose.position.x;
            double dy = std::next(current)->pose.position.y - current->pose.position.y;
            accumulated_dist += std::hypot(dx, dy);
        }
        current = std::next(current);
    }

    // 충분한 점이 없으면 반환
    if (curve_points.size() < 3) return 0.0;

    // 시작, 중간, 끝 점 선택
    const auto& p1 = curve_points.front();
    const auto& p2 = curve_points[curve_points.size()/2];
    const auto& p3 = curve_points.back();

    // Menger 곡률 계산
    double x1 = p1.x, y1 = p1.y;
    double x2 = p2.x, y2 = p2.y;
    double x3 = p3.x, y3 = p3.y;

    double area = ((x2-x1)*(y3-y1) - (x3-x1)*(y2-y1)) / 2.0;
    double d1 = std::hypot(x2-x1, y2-y1);
    double d2 = std::hypot(x3-x2, y3-y2);
    double d3 = std::hypot(x1-x3, y1-y3);

    if (d1 * d2 * d3 == 0) return 0.0;

    double measured_curvature = std::abs(4 * area / (d1 * d2 * d3));
    
    // 칼만 필터 적용
    double filtered_curvature = applyKalmanFilter(measured_curvature);
    
    return filtered_curvature;
}

double HybridControl::applyKalmanFilter(double measured_curvature) {
    // Predict
    double prediction = curvature_estimate;
    estimate_uncertainty += process_noise;

    // Update
    double kalman_gain = estimate_uncertainty / (estimate_uncertainty + measurement_noise);
    curvature_estimate = prediction + kalman_gain * (measured_curvature - prediction);
    estimate_uncertainty = (1 - kalman_gain) * estimate_uncertainty;

    return curvature_estimate;
}

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
                    double curvature = calculateCurvature(this->localpath, closest_point);

                    while (steer_s > M_PI){
                        steer_s -= 2 * M_PI;
                    }
                    while (steer_s < -M_PI){
                        steer_s +=  2 * M_PI;
                    }

                    speed_control(curvature);
                    steer_control(curvature);

                    double steer_hy = steer_p * k_p + steer_s * k_s;
                    double steer = std::max(-35 * M_PI / 180, std::min(steer_hy, 35 * M_PI / 180));

                    if (target_vel < 0) {
                        target_vel = 1;
                    }
            
                    ctrl_cmd_msg.linear.x = std::max(0.0, target_vel);
                    ctrl_cmd_msg.angular.z = - steer;

                    // RMSE 계산
                    this->cte_sum += crossTrackError * crossTrackError;  // CTE 제곱 합
                    this->cte_count++;                                  // CTE 샘플 수 증가
                    double rmse = std::sqrt(this->cte_sum / this->cte_count);  // RMSE 계산

                    std::cout << std::fixed << std::setprecision(2);
                    std::cout << "k_p : " << k_p << " | k_s : " << k_s << std::endl;
                    std::cout << "pathYaw : " << pathYaw << " | CTE : " << crossTrackError << " | LD : " << LD << std::endl;
                    std::cout << "steer : " << steer * 180 / M_PI << " | steer_s : " << steer_s * 180 / M_PI << " | steer_p : " << steer_p * 180 / M_PI << std::endl;
                    std::cout << "Target Vel : " << target_vel << " | Current Vel : " << current_vel  << std::endl;
                    std::cout << "curvature : " << curvature << " | vel_penalty : " << vel_penalty << std::endl;
                    std::cout << "Map_state :" << map_state << " | Car_state :" << car_state << std::endl;
                    std::cout << "RMSE : " << rmse << std::endl;
                    
                    break;
                }
            }
        }

        if (!is_look_forward_point) {
            ctrl_cmd_msg.linear.x = 0;
            ctrl_cmd_msg.angular.z  = 0;
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
