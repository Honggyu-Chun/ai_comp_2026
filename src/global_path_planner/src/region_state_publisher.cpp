#include <ros/package.h>
#include <ros/ros.h>

#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <opencv2/opencv.hpp>

#include <array>
#include <cmath>
#include <string>

namespace
{
const std::array<std::string, 13> kStateTable = {
    "go",
    "traffic_light_1",
    "traffic_light_2",
    "traffic_light_3",
    "traffic_light_4",
    "static_obs",
    "gps_fail",
    "end",
    "slow_down_for_traffic_light3",
    "boost",
    "delivery",
    "slow_downpm",
    "stop"};

bool isTrafficLightSlowdownRegion(const std::string& state)
{
    return state == "slow_down_for_traffic_light0" ||
           state == "slow_down_for_traffic_light1" ||
           state == "slow_down_for_traffic_light2" ||
           state == "slow_down_for_traffic_light3";
}
}  // namespace

class RegionStatePublisher
{
public:
    explicit RegionStatePublisher(const std::string& region_name)
        : nh_(),
          region_name_(region_name),
          state_string_("go"),
          final_state_("go"),
          gps_state_("unknown"),
          gps_quat_state_("unknown"),
          traffic_light_state_("unknown"),
          pose_count_(0),
          region_map_loaded_(false),
          map_cache_ready_(false),
          first_algorithm_log_(true)
    {
        region_state_pub_ = nh_.advertise<std_msgs::String>("/state", 1);
        traffic_light_sub_ = nh_.subscribe("/traffic_light_state", 1, &RegionStatePublisher::trafficLightCallback, this);
        pose_sub_ = nh_.subscribe("/gps_utm_odom", 10, &RegionStatePublisher::poseCallback, this);
        gps_quat_sub_ = nh_.subscribe("/gphdt_quat_pub", 10, &RegionStatePublisher::gpsQuatCallback, this);
        map_client_ = nh_.serviceClient<nav_msgs::GetMap>("/region_map/static_map");

        const std::string package_path = ros::package::getPath("global_path_planner");
        const std::string image_path = package_path + "/map_data/region_" + region_name_ + ".png";
        region_image_ = cv::imread(image_path, 1);

        if (region_image_.empty()) {
            ROS_WARN("[region_state_publisher] failed to load region image: %s", image_path.c_str());
        } else {
            ROS_INFO("[region_state_publisher] loaded region image: %s (%d x %d)",
                     image_path.c_str(), region_image_.cols, region_image_.rows);
        }

        publishState();
    }

    void spin()
    {
        ros::Rate rate(10);
        while (ros::ok()) {
            if (!region_map_loaded_) {
                loadRegionMap();
            }

            if (pose_count_ != 0 && region_map_loaded_) {
                if (first_algorithm_log_) {
                    first_algorithm_log_ = false;
                    ROS_INFO("[region_state_publisher] algorithm works");
                }

                updateStateFromCurrentPose();
                publishState();
            } else {
                ROS_WARN_THROTTLE(2.0, "[region_state_publisher] waiting for pose or region map");
            }

            rate.sleep();
            ros::spinOnce();
        }
    }

private:
    struct CachedMapGeometry
    {
        double origin_x = 0.0;
        double origin_y = 0.0;
        double cos_yaw = 1.0;
        double sin_yaw = 0.0;
        double inv_resolution = 1.0;
        double scale_x = 1.0;
        double scale_y = 1.0;
        int image_cols = 0;
        int image_rows = 0;
    };

    void poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        ++pose_count_;
        current_pose_ = msg;

        if (msg->pose.covariance[0] > 3.0 ||
            msg->pose.covariance[7] > 3.0 ||
            msg->pose.covariance[14] > 10.0) {
            gps_state_ = "fail";
        } else {
            gps_state_ = "good";
        }
    }

    void trafficLightCallback(const std_msgs::String::ConstPtr& msg)
    {
        traffic_light_state_ = msg->data;
    }

    void gpsQuatCallback(const geometry_msgs::Quaternion::ConstPtr& msg)
    {
        if (msg->w == 1.0 && msg->x == 0.0 && msg->y == 0.0 && msg->z == 0.0) {
            gps_quat_state_ = "fail";
        } else {
            gps_quat_state_ = "good";
        }
    }

    void loadRegionMap()
    {
        nav_msgs::GetMap srv_region;
        if (!map_client_.call(srv_region)) {
            ROS_WARN_THROTTLE(2.0, "[region_state_publisher] failed to load region map");
            return;
        }

        region_map_info_ = srv_region.response.map.info;
        region_map_loaded_ = true;
        updateMapCache();

        ROS_INFO("[region_state_publisher] region map loaded: %u x %u, resolution=%.6f",
                 region_map_info_.width,
                 region_map_info_.height,
                 region_map_info_.resolution);
    }

    void updateMapCache()
    {
        map_cache_ready_ = false;
        if (region_image_.empty() ||
            region_map_info_.width == 0 ||
            region_map_info_.height == 0 ||
            region_map_info_.resolution <= 0.0) {
            ROS_WARN("[region_state_publisher] invalid region image or map metadata");
            return;
        }

        const auto& origin = region_map_info_.origin;
        tf2::Quaternion q(origin.orientation.x,
                          origin.orientation.y,
                          origin.orientation.z,
                          origin.orientation.w);
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        map_cache_.origin_x = origin.position.x;
        map_cache_.origin_y = origin.position.y;
        map_cache_.cos_yaw = std::cos(yaw);
        map_cache_.sin_yaw = std::sin(yaw);
        map_cache_.inv_resolution = 1.0 / region_map_info_.resolution;
        map_cache_.scale_x = static_cast<double>(region_image_.cols) / static_cast<double>(region_map_info_.width);
        map_cache_.scale_y = static_cast<double>(region_image_.rows) / static_cast<double>(region_map_info_.height);
        map_cache_.image_cols = region_image_.cols;
        map_cache_.image_rows = region_image_.rows;
        map_cache_ready_ = true;
    }

    void updateStateFromCurrentPose()
    {
        if (!current_pose_ || !map_cache_ready_) {
            return;
        }

        const double current_x = current_pose_->pose.pose.position.x;
        const double current_y = current_pose_->pose.pose.position.y;
        const double dx = current_x - map_cache_.origin_x;
        const double dy = current_y - map_cache_.origin_y;

        const double gx_m = map_cache_.cos_yaw * dx + map_cache_.sin_yaw * dy;
        const double gy_m = -map_cache_.sin_yaw * dx + map_cache_.cos_yaw * dy;
        const double gx = gx_m * map_cache_.inv_resolution;
        const double gy = gy_m * map_cache_.inv_resolution;

        const int ix = static_cast<int>(std::floor(gx * map_cache_.scale_x));
        const int iy = map_cache_.image_rows - 1 - static_cast<int>(std::floor(gy * map_cache_.scale_y));

        if (ix < 0 || ix >= map_cache_.image_cols || iy < 0 || iy >= map_cache_.image_rows) {
            return;
        }

        const cv::Vec3b original_map_data = region_image_.at<cv::Vec3b>(iy, ix);
        const int point = (original_map_data[0] + original_map_data[1] + original_map_data[2]) / 3;
        updateStateFromGrayValue(point);
    }

    void updateStateFromGrayValue(const int point)
    {
        if (point > 240) {
            state_string_ = kStateTable[0];
            final_state_ = kStateTable[0];
            return;
        }

        const int table_index = point / 20;
        if (table_index > 0 && table_index < static_cast<int>(kStateTable.size())) {
            state_string_ = kStateTable[static_cast<std::size_t>(table_index)];
            final_state_ = kStateTable[static_cast<std::size_t>(table_index)];
            ROS_DEBUG("[region_state_publisher] state_string: %s", state_string_.c_str());
        } else {
            ROS_WARN_THROTTLE(1.0, "[region_state_publisher] table_index error: %d", table_index);
        }

        if (isTrafficLightSlowdownRegion(state_string_)) {
            return;
        }

        final_state_ = state_string_;
    }

    void publishState()
    {
        std_msgs::String msg;
        msg.data = final_state_;
        region_state_pub_.publish(msg);
    }

    ros::NodeHandle nh_;
    ros::Publisher region_state_pub_;
    ros::Subscriber traffic_light_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber gps_quat_sub_;
    ros::ServiceClient map_client_;

    std::string region_name_;
    cv::Mat region_image_;
    nav_msgs::MapMetaData region_map_info_;
    CachedMapGeometry map_cache_;
    nav_msgs::Odometry::ConstPtr current_pose_;

    std::string state_string_;
    std::string final_state_;
    std::string gps_state_;
    std::string gps_quat_state_;
    std::string traffic_light_state_;

    int pose_count_;
    bool region_map_loaded_;
    bool map_cache_ready_;
    bool first_algorithm_log_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "region_state_publisher");

    std::string region_name = "unknown";
    if (argc == 2) {
        region_name = argv[1];
    }

    RegionStatePublisher node(region_name);
    node.spin();
    return 0;
}
