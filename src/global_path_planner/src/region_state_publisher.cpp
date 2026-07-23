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

#include <algorithm>
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
    explicit RegionStatePublisher(const std::string& default_region_name)
        : nh_(),
          pnh_("~"),
          region_name_(default_region_name),
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
        loadParams();

        region_state_pub_ = nh_.advertise<std_msgs::String>(state_topic_, publish_queue_size_);
        traffic_light_sub_ = nh_.subscribe(traffic_light_topic_, subscribe_queue_size_, &RegionStatePublisher::trafficLightCallback, this);
        pose_sub_ = nh_.subscribe(odom_topic_, subscribe_queue_size_, &RegionStatePublisher::poseCallback, this);
        gps_quat_sub_ = nh_.subscribe(gps_quat_topic_, subscribe_queue_size_, &RegionStatePublisher::gpsQuatCallback, this);
        map_client_ = nh_.serviceClient<nav_msgs::GetMap>(region_map_service_);

        const std::string package_path = ros::package::getPath("global_path_planner");
        if (region_image_path_.empty()) {
            region_image_path_ = package_path + "/" + region_image_folder_ + "/" +
                                 region_image_prefix_ + region_name_ + region_image_extension_;
        }
        region_image_ = cv::imread(region_image_path_, 1);

        if (region_image_.empty()) {
            ROS_WARN("[region_state_publisher] failed to load region image: %s", region_image_path_.c_str());
        } else {
            ROS_INFO("[region_state_publisher] loaded region image: %s (%d x %d)",
                     region_image_path_.c_str(), region_image_.cols, region_image_.rows);
        }

        publishState();
    }

    void spin()
    {
        ros::Rate rate(rate_hz_);
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

    void loadParams()
    {
        pnh_.param("region_name", region_name_, region_name_);
        pnh_.param("state_topic", state_topic_, state_topic_);
        pnh_.param("traffic_light_topic", traffic_light_topic_, traffic_light_topic_);
        pnh_.param("odom_topic", odom_topic_, odom_topic_);
        pnh_.param("gps_quat_topic", gps_quat_topic_, gps_quat_topic_);
        pnh_.param("region_map_service", region_map_service_, region_map_service_);
        pnh_.param("region_image_path", region_image_path_, region_image_path_);
        pnh_.param("region_image_folder", region_image_folder_, region_image_folder_);
        pnh_.param("region_image_prefix", region_image_prefix_, region_image_prefix_);
        pnh_.param("region_image_extension", region_image_extension_, region_image_extension_);
        pnh_.param("rate_hz", rate_hz_, rate_hz_);
        pnh_.param("subscribe_queue_size", subscribe_queue_size_, subscribe_queue_size_);
        pnh_.param("publish_queue_size", publish_queue_size_, publish_queue_size_);
        pnh_.param("gps_cov_x_threshold", gps_cov_x_threshold_, gps_cov_x_threshold_);
        pnh_.param("gps_cov_y_threshold", gps_cov_y_threshold_, gps_cov_y_threshold_);
        pnh_.param("gps_cov_z_threshold", gps_cov_z_threshold_, gps_cov_z_threshold_);

        rate_hz_ = std::max(1.0, rate_hz_);
        subscribe_queue_size_ = std::max(1, subscribe_queue_size_);
        publish_queue_size_ = std::max(1, publish_queue_size_);
        gps_cov_x_threshold_ = std::max(0.0, gps_cov_x_threshold_);
        gps_cov_y_threshold_ = std::max(0.0, gps_cov_y_threshold_);
        gps_cov_z_threshold_ = std::max(0.0, gps_cov_z_threshold_);
    }

    void poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        ++pose_count_;
        current_pose_ = msg;

        if (msg->pose.covariance[0] > gps_cov_x_threshold_ ||
            msg->pose.covariance[7] > gps_cov_y_threshold_ ||
            msg->pose.covariance[14] > gps_cov_z_threshold_) {
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
    ros::NodeHandle pnh_;
    ros::Publisher region_state_pub_;
    ros::Subscriber traffic_light_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber gps_quat_sub_;
    ros::ServiceClient map_client_;

    std::string region_name_;
    std::string state_topic_ = "/state";
    std::string traffic_light_topic_ = "/traffic_light_state";
    std::string odom_topic_ = "/gps_utm_odom";
    std::string gps_quat_topic_ = "/gphdt_quat_pub";
    std::string region_map_service_ = "/region_map/static_map";
    std::string region_image_path_;
    std::string region_image_folder_ = "map_data";
    std::string region_image_prefix_ = "region_";
    std::string region_image_extension_ = ".png";
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
    double rate_hz_ = 10.0;
    int subscribe_queue_size_ = 10;
    int publish_queue_size_ = 1;
    double gps_cov_x_threshold_ = 3.0;
    double gps_cov_y_threshold_ = 3.0;
    double gps_cov_z_threshold_ = 10.0;
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
