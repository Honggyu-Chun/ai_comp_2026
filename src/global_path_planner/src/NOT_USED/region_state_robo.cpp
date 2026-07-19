#include "ros/ros.h"
#include "ros/package.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Quaternion.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <queue>
#include <vector>
#include <string>
#include <stdlib.h>
#include <signal.h>
#include <iostream>
#include <Eigen/Dense>

using namespace std;
using namespace cv;

ros::Publisher region_state_pub;
ros::Publisher legacy_state_pub;
nav_msgs::OccupancyGrid region_map;
nav_msgs::Odometry::ConstPtr currentPose;

string state_string = "go";
string final_state = "go";
string gps_quat_state = "unknown";
string gps_state = "unknown";

// region grayscale 정책
// 255: go, 20: overtake, 40: slow, 60: warning, 80: curve
const string state_table[5] = {"go", "overtake", "slow", "warning", "curve"};
//                gray scale    255      20         40      60         80

int pose_count = 0;
bool region_map_loaded = false;

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    pose_count++;
    currentPose = msg;

    if(msg->pose.covariance[0] > 3.0 || msg->pose.covariance[7] > 3.0 || msg->pose.covariance[14] > 10.0) 
        gps_state = "fail";
    else 
        gps_state = "good";
}

void gpsquatCallback(const geometry_msgs::Quaternion::ConstPtr& msg)
{
    double w = msg->w;
    double x = msg->x;
    double y = msg->y;
    double z = msg->z;

    if(w == 1.0 && x == 0.0 && y == 0.0 && z == 0.0) 
        gps_quat_state = "fail";
    else 
        gps_quat_state = "good";
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "region_state_robo");
    ros::NodeHandle n;
    
    ros::Subscriber pose_sub = n.subscribe("/gps_utm_odom",10,poseCallback);
    ros::Subscriber gps_quat_sub = n.subscribe("/gphdt_quat_pub",10,gpsquatCallback);
    ros::ServiceClient map_client3 = n.serviceClient<nav_msgs::GetMap>("/region_map/static_map");
    // 로컬 플래너는 /region을 구독하므로 /region으로 발행.
    // 기존 호환을 위해 /state도 같이 발행.
    region_state_pub = n.advertise<std_msgs::String>("/region", 1);
    legacy_state_pub = n.advertise<std_msgs::String>("/state", 1);

    ros::Rate r(10);
    nav_msgs::GetMap srv_region;
    bool state_ok = true;
    string arg_str = "unknown";
    cout << argc;

    if(argc == 2) arg_str = argv[1];
    std::string file_path = ros::package::getPath("global_path_planner_robo");
    if (file_path.empty()) {
        file_path = ros::package::getPath("global_path_planner_robo");
    }

    cout << file_path;
    cv::Mat region_image = cv::imread(file_path+"/map_data/region_"+arg_str+".png",1);

    while (ros::ok())
    {
        if(!region_map_loaded)
        {
            if(map_client3.call(srv_region))
            {
                cout << "region_call!" << endl;
                region_map = srv_region.response.map;
                region_map.data = srv_region.response.map.data;
                cout << region_map.info.width << " x " << region_map.info.height << endl;
                region_map_loaded = true;
            }
            else 
                cout << "failed to load region map!" << endl;
        }
        
        if(pose_count != 0 && region_map_loaded)
        {
            if(state_ok)
            {
                state_ok = false;
                cout << "algorithm works!\n";
            }

            // 차량 좌표계를 맵 좌표계로 변환
            double current_x = currentPose->pose.pose.position.x;
            double current_y = currentPose->pose.pose.position.y;
            double resolution = region_map.info.resolution;

            // 변환 행렬 초기화
            Eigen::Matrix4f transformMatrix;
            transformMatrix << 1, 0, 0, current_x,
                               0, 1, 0, current_y,
                               0, 0, 1, 0,
                               0, 0, 0, 1;

            Eigen::Vector4f vehicle_coord(0, 0, 0, 1);
            Eigen::Vector4f map_coord = transformMatrix * vehicle_coord;

            int region_map_x = (map_coord(0) - region_map.info.origin.position.x) / resolution;
            int region_map_y = (map_coord(1) - region_map.info.origin.position.y) / resolution;
            cv::Vec3b original_map_data;
            
            if(region_map_x < region_image.cols && region_image.rows - region_map_y < region_image.rows && region_map_x > 0 && region_image.rows - region_map_y > 0)
            {
                original_map_data = region_image.at<cv::Vec3b>(region_image.rows - region_map_y, region_map_x);

                int point = (original_map_data[0] + original_map_data[1] + original_map_data[2]) / 3;

                if(point > 240)
                {
                    state_string = state_table[0];
                    final_state = state_table[0];
                }
                else
                {
                    int table_index = point / 20;
                    if (table_index > 0 && table_index < 5)
                    {
                        state_string = state_table[table_index];
                        final_state = state_table[table_index];
                    }

                    // cout << "state_string : " << state_string << "\n";
                    // if(state_string == "overtake") 
                    //     final_state = "overtake";
                    // else if(state_string == "slow_down") 
                    //     final_state = "slow_down";
                    // else if(state_string == "slow_down2") 
                    //     final_state = "slow_down2";

                    cout << "final_string : " << final_state << "\n";
                }

                if(gps_state == "fail" || gps_quat_state == "fail")
                {
                    // final_state = "gps_fail";
                    //24.0325 jbk -> ?????????????????
                }

                std_msgs::String msg;
                std::stringstream ss;
                ss << final_state;
                msg.data = ss.str();
                region_state_pub.publish(msg);
                legacy_state_pub.publish(msg);
            }
            else
            {
                std_msgs::String msg;
                std::stringstream ss;
                msg.data = ss.str();
                //ROS_INFO("%s", msg.data.c_str());
                // region_state_pub.publish(msg);
            }
        }
        else 
            cout << "pose call back failed!" << endl;
        
        r.sleep();
        ros::spinOnce();
    }
    
    ros::spin();
    return 0;
}
