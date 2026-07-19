#include "ros/ros.h"
#include "ros/package.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Quaternion.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <queue>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <vector>
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <stdlib.h>
#include <signal.h>

///namespace
using namespace std;
using namespace cv;

nav_msgs::Odometry::ConstPtr currentPose;
nav_msgs::OccupancyGrid region_map;
ros::Publisher region_state_pub;

string state_string = "go";
string final_state = "go";
string gps_state = "unknown";
string gps_quat_state = "unknown";
string traffic_light_state = "unknown";
// string sign_state = "unknown";


bool region_map_loaded = false;
int pose_count = 0;
const string state_table[13] = {"go","traffic_light_1","traffic_light_2","traffic_light_3","traffic_light_4","static_obs","gps_fail","end","slow_down_for_traffic_light3","boost","delivery","slow_downpm", "stop"};
// line 169 point=gray scale     255            20             40             60                    80         100           120      140                      160                180       200          220         240                
// line 177 table index=point/20                1                                 2                     3                     4                     5             6            7                        8                   9       10          11           12    

int state_string_result(void)//내가 현재 신호등 구역에 있는가(1) 없는가(0)을 판단해주는 함수
{
    if(state_string == "slow_down_for_traffic_light0" || 
        state_string == "slow_down_for_traffic_light1" || 
        state_string == "slow_down_for_traffic_light2" || 
        state_string == "slow_down_for_traffic_light3")
    {
        return 1;
    }
    else return 0;
}
void poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    pose_count++;
    std::queue<nav_msgs::Odometry::ConstPtr> temp_q;
    temp_q.push(msg);
    currentPose = temp_q.front();
    //cout << msg->pose.covariance[0] << ", " << msg->pose.covariance[7] << ", " << msg->pose.covariance[14] << "\n";
    if(msg->pose.covariance[0] > 3.0 || msg->pose.covariance[7] > 3.0 || msg->pose.covariance[14] > 10.0) //meter
    {
        gps_state = "fail";
    }
    else gps_state = "good";
    temp_q.pop();
}
// void signstateCallback(const std_msgs::String::ConstPtr& msg)
// {
//     sign_state = msg->data.c_str();
// }
void trafficlightCallback(const std_msgs::String::ConstPtr& msg)
{
    traffic_light_state = msg->data.c_str();
}
void gpsquatCallback(const geometry_msgs::Quaternion::ConstPtr& msg)
{
    double w = msg->w;
    double x = msg->x;
    double y = msg->y;
    double z = msg->z;
    if(w == 1.0 && x == 0.0 && y == 0.0 && z == 0.0) gps_quat_state = "fail";
    else gps_quat_state = "good";
}
////////start main
int main(int argc, char **argv)
{
    bool state_ok = true;

    ros::init(argc, argv, "ai_region_state_publisher");
    ros::NodeHandle n;
    region_state_pub = n.advertise<std_msgs::String>("/state",1);
    ros::Subscriber traffic_light_sub = n.subscribe("/traffic_light_state", 1, trafficlightCallback);
    // ros::Subscriber sign_state_sub = n.subscribe("/sign_state", 1, signstateCallback);  
    ros::Subscriber pose_sub = n.subscribe("/gps_utm_odom",10,poseCallback);
    ros::Subscriber gps_quat_sub = n.subscribe("/gphdt_quat_pub",10,gpsquatCallback);
    ros::ServiceClient map_client3 = n.serviceClient<nav_msgs::GetMap>("/ai_region_map/static_map");
    nav_msgs::GetMap srv_region;

    ros::Time timer_time, current_time;
    ros::Rate r(10);

    string arg_str = "unknown"; std_msgs::String msg;
                std::stringstream ss;
                ss << final_state;
                msg.data = ss.str();
                region_state_pub.publish(msg);
    cout << argc;
    if(argc == 2) arg_str = argv[1];

    std::string file_path = ros::package::getPath("global_path_planner");
    cv::Mat region_image = cv::imread(file_path+"/map_data/ai_region_"+arg_str+".png",1);
    cout << file_path;

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
            else cout << "failed to load region map!" << endl;
        }

        if(pose_count != 0 && region_map_loaded)
        {
            if(state_ok)
            {
                state_ok = false;
                cout << "algorithm works!\n";
            }

            double current_x = currentPose->pose.pose.position.x;
            double current_y = currentPose->pose.pose.position.y;
            double resolution = region_map.info.resolution;

            Eigen::Matrix4f transformMatrix;
            transformMatrix << 1, 0, 0, current_x,
                               0, 1, 0, current_y,
                               0, 0, 1, 0,
                               0, 0, 0, 1;

            Eigen::Vector4f vehicle_coord(0, 0, 0, 1);
            Eigen::Vector4f map_coord = transformMatrix * vehicle_coord;

            cv::Vec3b original_map_data;
            cv::Vec3b traff_map_data;

            if(!region_image.empty() && region_map.info.width > 0 && region_map.info.height > 0 && resolution > 0.0)
            {
                double ox = region_map.info.origin.position.x;
                double oy = region_map.info.origin.position.y;
                double qx = region_map.info.origin.orientation.x;
                double qy = region_map.info.origin.orientation.y;
                double qz = region_map.info.origin.orientation.z;
                double qw = region_map.info.origin.orientation.w;
                tf2::Quaternion q(qx, qy, qz, qw);
                double roll, pitch, yaw;
                tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

                double dx = map_coord(0) - ox;
                double dy = map_coord(1) - oy;
                double c = cos(yaw);
                double s = sin(yaw);
                double gx_m =  c*dx + s*dy;
                double gy_m = -s*dx + c*dy;

                double gx = gx_m / resolution;
                double gy = gy_m / resolution;

                double sx = static_cast<double>(region_image.cols)  / static_cast<double>(region_map.info.width);
                double sy = static_cast<double>(region_image.rows) / static_cast<double>(region_map.info.height);

                int ix = static_cast<int>(floor(gx * sx));
                int iy = region_image.rows - 1 - static_cast<int>(floor(gy * sy));

                if(0 <= ix && ix < region_image.cols && 0 <= iy && iy < region_image.rows)
                {
                    cout << "dddddddddddddd\n";
                    original_map_data = region_image.at<cv::Vec3b>(iy, ix);

                    int point = (original_map_data[0] + original_map_data[1] + original_map_data[2]) / 3;

                    if(point > 240)//grayscale ==255일 경우 -> GO를 나타냄
                    {
                        state_string = state_table[0];  //state_table[0] == "go"
                        final_state = state_table[0];
                    }
                    else //grayscale이 255가 아닐 경우 즉, "GO"를 표시하고 있지 않는 경우 -> 값 확인후 해야될 미션 확인
                    {
                        int table_index = point / 20;

                        if(table_index > 0 && table_index < 13)//계산한 grayscale를 각 미션값(1~12)으로 변환
                        {
                            state_string = state_table[table_index];
                            final_state = state_table[table_index];

                            cout << "state_string : " << state_string << "\n";
                        }
                        else //table_index가 잘못들어갔을 경우 해당 값 확인을 위한 출력
                        {
                            cout << "table_index error result!!!!" << table_index << "\n";
                            cout << "table_index error result!!!!" << table_index << "\n";
                            cout << "table_index error result!!!!" << table_index << "\n";
                        }

                        /*
                        if(traffic_light_state == "stop"&&state_string_result()==1)
                        {
                            final_state = "stop";
                            cout << "stop for red!\n";
                        }
                        else if(traffic_light_state == "go"&&state_string_result()==1) //msg=go
                        {
                            final_state = "go";
                            cout << "gogogogogogo!\n";
                        }
                        else if(state_string_result()==1) final_state = "slow_down_for_traffic_light";*/ // 아래의 if문과 내용 동일

                        if(state_string_result()==1)//새로 개발한 구문
                        {
                            // if(traffic_light_state == "stop") final_state ="stop",cout << "stop for red!\n";
                            // else if(traffic_light_state == "go") final_state ="go",cout << "gogogogogogo!\n";
                            // else final_state = "slow_down_for_traffic_light",cout << "slow_down_for_traffic_light!\n";
                        }
                        else if(state_string_result()==0)
                        {
                            // if(state_string == "parking") final_state = "parking",cout << "parking\n";
                            // else if(state_string == "big_obs") final_state = "big_obs",cout << "big_obs\n";
                            // else if(state_string == "static_obs") final_state = "static_obs",cout << "static_obs\n";
                            // else if(state_string == "school_zone") final_state = "school_zone",cout << "school_zone\n";
                            // else if(state_string == "stop_crosswalk") final_state = "stop_crosswalk",cout << "stop_crosswalk\n";
                            // else if(state_string == "slow_downpm") final_state = "slow_downpm",cout << "slow_downpm\n";
                            // else if(state_string == "boost") final_state = "boost",cout << "boost\n";
                            // else if(state_string == "delivery")
                            // {
                            //     // if(sign_state == "delivery_go") final_state = "go"; 
                            //     // else if(sign_state == "delivery_stop") final_state = "stop";
                            //     final_state = "delivery";
                            // }
                            // else final_state = "go";
                            cout << "dddddddddddddd\n" ;
                            final_state = state_string;
                        }
                    }
                    //mission publish
                    // final_state="parking";
                    
                    std_msgs::String msg;
                    std::stringstream ss;
                    ss << final_state;
                    msg.data = ss.str();
                    region_state_pub.publish(msg);
                }
            }

            /*delivery mission publish*/
            // final_state="parking";
            std_msgs::String msg;
            std::stringstream ss;
            ss << final_state;
            msg.data = ss.str();
            region_state_pub.publish(msg);
        }
        else cout << "pose call back failed!" << endl;
        r.sleep();
        ros::spinOnce();
        current_time = ros::Time::now();
    }
    ros::spin();
    return 0;
}
