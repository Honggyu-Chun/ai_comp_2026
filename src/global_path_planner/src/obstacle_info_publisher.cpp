// /Object_topic (morai_msgs/ObjectStatusList, MORAI GT) 를 구독해 각 객체를
// 자차 base_link 상대좌표(SI 단위)로 변환하고 /obstacle_info 로 발행한다.
// 상대변환용 pose 는 객체와 동일한 MORAI 월드 프레임인 /Ego_topic(EgoVehicleStatus)
// 을 쓴다 — /gps_utm_odom(UTM)과 섞으면 원점 오프셋으로 빼셈이 깨질 수 있어서다.

#include <ros/ros.h>

#include <morai_msgs/EgoVehicleStatus.h>
#include <morai_msgs/ObjectStatus.h>
#include <morai_msgs/ObjectStatusList.h>

#include <katri_msgs/ObstacleInfo.h>
#include <katri_msgs/ObstacleInfoArray.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <cmath>

namespace
{
constexpr double kDeg2Rad = M_PI / 180.0;
constexpr double kKphToMps = 1.0 / 3.6;

double normalizeAngle(const double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));  // odom_path_publisher.cpp:253 와 동일
}
}  // namespace

class ObstacleInfoPublisher
{
public:
    ObstacleInfoPublisher()
        : ego_x_(0.0), ego_y_(0.0), ego_yaw_(0.0),
          cos_yaw_(1.0), sin_yaw_(0.0), got_ego_(false)
    {
        ros::NodeHandle nh;
        ego_sub_ = nh.subscribe("/Ego_topic", 1, &ObstacleInfoPublisher::egoCallback, this);
        object_sub_ = nh.subscribe("/Object_topic", 1, &ObstacleInfoPublisher::objectCallback, this);
        obstacle_pub_ = nh.advertise<katri_msgs::ObstacleInfoArray>("/obstacle_info", 1);
        marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/obstacle_markers", 1);
    }

private:
    void egoCallback(const morai_msgs::EgoVehicleStatus::ConstPtr& msg)
    {
        ego_x_ = msg->position.x;
        ego_y_ = msg->position.y;
        // ponytail: EgoVehicleStatus.heading 는 [deg], ENU 반시계 + 로 가정.
        //           부호/기준이 어긋나면 회전 시나리오에서 상대 y 가 뒤집힌다 → sim 실측 확인.
        ego_yaw_ = msg->heading * kDeg2Rad;
        cos_yaw_ = std::cos(ego_yaw_);
        sin_yaw_ = std::sin(ego_yaw_);
        got_ego_ = true;
    }

    void objectCallback(const morai_msgs::ObjectStatusList::ConstPtr& msg)
    {
        if (!got_ego_) {
            ROS_WARN_THROTTLE(2.0, "[obstacle_info] waiting for /Ego_topic");
            return;
        }

        katri_msgs::ObstacleInfoArray out;
        out.header = msg->header;
        out.header.frame_id = "base_link";

        appendList(msg->npc_list, out);
        appendList(msg->pedestrian_list, out);
        appendList(msg->obstacle_list, out);

        obstacle_pub_.publish(out);
        publishMarkers(out);
    }

    // 타입별로 모양·색을 달리해 RViz(base_link)에 표시한다.
    // 보행자 → 빨강 원기둥, NPC → 파랑 큐브, 정적 → 주황 큐브.
    void publishMarkers(const katri_msgs::ObstacleInfoArray& obstacles) const
    {
        visualization_msgs::MarkerArray markers;

        // 이전 프레임 마커를 매번 싹 지운다(개수가 줄어도 잔상이 안 남음).
        visualization_msgs::Marker clear;
        clear.header = obstacles.header;
        clear.action = visualization_msgs::Marker::DELETEALL;
        markers.markers.push_back(clear);

        int id = 0;
        for (const auto& ob : obstacles.obstacles) {
            markers.markers.push_back(toMarker(ob, id++, obstacles.header));
        }
        marker_pub_.publish(markers);
    }

    visualization_msgs::Marker toMarker(const katri_msgs::ObstacleInfo& ob,
                                        const int id,
                                        const std_msgs::Header& header) const
    {
        visualization_msgs::Marker m;
        m.header = header;
        m.ns = "obstacles";
        m.id = id;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = ob.x_m;
        m.pose.position.y = ob.y_m;
        m.pose.orientation.z = std::sin(ob.yaw_rad * 0.5);
        m.pose.orientation.w = std::cos(ob.yaw_rad * 0.5);
        m.color.a = 0.8f;

        if (ob.is_pedestrian) {                       // 보행자: 빨강 원기둥
            m.type = visualization_msgs::Marker::CYLINDER;
            const double d = std::max(ob.width_m, 0.4);
            m.scale.x = d;
            m.scale.y = d;
            m.scale.z = 1.7;
            m.pose.position.z = 0.85;
            m.color.r = 1.0f;
        } else {                                      // 차량/정적: 큐브 (길이×폭)
            m.type = visualization_msgs::Marker::CUBE;
            m.scale.x = std::max(ob.length_m, 0.2);
            m.scale.y = std::max(ob.width_m, 0.2);
            m.scale.z = 1.5;
            m.pose.position.z = 0.75;
            if (ob.type == 1) {                       // NPC 차량: 파랑
                m.color.b = 1.0f;
            } else {                                  // 정적 장애물: 주황
                m.color.r = 1.0f;
                m.color.g = 0.55f;
            }
        }
        return m;
    }

    void appendList(const std::vector<morai_msgs::ObjectStatus>& list,
                    katri_msgs::ObstacleInfoArray& out) const
    {
        for (const auto& obj : list) {
            out.obstacles.push_back(toObstacleInfo(obj));
        }
    }

    katri_msgs::ObstacleInfo toObstacleInfo(const morai_msgs::ObjectStatus& obj) const
    {
        // ENU → base_link (odom_path_publisher.cpp:433 transformPointToBaseLink 와 동일 회전)
        const double dx = obj.position.x - ego_x_;
        const double dy = obj.position.y - ego_y_;
        const double x_m = cos_yaw_ * dx + sin_yaw_ * dy;
        const double y_m = -sin_yaw_ * dx + cos_yaw_ * dy;

        katri_msgs::ObstacleInfo info;
        info.id = obj.unique_id;
        info.type = obj.type;
        info.is_pedestrian = (obj.type == 0);
        info.x_m = x_m;
        info.y_m = y_m;
        info.yaw_rad = normalizeAngle(obj.heading * kDeg2Rad - ego_yaw_);
        // ponytail: MORAI 객체 velocity 단위 km/h 가정(ObjectStatus.msg 주석). 실측 확인 필요(설계 열린항목 #7).
        info.speed_mps = std::hypot(obj.velocity.x, obj.velocity.y) * kKphToMps;
        // MORAI ObjectStatus.size 실측 규약 = (length, width, height).
        // (msg 파일 주석의 "(width, length, height)"는 틀림 — 라이브 NPC size.x≈5m=길이로 확인)
        info.length_m = obj.size.x;
        info.width_m = obj.size.y;
        info.distance_m = std::hypot(x_m, y_m);
        return info;
    }

    ros::Subscriber ego_sub_;
    ros::Subscriber object_sub_;
    ros::Publisher obstacle_pub_;
    ros::Publisher marker_pub_;

    double ego_x_;
    double ego_y_;
    double ego_yaw_;
    double cos_yaw_;
    double sin_yaw_;
    bool got_ego_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_info_publisher");
    ObstacleInfoPublisher node;
    ros::spin();
    return 0;
}
