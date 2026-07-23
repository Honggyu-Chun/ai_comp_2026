// MORAI ObjectInfo UDP(기본 포트 7505)를 직접 수신해 각 객체를 자차 base_link
// 상대좌표(SI 단위)로 변환하고 /obstacle_info · /obstacle_markers 로 발행한다.
//
// 이전에는 /Object_topic(ObjectStatusList) + /morai/ego_topic(EgoVehicleStatus)를
// 구독했으나, competition UDP bridge 전환 후 두 토픽 모두 발행되지 않는다.
//   - 객체: UDP ObjectInfo (MORAI 월드 ENU, pose_x/y, heading[deg]) 를 직접 파싱.
//   - 자차: /odometry/filtered (map 프레임 = 동일 MORAI 월드 ENU, 실측 확인). 이 프레임을
//     써야 객체-자차 빼셈이 원점 오프셋 없이 맞는다. quaternion → yaw.
//
// UDP 패킷 규약: MORAI_UDP_NetworkModule lib/define/ObjectInfo.py 와 1:1 (packed, LE).

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

#include <katri_msgs/ObstacleInfo.h>
#include <katri_msgs/ObstacleInfoArray.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <thread>

namespace
{
constexpr double kDeg2Rad = M_PI / 180.0;

double normalizeAngle(const double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));  // odom_path_publisher.cpp:253 와 동일
}

// MORAI ObjectInfo UDP 패킷 (lib/define/ObjectInfo.py 와 동일. ctypes _pack_=1 → packed).
#pragma pack(push, 1)
struct ObjectInfoData
{
    int16_t obj_id;
    int16_t objType;
    float pose_x, pose_y, pose_z;
    float heading;
    float size_x, size_y, size_z;
    float overhang, wheelbase, rearoverhang;
    float vel_x, vel_y, vel_z;
    float accel_x, accel_y, accel_z;
    char link_id[38];
};

struct ObjectInfoPacket
{
    char header[14];
    int32_t data_length;
    int32_t aux_data[3];
    int32_t sec;
    int32_t nsec;
    ObjectInfoData data[20];
    char tail[2];
};
#pragma pack(pop)
static_assert(sizeof(ObjectInfoData) == 106, "ObjectInfoData layout mismatch");
static_assert(sizeof(ObjectInfoPacket) == 2160, "ObjectInfoPacket layout mismatch");
}  // namespace

class ObstacleInfoPublisher
{
public:
    ObstacleInfoPublisher()
    {
        ros::NodeHandle nh;
        ros::NodeHandle pnh("~");

        pnh.param<std::string>("udp_ip", udp_ip_, "127.0.0.1");
        pnh.param<int>("udp_port", udp_port_, 7505);
        pnh.param<std::string>("ego_topic", ego_topic_, "/odometry/filtered");
        pnh.param<std::string>("frame_id", frame_id_, "base_link");
        pnh.param<double>("publish_rate_hz", publish_rate_hz_, 30.0);
        pnh.param<double>("ego_timeout", ego_timeout_, 0.5);
        // MORAI 객체 velocity 단위는 sim 실측으로 확정 필요(움직이는 NPC로 검증).
        // 기본값 1/3.6 = km/h→m/s (구 ObjectStatusList 규약과 동일). m/s 면 1.0 으로.
        pnh.param<double>("velocity_scale", velocity_scale_, 1.0 / 3.6);
        // MORAI ObjectInfo objType 규약(실측): 0=보행자, 1=차량(NPC), 2=정적장애물.
        pnh.param<int>("pedestrian_objtype", pedestrian_objtype_, 0);
        // ObjectInfo 에는 자차가 포함되지 않으므로 기본 비활성(-1). objType 로 특정 클래스를
        // 통째로 빼고 싶을 때만 그 값을 지정한다.
        pnh.param<int>("ego_objtype", ego_objtype_, -1);

        obstacle_pub_ = nh.advertise<katri_msgs::ObstacleInfoArray>("/obstacle_info", 1);
        marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/obstacle_markers", 1);
        ego_sub_ = nh.subscribe(ego_topic_, 1, &ObstacleInfoPublisher::egoCallback, this);

        if (!openSocket()) {
            ROS_FATAL("[obstacle_info] UDP bind %s:%d failed", udp_ip_.c_str(), udp_port_);
            ros::shutdown();
            return;
        }
        recv_thread_ = std::thread(&ObstacleInfoPublisher::recvLoop, this);
        timer_ = nh.createTimer(ros::Duration(1.0 / std::max(1.0, publish_rate_hz_)),
                                &ObstacleInfoPublisher::onTimer, this);
        ROS_INFO("[obstacle_info] UDP %s:%d, ego=%s", udp_ip_.c_str(), udp_port_, ego_topic_.c_str());
    }

    ~ObstacleInfoPublisher()
    {
        running_ = false;
        if (sock_fd_ >= 0) ::close(sock_fd_);
        if (recv_thread_.joinable()) recv_thread_.join();
    }

private:
    bool openSocket()
    {
        sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_fd_ < 0) return false;
        int reuse = 1;
        ::setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
        // 셧다운 시 recvfrom 이 빠져나오도록 200ms 타임아웃.
        timeval tv{};
        tv.tv_sec = 0;
        tv.tv_usec = 200000;
        ::setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(static_cast<uint16_t>(udp_port_));
        addr.sin_addr.s_addr = (udp_ip_ == "0.0.0.0") ? INADDR_ANY : ::inet_addr(udp_ip_.c_str());
        return ::bind(sock_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0;
    }

    void recvLoop()
    {
        ObjectInfoPacket buf;
        while (running_ && ros::ok()) {
            const ssize_t n = ::recvfrom(sock_fd_, &buf, sizeof(buf), 0, nullptr, nullptr);
            if (n < static_cast<ssize_t>(sizeof(ObjectInfoPacket))) continue;  // 타임아웃/불완전 패킷 무시
            {
                std::lock_guard<std::mutex> lk(pkt_mtx_);
                latest_ = buf;
                have_pkt_ = true;
            }
        }
    }

    void egoCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        const auto& q = msg->pose.pose.orientation;
        const double siny = 2.0 * (q.w * q.z + q.x * q.y);
        const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        std::lock_guard<std::mutex> lk(ego_mtx_);
        ego_x_ = msg->pose.pose.position.x;
        ego_y_ = msg->pose.pose.position.y;
        ego_yaw_ = std::atan2(siny, cosy);
        ego_stamp_ = ros::Time::now();
        got_ego_ = true;
    }

    void onTimer(const ros::TimerEvent&)
    {
        double ex, ey, eyaw;
        {
            std::lock_guard<std::mutex> lk(ego_mtx_);
            if (!got_ego_ || (ros::Time::now() - ego_stamp_).toSec() > ego_timeout_) {
                ROS_WARN_THROTTLE(2.0, "[obstacle_info] waiting for fresh ego (%s)", ego_topic_.c_str());
                return;
            }
            ex = ego_x_;
            ey = ego_y_;
            eyaw = ego_yaw_;
        }

        ObjectInfoPacket pkt;
        {
            std::lock_guard<std::mutex> lk(pkt_mtx_);
            if (!have_pkt_) return;
            pkt = latest_;
        }

        const double cos_yaw = std::cos(eyaw);
        const double sin_yaw = std::sin(eyaw);

        katri_msgs::ObstacleInfoArray out;
        out.header.stamp = ros::Time::now();
        out.header.frame_id = frame_id_;

        for (const auto& obj : pkt.data) {
            if (obj.obj_id == 0) break;                 // 빈 슬롯(종료 표시)
            if (obj.objType == ego_objtype_) continue;  // 자기 자신 제외
            out.obstacles.push_back(toObstacleInfo(obj, ex, ey, eyaw, cos_yaw, sin_yaw));
        }

        obstacle_pub_.publish(out);
        publishMarkers(out);
    }

    katri_msgs::ObstacleInfo toObstacleInfo(const ObjectInfoData& obj, double ex, double ey,
                                            double eyaw, double cos_yaw, double sin_yaw) const
    {
        // ENU(월드) → base_link (odom_path_publisher.cpp transformPointToBaseLink 와 동일 회전)
        const double dx = static_cast<double>(obj.pose_x) - ex;
        const double dy = static_cast<double>(obj.pose_y) - ey;
        const double x_m = cos_yaw * dx + sin_yaw * dy;
        const double y_m = -sin_yaw * dx + cos_yaw * dy;

        katri_msgs::ObstacleInfo info;
        info.id = obj.obj_id;
        info.type = obj.objType;
        info.is_pedestrian = (obj.objType == pedestrian_objtype_);
        info.x_m = x_m;
        info.y_m = y_m;
        info.yaw_rad = normalizeAngle(static_cast<double>(obj.heading) * kDeg2Rad - eyaw);
        info.speed_mps = std::hypot(obj.vel_x, obj.vel_y) * velocity_scale_;
        info.length_m = obj.size_x;   // MORAI size = (length, width, height)
        info.width_m = obj.size_y;
        info.distance_m = std::hypot(x_m, y_m);
        return info;
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

    visualization_msgs::Marker toMarker(const katri_msgs::ObstacleInfo& ob, const int id,
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
            const double d = std::max<double>(ob.width_m, 0.4);
            m.scale.x = d;
            m.scale.y = d;
            m.scale.z = 1.7;
            m.pose.position.z = 0.85;
            m.color.r = 1.0f;
        } else {                                      // 차량/정적: 큐브 (길이×폭)
            m.type = visualization_msgs::Marker::CUBE;
            m.scale.x = std::max<double>(ob.length_m, 0.2);
            m.scale.y = std::max<double>(ob.width_m, 0.2);
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

    // params
    std::string udp_ip_;
    int udp_port_ = 7505;
    std::string ego_topic_;
    std::string frame_id_ = "base_link";
    double publish_rate_hz_ = 30.0;
    double ego_timeout_ = 0.5;
    double velocity_scale_ = 1.0 / 3.6;
    int pedestrian_objtype_ = 0;
    int ego_objtype_ = -1;

    ros::Subscriber ego_sub_;
    ros::Publisher obstacle_pub_;
    ros::Publisher marker_pub_;
    ros::Timer timer_;

    // ego (odom callback ↔ timer)
    std::mutex ego_mtx_;
    double ego_x_ = 0.0, ego_y_ = 0.0, ego_yaw_ = 0.0;
    ros::Time ego_stamp_;
    bool got_ego_ = false;

    // udp (recv thread ↔ timer)
    int sock_fd_ = -1;
    std::atomic<bool> running_{true};
    std::thread recv_thread_;
    std::mutex pkt_mtx_;
    ObjectInfoPacket latest_{};
    bool have_pkt_ = false;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_info_publisher");
    ObstacleInfoPublisher node;
    ros::spin();
    return 0;
}
