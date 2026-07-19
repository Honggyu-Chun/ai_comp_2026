// ADAS ACC (Adaptive Cruise Control) — 상시 종방향 판단.
// /obstacle_info(자차 base_link 상대장애물) + /Ego_topic(자차 속도) 를 받아
// 자차 차로 앞 선행차와 시간간격(headway)을 유지하는 목표속도를 /acc/target_speed[km/h] 로 발행한다.
// 선행차가 없으면 자유주행 상한(v_set)을 낸다 → 무장애물 시 60kph 상시 지령.
// 제어기는 이 값을 velocity 상한으로 min-캡한다(설계 "속도 = min 중재").
// 출력은 목표속도뿐 — accel/brake 변환·조향은 제어기(purepursuit) 담당.
//
// CTG(정속 시간간격) 선형 ACC:
//   v_target = lead_speed + k*(gap - desired_gap),  desired_gap = d0 + tau*v_ego
//
// ponytail: 차로 판정을 base_link y-게이트로 근사(직선도로 가정). 커브에선 옆 차로 오인/앞차 놓침.
//           정밀화하려면 ReferencePath(Frenet 투영) 필요 → 경로 어댑터 붙으면 교체.

#include <ros/ros.h>

#include <morai_msgs/EgoVehicleStatus.h>
#include <katri_msgs/ObstacleInfoArray.h>
#include <std_msgs/Float64.h>

#include <algorithm>
#include <cmath>
#include <limits>

struct AccParams
{
    double v_set_mps;    // 자유주행 상한(선행차 없을 때)
    double headway_s;    // 시간간격 tau
    double min_gap_m;    // 정지 최소간격 d0
    double gain_k;       // 간격오차 비례게인
    double lane_gate_m;  // |y_m| 이보다 크면 다른 차로로 보고 무시
    double standoff_m;   // base_link 원점→자차 앞범퍼 오프셋(선행차 length/2 는 별도 차감)
};

// 순수함수: ROS 없이 단위 테스트 가능. gap 은 이미 계산된 범퍼간 종거리[m]. 반환 [m/s].
double accTargetSpeed(const bool has_lead, const double lead_speed_mps,
                      const double gap_m, const double v_ego_mps, const AccParams& p)
{
    if (!has_lead) {
        return p.v_set_mps;
    }
    const double desired_gap = p.min_gap_m + p.headway_s * v_ego_mps;
    const double v = lead_speed_mps + p.gain_k * (gap_m - desired_gap);
    return std::max(0.0, std::min(v, p.v_set_mps));
}

class AccNode
{
public:
    AccNode() : v_ego_mps_(0.0), got_ego_(false), got_obs_(false)
    {
        ros::NodeHandle pnh("~");
        double v_set_kph = 60.0;
        pnh.param("v_set_kph", v_set_kph, v_set_kph);
        pnh.param("headway_s", p_.headway_s, 1.5);
        pnh.param("min_gap_m", p_.min_gap_m, 6.0);
        pnh.param("gain_k", p_.gain_k, 0.4);
        pnh.param("lane_gate_m", p_.lane_gate_m, 1.5);
        pnh.param("standoff_m", p_.standoff_m, 2.5);
        pnh.param("rate_hz", rate_hz_, 30.0);
        p_.v_set_mps = v_set_kph / 3.6;

        ros::NodeHandle nh;
        ego_sub_ = nh.subscribe("/Ego_topic", 1, &AccNode::egoCallback, this);
        obs_sub_ = nh.subscribe("/obstacle_info", 1, &AccNode::obstacleCallback, this);
        speed_pub_ = nh.advertise<std_msgs::Float64>("/acc/target_speed", 1);
    }

    void spin()
    {
        ros::Rate rate(std::max(1.0, rate_hz_));
        while (ros::ok()) {
            ros::spinOnce();
            if (got_ego_) {
                publishTarget();
            } else {
                ROS_WARN_THROTTLE(2.0, "[acc] waiting for /Ego_topic");
            }
            rate.sleep();
        }
    }

private:
    void egoCallback(const morai_msgs::EgoVehicleStatus::ConstPtr& msg)
    {
        // EgoVehicleStatus.velocity 는 [m/s] (객체 velocity 는 km/h — 단위 다름 주의).
        v_ego_mps_ = std::hypot(msg->velocity.x, msg->velocity.y);
        got_ego_ = true;
    }

    void obstacleCallback(const katri_msgs::ObstacleInfoArray::ConstPtr& msg)
    {
        obstacles_ = *msg;
        got_obs_ = true;
    }

    // 자차 차로 최근접 선행차: x_m>0 이고 |y_m|<lane_gate 중 x_m 최소.
    bool findLead(katri_msgs::ObstacleInfo& lead) const
    {
        bool found = false;
        double best_x = std::numeric_limits<double>::infinity();
        for (const auto& ob : obstacles_.obstacles) {
            if (ob.x_m <= 0.0 || std::fabs(ob.y_m) > p_.lane_gate_m) {
                continue;
            }
            if (ob.x_m < best_x) {
                best_x = ob.x_m;
                lead = ob;
                found = true;
            }
        }
        return found;
    }

    void publishTarget()
    {
        katri_msgs::ObstacleInfo lead;
        const bool has_lead = got_obs_ && findLead(lead);

        double gap = 0.0;
        double lead_speed = 0.0;
        if (has_lead) {
            gap = lead.x_m - p_.standoff_m - 0.5 * lead.length_m;  // 범퍼간 종거리
            lead_speed = lead.speed_mps;
        }

        std_msgs::Float64 out;
        out.data = accTargetSpeed(has_lead, lead_speed, gap, v_ego_mps_, p_) * 3.6;  // m/s → km/h
        speed_pub_.publish(out);

        ROS_INFO_THROTTLE(1.0, "[acc] lead=%d gap=%.1f v_lead=%.1f v_ego=%.1f -> v_target=%.1f kph",
                          has_lead ? 1 : 0, gap, lead_speed, v_ego_mps_, out.data);
    }

    ros::Subscriber ego_sub_;
    ros::Subscriber obs_sub_;
    ros::Publisher speed_pub_;

    AccParams p_;
    double rate_hz_;
    double v_ego_mps_;
    bool got_ego_;
    bool got_obs_;
    katri_msgs::ObstacleInfoArray obstacles_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "acc_node");
    AccNode node;
    node.spin();
    return 0;
}
