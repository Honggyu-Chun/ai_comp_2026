#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# acc_node CTG 로직 통합 테스트.
# 합성 /Ego_topic + /obstacle_info 를 발행하고 /acc/target_speed[km/h] 를 검사.
# 실행: (roscore + acc_node 가 떠 있는 상태에서)
#   rosrun global_path_planner test_acc.py
#
# 기본 파라미터 기준: v_set=60kph, headway=1.5s, min_gap=6m, k=0.4, lane_gate=1.5m, standoff=2.5m

import sys

import rospy
from morai_msgs.msg import EgoVehicleStatus
from katri_msgs.msg import ObstacleInfo, ObstacleInfoArray
from std_msgs.msg import Float64

TOL = 0.05


def make_obstacle(x, y, speed_mps, length):
    o = ObstacleInfo()
    o.x_m, o.y_m = x, y
    o.speed_mps = speed_mps
    o.length_m = length
    o.width_m = 2.0
    return o


class Tester:
    def __init__(self):
        self.ego_pub = rospy.Publisher("/Ego_topic", EgoVehicleStatus, queue_size=1, latch=True)
        self.obs_pub = rospy.Publisher("/obstacle_info", ObstacleInfoArray, queue_size=1, latch=True)
        self.latest = None
        rospy.Subscriber("/acc/target_speed", Float64, self._cb)

    def _cb(self, msg):
        self.latest = msg.data

    def run_case(self, name, ego_vx_mps, obstacles, expected_kph):
        ego = EgoVehicleStatus()
        ego.velocity.x = ego_vx_mps
        self.ego_pub.publish(ego)

        arr = ObstacleInfoArray()
        arr.obstacles = obstacles
        self.obs_pub.publish(arr)

        # acc_node 는 30Hz 로 계속 발행 → 입력 반영될 시간을 준 뒤 최신값 확인
        self.latest = None
        deadline = rospy.Time.now() + rospy.Duration(4.0)
        rate = rospy.Rate(20)
        while self.latest is None and rospy.Time.now() < deadline and not rospy.is_shutdown():
            rate.sleep()
        rospy.sleep(0.4)  # 입력 반영 후 값 안정화

        assert self.latest is not None, f"[{name}] /acc/target_speed 미수신"
        assert abs(self.latest - expected_kph) < TOL, \
            f"[{name}] 기대 {expected_kph:.2f} kph, 실제 {self.latest:.2f} kph"
        rospy.loginfo("[PASS] %s -> %.2f kph", name, self.latest)


def main():
    rospy.init_node("test_acc", anonymous=True)
    t = Tester()
    rospy.sleep(1.0)

    # A: 선행차 없음 → v_set 60
    t.run_case("A_no_lead", 10.0, [], 60.0)

    # B: 선행차 gap=desired(21m) 정확, lead 10m/s → v=10m/s=36kph
    #    desired=6+1.5*10=21; gap=x-2.5-0.5*len; len=5 -> x=26 -> gap=21
    t.run_case("B_gap_matched", 10.0, [make_obstacle(26.0, 0.0, 10.0, 5.0)], 36.0)

    # C: 선행차 근접(gap=3) → v=10+0.4*(3-21)=2.8m/s=10.08kph
    t.run_case("C_close_lead", 10.0, [make_obstacle(8.0, 0.0, 10.0, 5.0)], 2.8 * 3.6)

    # D: 옆 차로(|y|=3>gate) → 무시 → 60
    t.run_case("D_lateral_gate", 10.0, [make_obstacle(20.0, 3.0, 0.0, 5.0)], 60.0)

    rospy.loginfo("ALL ACC TESTS PASSED")


if __name__ == "__main__":
    try:
        main()
    except (AssertionError, rospy.ROSException) as e:
        rospy.logerr("TEST FAILED: %s", e)
        sys.exit(1)
