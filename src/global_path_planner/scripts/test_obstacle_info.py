#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# obstacle_info_publisher 통합 테스트.
# 합성 /Ego_topic + /Object_topic 을 발행하고 /obstacle_info 를 받아 상대좌표를 assert.
# 실행: (roscore + obstacle_info_publisher 노드가 떠 있는 상태에서)
#   rosrun global_path_planner test_obstacle_info.py
#
# 주의: 노드는 object 콜백에서만 발행하므로, ego 를 먼저 세팅한 뒤
#       object 를 결과가 올 때까지 반복 발행한다(콜백 순서/연결 레이스 방지).

import math
import sys

import rospy
from morai_msgs.msg import EgoVehicleStatus, ObjectStatus, ObjectStatusList
from katri_msgs.msg import ObstacleInfoArray
from visualization_msgs.msg import Marker, MarkerArray

TOL = 1e-3


def make_object(uid, otype, x, y, heading_deg, vx_kph, w, l):
    o = ObjectStatus()
    o.unique_id = uid
    o.type = otype
    o.heading = heading_deg
    o.position.x, o.position.y, o.position.z = x, y, 0.0
    o.velocity.x, o.velocity.y, o.velocity.z = vx_kph, 0.0, 0.0
    o.size.x, o.size.y, o.size.z = l, w, 1.0  # MORAI size = (length, width, height)
    return o


class Tester:
    def __init__(self):
        self.ego_pub = rospy.Publisher("/Ego_topic", EgoVehicleStatus, queue_size=1, latch=True)
        self.obj_pub = rospy.Publisher("/Object_topic", ObjectStatusList, queue_size=1)
        self.latest = None
        self.markers = None
        rospy.Subscriber("/obstacle_info", ObstacleInfoArray, self._cb)
        rospy.Subscriber("/obstacle_markers", MarkerArray, self._mcb)

    def _cb(self, msg):
        self.latest = msg

    def _mcb(self, msg):
        self.markers = msg

    def run_case(self, name, ego_heading_deg, obj, checks, marker_shape=None):
        # ego 세팅(latched)
        ego = EgoVehicleStatus()
        ego.position.x, ego.position.y = 100.0, 200.0
        ego.heading = ego_heading_deg
        self.ego_pub.publish(ego)
        rospy.sleep(0.5)  # 노드가 ego 를 확실히 받도록

        lst = ObjectStatusList()
        if obj.type == 0:
            lst.pedestrian_list = [obj]
        else:
            lst.obstacle_list = [obj]

        # 결과가 올 때까지 object 를 반복 발행
        self.latest = None
        rate = rospy.Rate(20)
        deadline = rospy.Time.now() + rospy.Duration(5.0)
        while self.latest is None and rospy.Time.now() < deadline and not rospy.is_shutdown():
            self.obj_pub.publish(lst)
            rate.sleep()

        assert self.latest is not None, f"[{name}] /obstacle_info 미수신(timeout)"
        assert len(self.latest.obstacles) == 1, \
            f"[{name}] 기대 1개, 실제 {len(self.latest.obstacles)}"
        ob = self.latest.obstacles[0]
        for field, expected in checks.items():
            actual = getattr(ob, field)
            assert abs(actual - expected) < TOL, \
                f"[{name}] {field}: 기대 {expected}, 실제 {actual}"

        if marker_shape is not None:
            assert self.markers is not None, f"[{name}] /obstacle_markers 미수신"
            adds = [m for m in self.markers.markers if m.action == Marker.ADD]
            assert len(adds) == 1, f"[{name}] 마커 1개 기대, 실제 {len(adds)}"
            assert adds[0].type == marker_shape, \
                f"[{name}] 마커 shape: 기대 {marker_shape}, 실제 {adds[0].type}"
        rospy.loginfo("[PASS] %s", name)


def main():
    rospy.init_node("test_obstacle_info", anonymous=True)
    t = Tester()
    rospy.sleep(1.0)  # 퍼블리셔/서브스크라이버 연결 대기

    # A: heading 0° — 전방이 +x(East). 객체는 East 로 10m 앞.
    t.run_case("A_heading0",
               0.0,
               make_object(1, 2, 110.0, 200.0, 0.0, 0.0, 2.0, 3.0),
               {"x_m": 10.0, "y_m": 0.0, "width_m": 2.0, "length_m": 3.0, "distance_m": 10.0},
               marker_shape=Marker.CUBE)

    # B: heading 90° — 전방이 +y(North). 객체는 North 로 10m 앞 → 전방 10, 좌우 0.
    t.run_case("B_heading90",
               90.0,
               make_object(2, 1, 100.0, 210.0, 90.0, 0.0, 2.0, 5.0),
               {"x_m": 10.0, "y_m": 0.0, "yaw_rad": 0.0},
               marker_shape=Marker.CUBE)

    # C: 보행자, 10km/h → 2.777... m/s, is_pedestrian True.
    t.run_case("C_pedestrian",
               0.0,
               make_object(3, 0, 105.0, 200.0, 0.0, 10.0, 0.5, 0.5),
               {"x_m": 5.0, "y_m": 0.0, "speed_mps": 10.0 / 3.6, "is_pedestrian": True},
               marker_shape=Marker.CYLINDER)

    rospy.loginfo("ALL TESTS PASSED")


if __name__ == "__main__":
    try:
        main()
    except (AssertionError, rospy.ROSException) as e:
        rospy.logerr("TEST FAILED: %s", e)
        sys.exit(1)
