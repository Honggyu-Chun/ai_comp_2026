#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import math
import struct
import ctypes
import rospy
import tf
from nav_msgs.msg import Odometry
import time

UDP_MODULE_ROOT = "/home/amsl/catkin_ws/src/MORAI_UDP_NetworkModule-24.R2.0"
if UDP_MODULE_ROOT not in sys.path:
    sys.path.append(UDP_MODULE_ROOT)

from lib.network.UDP import Receiver  # noqa: E402
from lib.define.EgoVehicleStatus import EgoVehicleStatus

# IP = '127.0.0.1' 
# PORT = 9011
IP = '192.168.0.11' 
PORT = 1234

class EgoStatusToOdom:

    def __init__(self):
        # ROS
        rospy.init_node("ai_gpsudptoros", anonymous=True)
        self.pub = rospy.Publisher("/gps_utm_odom", Odometry, queue_size=10)
        self.br = tf.TransformBroadcaster()
        self.rate_hz = rospy.get_param("~rate", 30.0)

        # 프레임명
        self.frame_id = "map"
        self.child_frame_id = "base_link"

        # 네트워크
        self.egovehiclestatus = Receiver(IP, PORT, EgoVehicleStatus())

        rospy.loginfo("[ai_gpsudptoros] listening UDP %s:%d -> /gps_utm_odom", IP, PORT)
    

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()

    def step(self):
        status = self.egovehiclestatus.get_data()
        if status is None:
            return
        print(status)
        x = status.pos_x
        y = status.pos_y
        yaw_deg = status.yaw
        yaw_rad = math.radians(yaw_deg)
        velocity = status.vel_x

        q = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw_rad)

        if x is None or y is None:
            rospy.logwarn_throttle(2.0, "[ai_gpsudptoros] pos_x/pos_y 필드를 찾지 못했습니다. fields=%s", dir(status))
            return

        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = float(x)
        odom.pose.pose.position.y = float(y)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x=velocity

        self.pub.publish(odom)

        br = tf.TransformBroadcaster()
        br.sendTransform((x, y, 0),
                     q,
                     rospy.Time.now(),
                     "base_link",
                     "map")

if __name__ == "__main__":
    node = EgoStatusToOdom()
    node.spin()