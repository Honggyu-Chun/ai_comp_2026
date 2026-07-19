#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import rospkg
import sys

class PATH:

    def __init__(self):
        self.arg = sys.argv[1]
        #self.arg = rospy.get_param('~arg', '25hl_global_path')
        self.global_path_pub = rospy.Publisher('/global_path1', Path, queue_size=1)
        self.global_path = Path()
        self.global_path.header.frame_id = 'map'
        self.load_path()
        self.publish_path()

    def load_path(self):
        rospack = rospkg.RosPack()
        pkgpath = rospack.get_path('global_path_planner')
        with open(pkgpath + "/path_data/" + str(self.arg) + ".txt", 'r') as file:
            for line in file:
                line = line.strip()
                if line:
                    field = line.split()
                    x = float(field[0])
                    y = float(field[1])
                    temp_pose = PoseStamped()
                    temp_pose.header.frame_id = 'map'
                    temp_pose.pose.position.x = x
                    temp_pose.pose.position.y = y
                    self.global_path.poses.append(temp_pose)
                    print("x: ", x, " y: ", y)
        
        self.global_path.header.stamp = rospy.Time.now()

    def publish_path(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.global_path.header.stamp = rospy.Time.now()
            self.global_path_pub.publish(self.global_path)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('globalpath_publisher_1', anonymous=True)
    path = PATH()
    rospy.spin()
