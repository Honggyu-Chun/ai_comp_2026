#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from math import sqrt, pow
import numpy as np
import matplotlib.pyplot as plt
import os
from scipy.interpolate import InterpolatedUnivariateSpline


class make_path:

    def __init__(self):
        filename = 'global_path_2.txt'
        self.sample_distance = 0.1  
        self.smoothing_factor = 0.1

        rospy.init_node('path_maker', anonymous=False)
        filename = rospy.get_param("~file_name", filename)
        if not filename.endswith(".txt"):
            filename = filename + ".txt"
        rospy.Subscriber("gps_utm_odom", Odometry, self.odom_callback)
        self.path_pub = rospy.Publisher('/local_path', Path, queue_size=1)
        self.is_odom = False
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'
        self.prev_x = 0
        self.prev_y = 0
        self.positions = []
        self.line_width = 1
        self.file_closed = False
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('global_path_planner')
        directory = pkg_path + "/path_data/"
        if not os.path.exists(directory):
            os.makedirs(directory)
        if os.path.exists(os.path.join(directory, filename)):
            i = 1
            while os.path.exists(os.path.join(directory, f"{os.path.splitext(filename)[0]}_{i}.txt")):
                i += 1
            filename = f"{os.path.splitext(filename)[0]}_{i}.txt"
        self.full_path = os.path.join(directory, filename)
        self.f = open(self.full_path, 'w')
        rospy.on_shutdown(self.plot_waypoints)
        rospy.spin()
        self.f.close()
        self.file_closed = True

    def odom_callback(self, msg):
        if self.file_closed:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.positions.append((x, y))

        if self.is_odom:
            distance = sqrt(pow(x - self.prev_x, 2) + pow(y - self.prev_y, 2))
            if distance > 0.1:
                waypint_pose = PoseStamped()
                waypint_pose.pose.position.x = x
                waypint_pose.pose.position.y = y
                waypint_pose.pose.orientation.w = 1
                self.path_msg.poses.append(waypint_pose)
                self.path_pub.publish(self.path_msg)
                data = '{0}\t{1}\n'.format(x, y)
                self.f.write(data)
                self.prev_x = x
                self.prev_y = y
                print(x, y)
        else:
            self.is_odom = True
            self.prev_x = x
            self.prev_y = y

    def plot_waypoints(self):
        if not self.file_closed:
            self.f.close()
            self.file_closed = True
        
        x_coords = []
        y_coords = []

        with open(self.full_path, 'r') as file:
            for line in file:
                x, y = map(float, line.strip().split())
                x_coords.append(x)
                y_coords.append(y)

        
        distances = np.sqrt(np.diff(x_coords) ** 2 + np.diff(y_coords) ** 2)
        cumulative_distances = np.insert(np.cumsum(distances), 0, 0)
        spl_x = InterpolatedUnivariateSpline(cumulative_distances, x_coords)
        spl_y = InterpolatedUnivariateSpline(cumulative_distances, y_coords)
        spl_x.set_smoothing_factor(self.smoothing_factor)
        spl_y.set_smoothing_factor(self.smoothing_factor)
        total_distance = cumulative_distances[-1]
        new_distances = np.arange(0, total_distance, self.sample_distance)
        smoothed_x = spl_x(new_distances)
        smoothed_y = spl_y(new_distances)
        


        plt.figure()
        plt.plot(x_coords, y_coords, '-k', lw = self.line_width, label='raw')
        plt.plot(x_coords, y_coords, '.k', label='raw')
        plt.plot(smoothed_x, smoothed_y, '-b', lw=self.line_width, label='smoothed')
        plt.plot(smoothed_x, smoothed_y, '.b', label='smoothed')
        plt.axis('equal')
        plt.grid(True)
        plt.legend()
        plt.title('Waypoints Path')
        plt.xlabel('X Coordinates')
        plt.ylabel('Y Coordinates')
        plt.show()
        plt.close()

        sampled_path = self.full_path.replace('.txt', '_sm.txt')
        with open(sampled_path, 'w') as f:
            for x, y in zip(smoothed_x, smoothed_y):
                f.write(f'{x}\t{y}\n')
       


if __name__ == '__main__':
    try:
        test_track = make_path()
    except rospy.ROSInterruptException:
        pass
