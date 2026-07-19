#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header
import tf2_ros
import tf2_geometry_msgs

class LatticePlanner:
    def __init__(self):
        self.current_pose = None
        self.target_lane = None
        self.lane_change_initiated = False

        self.pose_sub = rospy.Subscriber('/gps_utm_odom', Odometry, self.pose_callback)
        self.target_lane_sub = rospy.Subscriber('/target_lane', Path, self.target_lane_callback)
        self.lane_change_pub = rospy.Publisher('/lane_change_path', Path, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def target_lane_callback(self, msg):
        self.target_lane = msg.poses
        if self.current_pose:
            self.lane_change_initiated = True
            self.generate_lane_change_path()

    def generate_lane_change_path(self):
        if not self.lane_change_initiated:
            return

        start_pose = self.current_pose
        end_pose = self.target_lane[-1].pose

        # Generate lattice points
        lattice_points = self.create_lattice_points(start_pose, end_pose)

        # Evaluate paths and select the best one
        best_path = self.evaluate_paths(lattice_points)

        # Publish the best path
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = 'map'
        path_msg.poses = best_path

        self.lane_change_pub.publish(path_msg)
        self.lane_change_initiated = False

    def create_lattice_points(self, start_pose, end_pose):
        lattice_points = []
        num_points = 10  # Number of points to generate between start and end

        for i in range(num_points + 1):
            t = i / num_points
            x = (1 - t) * start_pose.position.x + t * end_pose.position.x
            y = (1 - t) * start_pose.position.y + t * end_pose.position.y
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            lattice_points.append(pose)

        return lattice_points

    def evaluate_paths(self, lattice_points):
        # Here we just return the generated points as a single path
        # In a real application, you would evaluate each path based on certain criteria
        path = []
        for point in lattice_points:
            pose_stamped = PoseStamped()
            pose_stamped.header = Header()
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.pose = point
            path.append(pose_stamped)

        return path

if __name__ == '__main__':
    rospy.init_node('lattice_planner')
    planner = LatticePlanner()
    rospy.spin()
