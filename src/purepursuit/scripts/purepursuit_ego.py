#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from math import cos,sin,sqrt,pow,atan2,pi,atan,tan,radians
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry,Path
import numpy as np
import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from visualization_msgs.msg import Marker
from std_msgs.msg import String, Int16, Float64, Bool
from morai_msgs.msg import CtrlCmd, EventInfo, GPSMessage, EgoVehicleStatus
from morai_msgs.srv import MoraiEventCmdSrv

class pure_pursuit :
    def __init__(self, KP = 0.7, KI = 0.0, KD = 0.0):  #p 0.1
        rospy.init_node('purepursuit_curvature', anonymous=True)
        rospy.Subscriber("/gps_utm_odom", Odometry, self.GPSCallBack)
        rospy.Subscriber("/local_path", Path, self.LocalPathCallBack)
        rospy.Subscriber("/state", String, self.StateCallBack)
        # rospy.Subscriber("/parking_state", Int16, self.ParkingStateCallBack)
        # rospy.Subscriber("/parking_end", String, self.ParkingEndCallBack)
        # rospy.Subscriber("/traffic_light_control", String, self.TrafficStateCallBack)
        rospy.Subscriber("/gps", GPSMessage, self.GPSStateCallBack)
        # rospy.Subscriber("/estop", Bool, self.EstopCallBack)
        rospy.Subscriber("/gps_count", Int16, self.GPSCountCallBack)
        # rospy.Subscriber("/speed_limit", Int16, self.SpeedLimitCallBack)
        # rospy.Subscriber("/CollisionData", CollisionData, self.collision_callback)  

        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.EgoVehicle_CallBack)
        rospy.Subscriber("/path_number", Int16, self.path_num_CallBack)
        rospy.Subscriber("/control_sign", String, self.controlCallback)


        rospy.wait_for_service('/Service_MoraiEventCmd')
        self.event_cmd_srv = rospy.ServiceProxy('Service_MoraiEventCmd', MoraiEventCmdSrv)

        self.ctrl_cmd_pub = rospy.Publisher('/ctrl_cmd_0',CtrlCmd, queue_size=1)
        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 1
        self.ctrl_cmd_msg.accel = 0
        self.ctrl_cmd_msg.brake = 0
        self.ctrl_cmd_msg.steering = 0
        self.steering_gain = 0.62
        self.KP = KP
        self.KI = KI
        self.KD = KD
        self.prev_error = 0
        self.integral = 0
        self.LD = 1
        self.target_vel = 5
        self.current_vel = 0
        self.imu_vel = 0
        self.estop = False
        self.gps_status = 1
        self.current_position = Point()
        self.localpath = Path() 
        self.forward_point = Point()
        self.vehicle_yaw = 0.0
        self.vehicle_length = 1.65
        self.vel_penalty = 0.0
        self.traffic_state = String()
        self.gps_count = 0
        self.state = String()
        self.is_look_forward_point=False
        # self.localpath = Path()
        self.gps_state = False
        self.path_state = False
        self.frequency = 20
        self.dt = 1/self.frequency
        self.parking_end = String()
        self.parking_state = Int16()
        self.speed_limit = 0
        self.highway_velocity = 60

        self.path_num = 1

        self.collision_detected = False  
        self.collision_start_time = None  
    
    def controlCallback(self, msg):
        self.control_sign = msg.data
        
    def GPSCallBack(self, msg):
        self.gps_state = True
        q= (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(q)
        self.vehicle_yaw = yaw
        self.current_position.x = msg.pose.pose.position.x  
        self.current_position.y = msg.pose.pose.position.y  
        self.current_position.z = msg.pose.pose.position.z  
        self.vehicle_position = self.current_position

    def GPSStateCallBack(self, msg):
        self.gps_status = msg.status

    def GPSCountCallBack(self, msg):
        self.gps_count = msg.data

    def EstopCallBack(self, msg):
        self.estop = msg.data
        
    def path_num_CallBack(self, msg):
        self.path_num = msg.data

    def LocalPathCallBack(self, msg):
        # print('aaa')
        self.path_state = True
        self.localpath = msg
        if len(self.localpath.poses) > 1:
            dy = self.localpath.poses[1].pose.position.y - self.localpath.poses[0].pose.position.y
            dx = self.localpath.poses[1].pose.position.x - self.localpath.poses[0].pose.position.x
            self.localpath_yaw = np.arctan2(dy,dx)
            self.local_error_front_axle = self.localpath.poses[0].pose.position.y
        else:
            rospy.logwarn("Local path poses are insufficient.")
        
    def StateCallBack(self, msg):
        self.state = msg.data

    def EgoVehicle_CallBack(self, msg):
        self.vehicle_info = True
        self.current_vel = abs(msg.velocity.x * 3600 / 1000)

    def Cruise_Control(self, target_vel):
        KP = self.KP
        KI = self.KI
        KD = self.KD   
       
        error = target_vel - self.current_vel
        
        self.integral += error
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error
        P = KP * error
        I = KI * self.integral
        D = KD * derivative
        accelation = P + I + D
        # print(accelation)
        if accelation > 0:
            self.ctrl_cmd_msg.accel = min(accelation, 1)  # Ensure that accel is in the range [0, 1]
            self.ctrl_cmd_msg.brake = 0
        else:
            self.ctrl_cmd_msg.accel = 0
            self.ctrl_cmd_msg.brake = min(-accelation * 0.08, 1)  # Ensure that brake is in the range [0, 1]
        if target_vel == 0:
            self.ctrl_cmd_msg.accel = 0
            self.ctrl_cmd_msg.brake = 1

    def send_gear_cmd(self, gear_mode):
        gear_cmd = EventInfo()
        gear_cmd.option = 2
        gear_cmd.ctrl_mode = 3
        gear_cmd.gear = gear_mode  # 1:P 2:R 3:N 4:D
        gear_cmd_resp = self.event_cmd_srv(gear_cmd)

    def collision_callback(self, collision_msg): 
        if collision_msg.collision_object:
            if not self.collision_detected:
                self.collision_start_time = rospy.get_time()  
            self.collision_detected = True
        else:
            self.collision_detected = False
            self.collision_start_time = None  
     
    
P = pure_pursuit()

class marker():
    
    def __init__(self):
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=1)
        self.marker = Marker()
        self.marker.header.frame_id = 'base_link'
        self.marker.type = Marker.LINE_STRIP
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.1
        self.marker.color.a = 100.0
        self.marker.color.g = 1.0
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        self.num_points = 36

    def marker_publish(self):
        self.radius = P.LD
        self.marker.action = Marker.DELETE
        self.marker.points = []
        self.marker_pub.publish(self.marker)
        self.marker.action = Marker.ADD
        for i in range(self.num_points+1):
            point = Point()
            point.x = self.radius * np.cos(2*np.pi*i/self.num_points)
            point.y = self.radius * np.sin(2*np.pi*i/self.num_points)
            point.z = 0.0
            self.marker.points.append(point)
        self.marker_pub.publish(self.marker)
        
M = marker()
        
def main():
    target_vel = 0
    curvature = 0
    steer = 0.0

    rate = rospy.Rate(P.frequency) # 50hz
    while not rospy.is_shutdown():
        if P.gps_state and P.path_state:
            P.is_look_forward_point= False
            

            path_x, path_y = [], []
            for i in range(len(P.localpath.poses)):
                x = P.localpath.poses[i].pose.position.x
                y = P.localpath.poses[i].pose.position.y
                path_x.append(x)
                path_y.append(y)
                dis = np.sqrt(x ** 2 + y ** 2)
                if P.LD <= dis < P.LD + 20:
                    P.is_look_forward_point = True
                    gradiant = np.arctan2(np.diff(path_y), np.diff(path_x))
                    curvature = np.sum(abs(np.diff(gradiant)))
                    break

            # if P.is_look_forward_point:
            alpha = np.arctan2(y, x)
            steer = np.arctan2((2 * P.vehicle_length * np.sin(alpha)), P.LD)

            # print(curvature)
            penalty = curvature * 5
            P.LD = 5
            P.steering_gain = 0.7
            if P.path_num == 1: 
                print('path num: 1')
                P.LD = 3
                P.send_gear_cmd(2)
                P.steering_gain = -0.5
                target_vel = 5
            
            elif P.control_sign == "stop":
                target_vel = 0
                P.steering_gain = 0
            
            elif P.control_sign == "parking":
                target_vel = 0
                P.steering_gain = 0
                P.send_gear_cmd(4)
                    
            elif P.path_num == 2:
                print('path num: 2')
                P.LD = 1
                P.send_gear_cmd(4)
                P.steering_gain = 0.7
                target_vel = 5

            else:
                P.LD = 2
                target_vel = 3
                P.steering_gain = 0.4

            P.ctrl_cmd_msg.steering = steer * P.steering_gain
            
            P.Cruise_Control(target_vel)
            
        else:
            print("GPS : ", P.gps_state,"  |  PATH : ", P.path_state)
            P.ctrl_cmd_msg.accel = 0
            P.ctrl_cmd_msg.brake = 1
            P.ctrl_cmd_msg.steering = 0
            
        P.ctrl_cmd_pub.publish(P.ctrl_cmd_msg)
        print("Steer [deg]: {:.1f} | Target Vel : {:.1f} | Current Vel : {:.1f} | Accel : {:.1f} | Brake : {:.1f} | LD [m]: {:.1f}".format(P.ctrl_cmd_msg.steering * 180 / np.pi, target_vel,  P.current_vel, P.ctrl_cmd_msg.accel, P.ctrl_cmd_msg.brake, P.LD))
        M.marker_publish()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass   
