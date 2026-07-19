#!/usr/bin/env python3

import ctypes
import math
import socket

import rospy
from sensor_msgs.msg import Imu


class MoraiImuPacket(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("header", ctypes.c_char * 9),
        ("data_lenght", ctypes.c_int),
        ("aux_data", ctypes.c_int * 3),
        ("sec", ctypes.c_int),
        ("nsec", ctypes.c_int),
        ("ori_w", ctypes.c_double),
        ("ori_x", ctypes.c_double),
        ("ori_y", ctypes.c_double),
        ("ori_z", ctypes.c_double),
        ("ang_vel_x", ctypes.c_double),
        ("ang_vel_y", ctypes.c_double),
        ("ang_vel_z", ctypes.c_double),
        ("lin_acc_x", ctypes.c_double),
        ("lin_acc_y", ctypes.c_double),
        ("lin_acc_z", ctypes.c_double),
        ("tail", ctypes.c_char * 2),
    ]


class MoraiImuUdpNode:
    def __init__(self):
        self.bind_ip = rospy.get_param("~bind_ip", "0.0.0.0")
        self.port = int(rospy.get_param("~port", 2224))
        self.topic = rospy.get_param("~topic", "/imu/data")
        self.frame_id = rospy.get_param("~frame_id", "imu_link")
        self.use_packet_time = rospy.get_param("~use_packet_time", True)
        self.debug_raw = rospy.get_param("~debug_raw", False)
        self.gyro_lpf_tau_s = float(rospy.get_param("~gyro_lpf_tau_s", 0.08))
        self.accel_lpf_tau_s = float(rospy.get_param("~accel_lpf_tau_s", 0.12))
        self.max_gyro_rps = float(rospy.get_param("~max_gyro_rps", 8.0))
        self.max_accel_mps2 = float(rospy.get_param("~max_accel_mps2", 30.0))

        self.last_stamp = None
        self.gyro_filt = None
        self.accel_filt = None

        self.packet = MoraiImuPacket()
        self.packet_size = ctypes.sizeof(self.packet)

        self.pub = rospy.Publisher(self.topic, Imu, queue_size=50)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2 ** 20)
        self.sock.bind((self.bind_ip, self.port))
        self.sock.settimeout(0.2)

        rospy.loginfo(
            "[morai_imu_udp] listening %s:%d, packet_size=%d -> %s",
            self.bind_ip,
            self.port,
            self.packet_size,
            self.topic,
        )
        rospy.loginfo(
            "[morai_imu_udp] filter gyro_tau=%.3f accel_tau=%.3f max_gyro=%.2f max_accel=%.2f",
            self.gyro_lpf_tau_s,
            self.accel_lpf_tau_s,
            self.max_gyro_rps,
            self.max_accel_mps2,
        )

    def _stamp(self):
        if self.use_packet_time and self.packet.sec > 0 and self.packet.nsec >= 0:
            try:
                return rospy.Time(self.packet.sec, self.packet.nsec)
            except Exception:
                pass
        return rospy.Time.now()

    @staticmethod
    def _clamp_vec(vec, max_norm):
        if max_norm <= 0.0:
            return vec
        norm = math.sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2])
        if norm <= max_norm or norm <= 1e-9:
            return vec
        scale = max_norm / norm
        return (vec[0] * scale, vec[1] * scale, vec[2] * scale)

    @staticmethod
    def _lpf(prev, current, dt, tau):
        if prev is None or tau <= 0.0 or dt <= 0.0:
            return current
        alpha = min(1.0, dt / (tau + dt))
        return (
            prev[0] + (current[0] - prev[0]) * alpha,
            prev[1] + (current[1] - prev[1]) * alpha,
            prev[2] + (current[2] - prev[2]) * alpha,
        )

    def _stable_vectors(self, stamp):
        gyro = (
            self.packet.ang_vel_x,
            self.packet.ang_vel_y,
            self.packet.ang_vel_z,
        )
        accel = (
            self.packet.lin_acc_x,
            self.packet.lin_acc_y,
            self.packet.lin_acc_z,
        )

        gyro = self._clamp_vec(gyro, self.max_gyro_rps)
        accel = self._clamp_vec(accel, self.max_accel_mps2)

        dt = 0.0
        if self.last_stamp is not None:
            dt = (stamp - self.last_stamp).to_sec()
        if dt < 0.0 or dt > 0.5:
            dt = 0.0

        self.gyro_filt = self._lpf(self.gyro_filt, gyro, dt, self.gyro_lpf_tau_s)
        self.accel_filt = self._lpf(self.accel_filt, accel, dt, self.accel_lpf_tau_s)
        self.last_stamp = stamp
        return self.gyro_filt, self.accel_filt

    def _build_msg(self):
        msg = Imu()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = self.frame_id

        q_norm = math.sqrt(
            self.packet.ori_w * self.packet.ori_w
            + self.packet.ori_x * self.packet.ori_x
            + self.packet.ori_y * self.packet.ori_y
            + self.packet.ori_z * self.packet.ori_z
        )
        if q_norm > 1e-6:
            msg.orientation.w = self.packet.ori_w / q_norm
            msg.orientation.x = self.packet.ori_x / q_norm
            msg.orientation.y = self.packet.ori_y / q_norm
            msg.orientation.z = self.packet.ori_z / q_norm
        else:
            msg.orientation.w = 1.0
            msg.orientation_covariance[0] = -1.0

        gyro, accel = self._stable_vectors(msg.header.stamp)

        msg.angular_velocity.x = gyro[0]
        msg.angular_velocity.y = gyro[1]
        msg.angular_velocity.z = gyro[2]

        msg.linear_acceleration.x = accel[0]
        msg.linear_acceleration.y = accel[1]
        msg.linear_acceleration.z = accel[2]

        # MORAI packet does not provide covariance; use conservative defaults.
        if q_norm > 1e-6:
            msg.orientation_covariance[0] = 0.02
            msg.orientation_covariance[4] = 0.02
            msg.orientation_covariance[8] = 0.04
        msg.angular_velocity_covariance[0] = 0.02
        msg.angular_velocity_covariance[4] = 0.02
        msg.angular_velocity_covariance[8] = 0.02
        msg.linear_acceleration_covariance[0] = 0.8
        msg.linear_acceleration_covariance[4] = 0.8
        msg.linear_acceleration_covariance[8] = 1.2
        return msg

    def spin(self):
        while not rospy.is_shutdown():
            try:
                raw_data, addr = self.sock.recvfrom(max(2048, self.packet_size))
            except socket.timeout:
                continue

            if len(raw_data) < self.packet_size:
                rospy.logwarn_throttle(
                    1.0,
                    "[morai_imu_udp] short packet from %s:%d len=%d expected=%d",
                    addr[0],
                    addr[1],
                    len(raw_data),
                    self.packet_size,
                )
                continue

            ctypes.memmove(
                ctypes.addressof(self.packet),
                raw_data[: self.packet_size],
                self.packet_size,
            )

            if self.debug_raw:
                rospy.loginfo_throttle(
                    1.0,
                    "[morai_imu_udp] header=%r t=%d.%09d q=(%.4f %.4f %.4f %.4f) "
                    "gyro=(%.4f %.4f %.4f) accel=(%.4f %.4f %.4f)",
                    bytes(self.packet.header),
                    self.packet.sec,
                    self.packet.nsec,
                    self.packet.ori_x,
                    self.packet.ori_y,
                    self.packet.ori_z,
                    self.packet.ori_w,
                    self.packet.ang_vel_x,
                    self.packet.ang_vel_y,
                    self.packet.ang_vel_z,
                    self.packet.lin_acc_x,
                    self.packet.lin_acc_y,
                    self.packet.lin_acc_z,
                )

            self.pub.publish(self._build_msg())


if __name__ == "__main__":
    rospy.init_node("morai_imu_udp")
    node = MoraiImuUdpNode()
    node.spin()
