#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import ctypes
import socket
from threading import Lock

import rospy
from morai_msgs.msg import CtrlCmd


class PackedStruct(ctypes.LittleEndianStructure):
    _pack_ = 1


class UdpEgoCtrlCmd(PackedStruct):
    _fields_ = [
        ("header", ctypes.c_char * 14),
        ("data_lenght", ctypes.c_int32),
        ("aux_data", ctypes.c_int32 * 3),
        ("ctrl_mode", ctypes.c_int8),
        ("gear", ctypes.c_int8),
        ("cmd_type", ctypes.c_int8),
        ("velocity", ctypes.c_float),
        ("acceleration", ctypes.c_float),
        ("accel", ctypes.c_float),
        ("brake", ctypes.c_float),
        ("steer", ctypes.c_float),
        ("tail", ctypes.c_char * 2),
    ]


def _get_field(msg, field_name, default):
    return getattr(msg, field_name, default)


def _bool_param(param_name, default):
    value = rospy.get_param(param_name, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    return str(value).strip().lower() in ("1", "true", "yes", "on")


class CompetitionControlUdpBridge:
    def __init__(self):
        rospy.init_node("competition_control_udp_bridge", anonymous=False)

        self.cmd_topic = rospy.get_param("~cmd_topic", "/ctrl_cmd")
        self.target_ip = rospy.get_param("~target_ip", "127.0.0.1")
        self.target_port = int(rospy.get_param("~target_port", 9093))
        self.hz = float(rospy.get_param("~hz", 30.0))
        self.ctrl_mode = int(rospy.get_param("~ctrl_mode", 2))
        self.gear = int(rospy.get_param("~gear", 4))
        self.default_cmd_type = int(rospy.get_param("~default_cmd_type", 2))
        self.send_without_cmd = _bool_param("~send_without_cmd", False)

        self.lock = Lock()
        self.latest_cmd = CtrlCmd()
        self.has_cmd = False

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rospy.on_shutdown(self.close)

        rospy.Subscriber(self.cmd_topic, CtrlCmd, self._cmd_callback, queue_size=10)
        rospy.Timer(rospy.Duration(1.0 / max(self.hz, 1.0)), self._timer_callback)

        rospy.loginfo(
            "[competition_control_udp_bridge] %s -> %s:%d | ctrl_mode=%d gear=%d hz=%.1f",
            self.cmd_topic,
            self.target_ip,
            self.target_port,
            self.ctrl_mode,
            self.gear,
            self.hz,
        )

    def close(self):
        try:
            self.socket.close()
        except OSError:
            pass

    def _cmd_callback(self, msg):
        with self.lock:
            self.latest_cmd = msg
            self.has_cmd = True

    def _cmd_type(self, value):
        try:
            cmd_type = int(value)
        except Exception:
            cmd_type = self.default_cmd_type
        if cmd_type not in (1, 2, 3):
            return self.default_cmd_type
        return cmd_type

    def _build_packet(self, msg):
        packet = UdpEgoCtrlCmd()
        packet.header = b"#MoraiCtrlCmd$"
        packet.data_lenght = 23
        packet.aux_data = (ctypes.c_int32 * 3)(0, 0, 0)
        packet.ctrl_mode = self.ctrl_mode
        packet.gear = self.gear
        packet.cmd_type = self._cmd_type(_get_field(msg, "longlCmdType", self.default_cmd_type))
        packet.velocity = float(_get_field(msg, "velocity", 0.0))
        packet.acceleration = float(_get_field(msg, "acceleration", 0.0))
        packet.accel = float(_get_field(msg, "accel", 0.0))
        packet.brake = float(_get_field(msg, "brake", 0.0))
        packet.steer = float(_get_field(msg, "steering", 0.0))
        packet.tail = b"\r\n"
        return packet

    def _send_packet(self, packet):
        payload = ctypes.string_at(ctypes.addressof(packet), ctypes.sizeof(packet))
        self.socket.sendto(payload, (self.target_ip, self.target_port))

    def _timer_callback(self, _event):
        with self.lock:
            has_cmd = self.has_cmd
            msg = self.latest_cmd

        if not has_cmd and not self.send_without_cmd:
            return

        try:
            self._send_packet(self._build_packet(msg))
        except OSError as exc:
            rospy.logwarn_throttle(2.0, "[competition_control_udp_bridge] UDP send failed: %s", exc)


def main():
    CompetitionControlUdpBridge()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
