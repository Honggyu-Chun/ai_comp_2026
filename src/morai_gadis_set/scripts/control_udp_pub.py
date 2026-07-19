#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import ctypes
import os
import socket
import sys
from pathlib import Path
from threading import Lock

import rospy
from morai_msgs.msg import CtrlCmd


def _ensure_udp_module_path(udp_module_root):
    root = Path(udp_module_root).expanduser().resolve()
    lib_dir = root / "lib"
    if not lib_dir.exists():
        raise RuntimeError('Invalid udp_module_root: "{}" (missing lib/)'.format(root))
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    return root


def _resolve_udp_module_root(param_root):
    candidates = []
    if param_root:
        candidates.append(Path(param_root))

    env_root = os.getenv("MORAI_UDP_MODULE_ROOT", "")
    if env_root:
        candidates.append(Path(env_root))

    this_file = Path(__file__).resolve()
    candidates.append(this_file.parents[2] / "MORAI_UDP_NetworkModule-24.R2.0")

    for cand in candidates:
        cand = cand.expanduser().resolve()
        if (cand / "lib").exists():
            return str(cand)

    raise RuntimeError(
        "Cannot resolve UDP module root. "
        "Set ~udp_module_root or MORAI_UDP_MODULE_ROOT. "
        "Tried: {}".format(", ".join(str(p) for p in candidates))
    )


class ControlUdpPublisher:
    def __init__(self):
        udp_module_root = rospy.get_param("~udp_module_root", "")
        self.ctrl_ip = rospy.get_param("~ctrl_ip", "127.0.0.1")
        self.ctrl_port = int(rospy.get_param("~ctrl_port", 9093))
        self.cmd_topic = rospy.get_param("~cmd_topic", "/ctrl_cmd")
        self.hz = float(rospy.get_param("~hz", 30.0))

        self.ctrl_mode = int(rospy.get_param("~ctrl_mode", 2))
        self.gear = int(rospy.get_param("~gear", 4))
        self.default_cmd_type = int(rospy.get_param("~default_cmd_type", 2))
        self.send_without_cmd = bool(rospy.get_param("~send_without_cmd", False))

        udp_module_root = _resolve_udp_module_root(udp_module_root)
        _ensure_udp_module_path(udp_module_root)
        from lib.define.EgoCtrlCmd import EgoCtrlCmd

        self.ego_ctrl_cls = EgoCtrlCmd
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.lock = Lock()
        self.latest_cmd = CtrlCmd()
        self.has_cmd = False

        rospy.Subscriber(self.cmd_topic, CtrlCmd, self._cmd_cb, queue_size=10)
        rospy.Timer(rospy.Duration(1.0 / max(self.hz, 1.0)), self._tick)

        rospy.loginfo(
            "control_udp_pub started | %s -> %s:%d | ctrl_mode=%d gear=%d",
            self.cmd_topic,
            self.ctrl_ip,
            self.ctrl_port,
            self.ctrl_mode,
            self.gear,
        )

    def _cmd_cb(self, msg):
        with self.lock:
            self.latest_cmd = msg
            self.has_cmd = True

    def _to_cmd_type(self, raw_type):
        try:
            cmd_type = int(raw_type)
        except Exception:
            cmd_type = self.default_cmd_type
        if cmd_type not in (1, 2, 3):
            cmd_type = self.default_cmd_type
        return cmd_type

    def _build_packet(self, msg):
        data = self.ego_ctrl_cls()
        data.ctrl_mode = self.ctrl_mode
        data.gear = self.gear
        data.cmd_type = self._to_cmd_type(msg.longlCmdType)

        data.accel = float(msg.accel)
        data.brake = float(msg.brake)
        data.steer = float(msg.steering)
        data.velocity = float(msg.velocity)
        data.acceleration = float(msg.acceleration)
        return data

    def _send(self, data):
        payload = ctypes.string_at(ctypes.addressof(data), ctypes.sizeof(data))
        self.socket.sendto(payload, (self.ctrl_ip, self.ctrl_port))

    def _tick(self, _event):
        with self.lock:
            has_cmd = self.has_cmd
            msg = self.latest_cmd

        if not has_cmd and not self.send_without_cmd:
            return

        data = self._build_packet(msg)
        try:
            self._send(data)
        except Exception as e:
            rospy.logwarn_throttle(2.0, "control_udp_pub send failed: %s", str(e))


def main():
    rospy.init_node("control_udp_pub", anonymous=False)
    ControlUdpPublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
