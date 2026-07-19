#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
from pathlib import Path
import os

import rospy
from std_msgs.msg import Header
from morai_msgs.msg import GPSMessage
from morai_msgs.msg import EgoVehicleStatus as MoraiEgoMsg


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
    # .../catkin_ws/src/morai_gadis_set/scripts/morai_udp_bridge.py
    # -> .../catkin_ws/src/MORAI_UDP_NetworkModule-24.R2.0
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


def _to_int(value, default=0):
    try:
        return int(value)
    except Exception:
        return default


def _has_payload(obj):
    header = getattr(obj, "header", None)
    if header is None:
        return True
    try:
        return bytes(header).strip(b"\x00") != b""
    except Exception:
        return True


def main():
    rospy.init_node("morai_udp_bridge", anonymous=False)

    udp_module_root = rospy.get_param("~udp_module_root", "")
    gps_ip = rospy.get_param("~gps_ip", "127.0.0.1")
    gps_port = int(rospy.get_param("~gps_port", 22222))
    ego_ip = rospy.get_param("~ego_ip", "127.0.0.1")
    ego_port = int(rospy.get_param("~ego_port", 11112))
    hz = float(rospy.get_param("~hz", 20.0))
    frame_id = rospy.get_param("~frame_id", "map")

    udp_module_root = _resolve_udp_module_root(udp_module_root)
    _ensure_udp_module_path(udp_module_root)

    from lib.network.UDP import Receiver
    from lib.define.GPS import GPS
    from lib.define.EgoVehicleStatus import EgoVehicleStatus as UdpEgoStatus

    gps_pub = rospy.Publisher("/gps", GPSMessage, queue_size=10)
    ego_pub = rospy.Publisher("/morai/ego_topic", MoraiEgoMsg, queue_size=10)

    gps_receiver = Receiver(gps_ip, gps_port, GPS())
    ego_receiver = Receiver(ego_ip, ego_port, UdpEgoStatus())

    rospy.loginfo(
        "morai_udp_bridge started | GPS %s:%d -> /gps | EGO %s:%d -> /morai/ego_topic",
        gps_ip,
        gps_port,
        ego_ip,
        ego_port,
    )

    rate = rospy.Rate(hz)
    while not rospy.is_shutdown():
        now = rospy.Time.now()

        gps_obj = gps_receiver.get_data()
        if gps_obj is not None and _has_payload(gps_obj):
            gps_msg = GPSMessage()
            gps_msg.header = Header(stamp=now, frame_id="gps")
            gps_msg.latitude = float(getattr(gps_obj.gprmc, "lat", 0.0))
            gps_msg.longitude = float(getattr(gps_obj.gprmc, "lon", 0.0))
            gps_msg.altitude = float(getattr(gps_obj.gpgga, "alt", 0.0))
            gps_msg.status = _to_int(getattr(gps_obj.gpgga, "quality", 0), 0)
            gps_pub.publish(gps_msg)

        ego_obj = ego_receiver.get_data()
        if ego_obj is not None and _has_payload(ego_obj):
            ego_msg = MoraiEgoMsg()
            ego_msg.header.stamp = now
            ego_msg.header.frame_id = frame_id

            ego_msg.position.x = float(getattr(ego_obj, "pos_x", 0.0))
            ego_msg.position.y = float(getattr(ego_obj, "pos_y", 0.0))
            ego_msg.position.z = float(getattr(ego_obj, "pos_z", 0.0))

            ego_msg.velocity.x = float(getattr(ego_obj, "vel_x", 0.0))
            ego_msg.velocity.y = float(getattr(ego_obj, "vel_y", 0.0))
            ego_msg.velocity.z = float(getattr(ego_obj, "vel_z", 0.0))

            ego_msg.acceleration.x = float(getattr(ego_obj, "accel_x", 0.0))
            ego_msg.acceleration.y = float(getattr(ego_obj, "accel_y", 0.0))
            ego_msg.acceleration.z = float(getattr(ego_obj, "accel_z", 0.0))

            ego_msg.heading = float(getattr(ego_obj, "yaw", 0.0))
            ego_msg.accel = float(getattr(ego_obj, "accel", 0.0))
            ego_msg.brake = float(getattr(ego_obj, "brake", 0.0))
            ego_msg.wheel_angle = float(getattr(ego_obj, "steer", 0.0))

            ego_msg.tire_lateral_force_fl = float(getattr(ego_obj, "tire_lateral_force_fl", 0.0))
            ego_msg.tire_lateral_force_fr = float(getattr(ego_obj, "tire_lateral_force_fr", 0.0))
            ego_msg.tire_lateral_force_rl = float(getattr(ego_obj, "tire_lateral_force_rl", 0.0))
            ego_msg.tire_lateral_force_rr = float(getattr(ego_obj, "tire_lateral_force_rr", 0.0))

            ego_msg.side_slip_angle_fl = float(getattr(ego_obj, "side_slip_angle_fl", 0.0))
            ego_msg.side_slip_angle_fr = float(getattr(ego_obj, "side_slip_angle_fr", 0.0))
            ego_msg.side_slip_angle_rl = float(getattr(ego_obj, "side_slip_angle_rl", 0.0))
            ego_msg.side_slip_angle_rr = float(getattr(ego_obj, "side_slip_angle_rr", 0.0))

            ego_msg.tire_cornering_stiffness_fl = float(getattr(ego_obj, "tire_cornering_stiffness_fl", 0.0))
            ego_msg.tire_cornering_stiffness_fr = float(getattr(ego_obj, "tire_cornering_stiffness_fr", 0.0))
            ego_msg.tire_cornering_stiffness_rl = float(getattr(ego_obj, "tire_cornering_stiffness_rl", 0.0))
            ego_msg.tire_cornering_stiffness_rr = float(getattr(ego_obj, "tire_cornering_stiffness_rr", 0.0))

            ego_pub.publish(ego_msg)

        rate.sleep()


if __name__ == "__main__":
    main()
