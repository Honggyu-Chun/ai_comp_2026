#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# MORAI UDP ObjectInfo(최대 20객체) 를 수신해 morai_msgs/ObjectStatusList 로
# /Object_topic 에 발행하는 브릿지.
#   목적: global_path_planner/obstacle_info_publisher.cpp 가 /Object_topic 을
#         그대로 구독하도록, MORAI 자체 ROS 브릿지와 동일한 메시지를 재현한다.
#   프로토콜: MORAI_UDP_NetworkModule-24.R2.0/lib/define/ObjectInfo.py
#   포트: sim host 7605 -> destination 7505 (우리는 destination 7505 로 bind 수신).

import os
import sys
from pathlib import Path

import rospy
from morai_msgs.msg import ObjectStatus, ObjectStatusList


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
    # .../catkin_ws/src/morai_gadis_set/scripts/object_info_udp_bridge.py
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


# MORAI UDP ObjectInfo.objType 공식 규약 (MORAI-ADModule/perception/object_info.py):
#   0: 보행자(person) / 1, 2: 차량(vehicle) / 3: 신호등(traffic light)
# 이를 obstacle_info_publisher 가 기대하는 ROS ObjectStatus.type 로 번역한다:
#   0: 보행자 / 1: NPC 차량 / 2: 정적 장애물
# - UDP objType 2 는 '정적'이 아니라 '차량'이므로 1(NPC)로 매핑해야 한다(그대로 두면 차량을 정적으로 오분류).
# - 신호등(3)은 장애물이 아니므로 obstacle_info 로 내보내지 않는다(팬텀 장애물/불필요 제동 방지).
# - UDP ObjectInfo 에는 '정적 장애물' 전용 코드가 없다 → 차량 외 미확인은 NPC 로 두고
#   정/동적 판단은 하류에서 속도로 처리한다(decision_architecture §4).
def _ros_type_for(obj_type):
    if obj_type == 0:
        return 0          # 보행자
    if obj_type in (1, 2):
        return 1          # 차량 -> NPC
    if obj_type == 3:
        return None       # 신호등 -> 장애물 아님, 스킵
    return 1              # 미확인 -> 보수적으로 NPC


def _to_status(obj, ros_type):
    # UDP ObjectInfo.Data 한 개 -> morai_msgs/ObjectStatus (ros_type 은 이미 번역된 값).
    # 필드 단위/규약은 obstacle_info_publisher 가 이미 가정한 MORAI ROS 규약과 맞춘다:
    #   heading[deg], size=(length, width, height), velocity[km/h 가정].
    status = ObjectStatus()
    status.unique_id = int(obj.obj_id)
    status.type = int(ros_type)
    status.name = ""

    status.heading = float(obj.heading)

    status.velocity.x = float(obj.vel_x)
    status.velocity.y = float(obj.vel_y)
    status.velocity.z = float(obj.vel_z)

    status.acceleration.x = float(obj.accel_x)
    status.acceleration.y = float(obj.accel_y)
    status.acceleration.z = float(obj.accel_z)

    status.size.x = float(obj.size_x)
    status.size.y = float(obj.size_y)
    status.size.z = float(obj.size_z)

    status.position.x = float(obj.pose_x)
    status.position.y = float(obj.pose_y)
    status.position.z = float(obj.pose_z)
    return status


def main():
    rospy.init_node("object_info_udp_bridge", anonymous=False)

    udp_module_root = rospy.get_param("~udp_module_root", "")
    obj_ip = rospy.get_param("~obj_ip", "127.0.0.1")
    obj_port = int(rospy.get_param("~obj_port", 7505))  # Destination Port (bind)
    hz = float(rospy.get_param("~hz", 20.0))
    frame_id = rospy.get_param("~frame_id", "map")
    topic = rospy.get_param("~topic", "/Object_topic")

    udp_module_root = _resolve_udp_module_root(udp_module_root)
    _ensure_udp_module_path(udp_module_root)

    from lib.network.UDP import Receiver
    from lib.define.ObjectInfo import ObjectInfo

    obj_pub = rospy.Publisher(topic, ObjectStatusList, queue_size=10)
    obj_receiver = Receiver(obj_ip, obj_port, ObjectInfo())

    rospy.loginfo(
        "object_info_udp_bridge started | ObjectInfo %s:%d -> %s",
        obj_ip,
        obj_port,
        topic,
    )

    rate = rospy.Rate(hz)
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        obj_data = obj_receiver.get_data()

        if obj_data is not None:
            out = ObjectStatusList()
            out.header.stamp = now
            out.header.frame_id = frame_id

            seen_types = set()
            for obj in obj_data.data:
                if int(obj.obj_id) == 0:  # 빈 슬롯(예제 규약)
                    continue
                obj_type = int(obj.objType)
                seen_types.add(obj_type)
                ros_type = _ros_type_for(obj_type)
                if ros_type is None:       # 신호등 등 -> 장애물 아님, 스킵
                    continue
                status = _to_status(obj, ros_type)
                if ros_type == 0:          # 보행자
                    out.pedestrian_list.append(status)
                elif ros_type == 2:        # 정적 장애물(현 매핑상 미발생, 향후 대비)
                    out.obstacle_list.append(status)
                else:                      # NPC 차량
                    out.npc_list.append(status)

            out.num_of_npcs = len(out.npc_list)
            out.num_of_pedestrian = len(out.pedestrian_list)
            out.num_of_obstacle = len(out.obstacle_list)

            if seen_types:
                rospy.loginfo_throttle(
                    5.0, "object_info_udp_bridge: objType seen=%s", sorted(seen_types)
                )
            obj_pub.publish(out)

        rate.sleep()


if __name__ == "__main__":
    main()
