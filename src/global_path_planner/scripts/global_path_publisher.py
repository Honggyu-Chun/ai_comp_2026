#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import os
import sys
import rospkg
import rospy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class GlobalPathPublisher:
    """
    ROS1 global path publisher (nav_msgs/Path)

    - Input: ~file_name or argv[1] = file name without extension (e.g., "kcity_utm")
    - File:  <package_name>/<folder_name>/<file_name>.txt
    - Output topic: ~topic_name (default: /global_path1)
    - Frame: ~frame_id (default: map)

    Publish mode:
    - Load the path file once, cache it in memory, then publish once with latch.
    - If ~debug is true, disable latch and keep publishing at 30 Hz.
    - ~republish_hz is kept only for launch compatibility and is ignored.
    """

    def __init__(self):
        # launch에서 인스턴스별로 param 주입 가능 (private param)
        self.package_name = rospy.get_param("~package_name", "global_path_planner")
        self.folder_name  = rospy.get_param("~folder_name",  "path_data")
        self.file_name    = str(rospy.get_param("~file_name", "")).strip()
        self.topic_name   = rospy.get_param("~topic_name",   "/global_path1")
        self.frame_id     = rospy.get_param("~frame_id",     "map")

        self.debug        = bool(rospy.get_param("~debug", False))
        self.latch        = bool(rospy.get_param("~latch", True))
        self.republish_hz = float(rospy.get_param("~republish_hz", 0.0))
        self.debug_hz     = 30.0

        # 기존 launch args 방식도 계속 지원한다.
        argv = rospy.myargv(argv=sys.argv)
        if not self.file_name and len(argv) >= 2:
            self.file_name = argv[1]

        if not self.file_name:
            rospy.logerr("Set ~file_name or pass <file_name_without_ext> as argv[1]")
            rospy.logerr("Example: rosrun global_path_planner global_path_publisher.py kcity_utm")
            raise SystemExit(1)

        if self.file_name.endswith(".txt"):
            self.file_name = self.file_name[:-4]

        if self.debug:
            self.latch = False

        # publisher
        self.pub = rospy.Publisher(self.topic_name, Path, queue_size=1, latch=self.latch)

        # load path file once
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id
        self._load_path_file()

        if self.debug:
            rospy.loginfo(
                "[global_path_publisher] debug=true publish %.1f Hz latch=False topic=%s",
                self.debug_hz, self.topic_name
            )
        # publish once; keep republish_hz only for old launch compatibility.
        elif self.republish_hz > 0.0:
            rospy.logwarn(
                "[global_path_publisher] republish_hz=%.3f is ignored; publishing once with latch=%s topic=%s",
                self.republish_hz, str(self.latch), self.topic_name
            )
        else:
            rospy.loginfo(
                "[global_path_publisher] publish-once latch=%s topic=%s",
                str(self.latch), self.topic_name
            )
        self._publish_once()
        if self.debug:
            self.publish_timer = rospy.Timer(
                rospy.Duration(1.0 / self.debug_hz), self._publish_timer_cb
            )
        rospy.loginfo("[global_path_publisher] publish initialized. node will stay alive (spin).")

    def _resolve_file_path(self) -> str:
        """txt 파일의 절대 경로를 계산."""
        rospack = rospkg.RosPack()
        pkgpath = rospack.get_path(self.package_name)
        return os.path.join(pkgpath, self.folder_name, f"{self.file_name}.txt")

    def _load_path_file(self) -> None:
        file_path = self._resolve_file_path()

        if not os.path.isfile(file_path):
            rospy.logerr("[global_path_publisher] file not found: %s", file_path)
            raise SystemExit(1)

        cnt = 0
        with open(file_path, "r") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                if line.startswith("#") or line.startswith("//"):
                    continue

                field = line.split()
                if len(field) < 2:
                    continue

                try:
                    x = float(field[0])
                    y = float(field[1])
                except ValueError:
                    continue

                ps = PoseStamped()
                ps.header.frame_id = self.frame_id
                ps.pose.position.x = x
                ps.pose.position.y = y
                ps.pose.position.z = 0.0
                ps.pose.orientation.w = 1.0  # (0,0,0,1) 단위 quaternion

                self.path_msg.poses.append(ps)
                cnt += 1

        if cnt < 2:
            rospy.logerr("[global_path_publisher] not enough points loaded: %d from %s", cnt, file_path)
            raise SystemExit(1)

        now = rospy.Time.now()
        self.path_msg.header.stamp = now
        for ps in self.path_msg.poses:
            ps.header.stamp = now

        p0 = self.path_msg.poses[0].pose.position
        pe = self.path_msg.poses[-1].pose.position
        rospy.loginfo("[global_path_publisher] loaded points=%d file=%s", cnt, os.path.basename(file_path))
        rospy.loginfo("[global_path_publisher] first=(%.3f,%.3f) last=(%.3f,%.3f)", p0.x, p0.y, pe.x, pe.y)

    def _stamp_path_header(self, stamp) -> None:
        """정적 경로이므로 Path header stamp만 갱신한다."""
        self.path_msg.header.stamp = stamp

    def _publish_once(self) -> None:
        # roslaunch/rosrun 직후 연결 타이밍 완화
        rospy.sleep(0.2)
        self._publish_path()

    def _publish_path(self) -> None:
        self._stamp_path_header(rospy.Time.now())
        self.pub.publish(self.path_msg)

    def _publish_timer_cb(self, _event) -> None:
        self._publish_path()


def main() -> None:
    # anonymous=True는 roslaunch에서 지정한 node name을 깨뜨릴 수 있음.
    rospy.init_node("global_path_publisher", anonymous=False)
    _ = GlobalPathPublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
