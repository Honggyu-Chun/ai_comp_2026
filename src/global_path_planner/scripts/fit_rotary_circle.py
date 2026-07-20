#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 로터리 원 피팅 캘리브레이션 유틸.
#   /obstacle_info(base_link 상대) + /odometry/filtered(EKF 융합 UTM pose)로 순환 NPC를 월드(UTM) 복원해
#   일정시간 샘플을 모은 뒤 최소자승 원피팅(Kåsa)으로 center_x/y, ring_radius_m 을 출력한다.
#   회전방향(dir)은 cross(p-C, v) 부호 다수결로 함께 출력.
#   → rotary_entry.launch 의 center_x/center_y/ring_radius_m/dir 파라미터에 넣으면 된다.

import math
import random
import rospy
import numpy as np
from katri_msgs.msg import ObstacleInfoArray
from nav_msgs.msg import Odometry


def kasa_fit(pts):
    # Kåsa 대수 최소자승 원피팅. pts: (N,2) → (cx, cy, R)
    x, y = pts[:, 0], pts[:, 1]
    A = np.c_[2 * x, 2 * y, np.ones(len(x))]
    b = x * x + y * y
    (a, bb, c), *_ = np.linalg.lstsq(A, b, rcond=None)
    R = math.sqrt(max(0.0, c + a * a + bb * bb))
    return a, bb, R


def circle_from_3(p1, p2, p3):
    # 세 점의 외접원. 공선(collinear)이면 None.
    ax, ay = p1; bx, by = p2; cx, cy = p3
    d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-9:
        return None
    a2 = ax * ax + ay * ay; b2 = bx * bx + by * by; c2 = cx * cx + cy * cy
    ux = (a2 * (by - cy) + b2 * (cy - ay) + c2 * (ay - by)) / d
    uy = (a2 * (cx - bx) + b2 * (ax - cx) + c2 * (bx - ax)) / d
    R = math.hypot(ax - ux, ay - uy)
    return ux, uy, R


class RotaryCircleFitter:
    def __init__(self):
        self.duration = float(rospy.get_param("~duration", 10.0))
        self.v_min = float(rospy.get_param("~v_agent_min_mps", 0.5))
        # 공간 게이트: 자차 base_link 기준 이 거리 안 장애물만 수집(먼 다른 차 제외)
        self.capture_radius = float(rospy.get_param("~capture_radius_m", 40.0))
        # RANSAC
        self.ransac_iters = int(rospy.get_param("~ransac_iters", 300))
        self.inlier_tol = float(rospy.get_param("~inlier_tol_m", 1.0))     # |r-R| 이내면 inlier
        self.min_inliers = int(rospy.get_param("~min_inliers", 20))
        self.r_min = float(rospy.get_param("~r_min_m", 2.0))               # 후보 원 반경 sanity
        self.r_max = float(rospy.get_param("~r_max_m", 60.0))
        self.ego = None
        self.samples = []          # (x, y, vx, vy) 월드(UTM)
        ego_topic = rospy.get_param("~ego_topic", "/odometry/filtered")
        rospy.Subscriber(ego_topic, Odometry, self.ego_cb, queue_size=1)
        rospy.Subscriber("/obstacle_info", ObstacleInfoArray, self.obs_cb, queue_size=1)

    def ego_cb(self, msg):
        # nav_msgs/Odometry (EKF 융합, UTM): pose + yaw(quaternion)
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.ego = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def obs_cb(self, msg):
        if self.ego is None:
            return
        ex, ey, eyaw = self.ego
        c, s = math.cos(eyaw), math.sin(eyaw)
        for o in msg.obstacles:
            if o.is_pedestrian:
                continue
            if o.speed_mps < self.v_min:
                continue
            # 공간 게이트: 자차 근처(로터리 인근)만 → 먼 다른 차 배제
            if math.hypot(o.x_m, o.y_m) > self.capture_radius:
                continue
            # base_link → 월드 복원 (obstacle_info_publisher 회전의 역변환)
            wx = ex + (c * o.x_m - s * o.y_m)
            wy = ey + (s * o.x_m + c * o.y_m)
            hd = eyaw + o.yaw_rad
            self.samples.append((wx, wy, o.speed_mps * math.cos(hd), o.speed_mps * math.sin(hd)))

    def ransac_circle(self, pts):
        # 3점 무작위 → 외접원 → inlier 최다 모델. 최종 inlier로 Kåsa 재피팅.
        n = len(pts)
        best_inliers = None
        best_count = 0
        for _ in range(self.ransac_iters):
            i, j, k = random.sample(range(n), 3)
            circ = circle_from_3(pts[i], pts[j], pts[k])
            if circ is None:
                continue
            cx, cy, R = circ
            if R < self.r_min or R > self.r_max:
                continue
            d = np.abs(np.hypot(pts[:, 0] - cx, pts[:, 1] - cy) - R)
            inliers = d < self.inlier_tol
            cnt = int(np.count_nonzero(inliers))
            if cnt > best_count:
                best_count = cnt
                best_inliers = inliers
        if best_inliers is None or best_count < self.min_inliers:
            return None, None
        cx, cy, R = kasa_fit(pts[best_inliers])  # inlier로 정밀 재피팅
        return (cx, cy, R), best_inliers

    def fit(self):
        if len(self.samples) < max(3, self.min_inliers):
            rospy.logwarn("샘플 부족(%d) — NPC 순환/토픽/capture_radius 확인", len(self.samples))
            return
        arr = np.array(self.samples, dtype=float)
        pts = arr[:, 0:2]
        result, inliers = self.ransac_circle(pts)
        if result is None:
            rospy.logwarn("RANSAC 실패(inlier<%d) — 다른 차 다수/반경 sanity 확인. 전체점 Kåsa 폴백.",
                          self.min_inliers)
            cx, cy, R = kasa_fit(pts)
            inliers = np.ones(len(pts), dtype=bool)
        else:
            cx, cy, R = result
        inl = arr[inliers]
        # 방향: inlier 의 cross((p-C), v) 부호 다수결
        cross = (inl[:, 0] - cx) * inl[:, 3] - (inl[:, 1] - cy) * inl[:, 2]
        direction = 1 if np.sum(np.sign(cross)) >= 0 else -1
        r_i = np.hypot(inl[:, 0] - cx, inl[:, 1] - cy)
        rospy.loginfo("=" * 52)
        rospy.loginfo("로터리 원피팅 (수집 %d, inlier %d, 이상치 %d 제거)",
                      len(pts), int(np.count_nonzero(inliers)), len(pts) - int(np.count_nonzero(inliers)))
        rospy.loginfo("  center_x := %.4f", cx)
        rospy.loginfo("  center_y := %.4f", cy)
        rospy.loginfo("  ring_radius_m := %.4f", R)
        rospy.loginfo("  dir := %d  (%s)", direction, "CCW 반시계" if direction > 0 else "CW 시계")
        rospy.loginfo("  inlier 반경 잔차 std = %.3f m (작을수록 신뢰)", float(np.std(r_i)))
        rospy.loginfo("  → rotary_entry.launch 의 center_x/center_y/ring_radius_m/dir 에 반영")
        rospy.loginfo("=" * 52)


def main():
    rospy.init_node("fit_rotary_circle", anonymous=True)
    f = RotaryCircleFitter()
    rospy.loginfo("fit_rotary_circle: %.1f초간 순환 NPC 샘플 수집...", f.duration)
    rospy.sleep(f.duration)
    f.fit()


if __name__ == "__main__":
    main()
