#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from math import sqrt, pow
import numpy as np
import matplotlib
matplotlib.use('Agg')  # #3 headless: shutdown 콜백에서 GUI 백엔드 행(hang) 방지
import matplotlib.pyplot as plt
import os
from scipy.interpolate import UnivariateSpline, splprep, splev


class make_path:

    def __init__(self):
        filename = 'global_path_2.txt'

        rospy.init_node('path_maker', anonymous=False)
        filename = rospy.get_param("~file_name", filename)
        # sample_distance 는 소비 노드(odom_path_publisher)의 waypoint_interval 과 반드시 일치시킬 것.
        # (back_steps/front_steps 가 거리→인덱스 환산에 균일 간격을 전제 — 둘 다 0.1 기본)
        self.sample_distance = float(rospy.get_param("~sample_distance", 0.1))
        # 스무딩 강도: 결과 경로가 원본에서 벗어나도 되는 평균 편차[m]. 클수록 더 매끄러움(원본에서 더 이탈).
        # UnivariateSpline 의 s(잔차제곱합 상한) = N * dev^2 로 환산한다. (0.1 하드코딩 = 사실상 무효였음)
        self.smoothing_deviation_m = float(rospy.get_param("~smoothing_deviation_m", 0.5))
        # 폐곡선(트랙) 모드: True 면 되돌아온 꼬리를 자르고 주기 스플라인(splprep per=1)으로 seam 을 닫는다.
        self.closed_loop = bool(rospy.get_param("~closed_loop", False))
        # 시작점 복귀 감지 반경[m]: 이 반경 밖으로 나갔다가 다시 들어온 뒤 시작점 최근접점을 한 바퀴 끝으로 본다.
        self.closed_loop_radius = float(rospy.get_param("~closed_loop_radius", 2.0))
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

    def smooth_open(self, x_coords, y_coords):
        # 열린 경로: arc-length 파라미터화 + UnivariateSpline(s=N*dev^2) 스무딩, 끝점 포함(#2).
        distances = np.sqrt(np.diff(x_coords) ** 2 + np.diff(y_coords) ** 2)
        cumulative = np.insert(np.cumsum(distances), 0, 0)
        total = cumulative[-1]
        # #1 s(잔차제곱합 상한) = N * dev^2 를 명시한 UnivariateSpline (보간+set_smoothing 무효 문제 해결)
        s_value = len(cumulative) * (self.smoothing_deviation_m ** 2)
        spl_x = UnivariateSpline(cumulative, x_coords, k=3, s=s_value)
        spl_y = UnivariateSpline(cumulative, y_coords, k=3, s=s_value)
        new_d = np.arange(0, total, self.sample_distance)
        if new_d.size == 0 or new_d[-1] < total:
            new_d = np.append(new_d, total)  # #2 끝점 포함
        return spl_x(new_d), spl_y(new_d)

    def _trim_to_one_loop(self, x_coords, y_coords):
        # 시작점 반경 밖으로 나갔다가 되돌아온 뒤, 시작점 최근접 index 까지 잘라 한 바퀴만 남긴다.
        # 감지 실패(폐곡선 아님/미복귀) 시 None → 호출부에서 열린 스무딩으로 폴백.
        d0 = np.hypot(x_coords - x_coords[0], y_coords - y_coords[0])
        r = self.closed_loop_radius
        left = np.where(d0 > r)[0]
        if left.size == 0:
            rospy.logwarn("path_maker: never left start radius %.2fm -> treat as open path", r)
            return None
        i_leave = left[0]
        back = np.where(d0[i_leave:] <= r)[0]
        if back.size == 0:
            rospy.logwarn("path_maker: never returned within %.2fm of start -> treat as open path", r)
            return None
        i_return = i_leave + back[0]
        j = i_return + int(np.argmin(d0[i_return:]))  # 복귀 후 시작점 최근접 = 한 바퀴 끝
        rospy.loginfo(
            "path_maker: loop closure at idx %d/%d, gap-to-start %.3fm (trimmed %d tail pts)",
            j, len(x_coords) - 1, d0[j], len(x_coords) - 1 - j)
        return x_coords[:j + 1], y_coords[:j + 1]

    def smooth_closed(self, x_coords, y_coords):
        # 폐곡선: 꼬리 트리밍 → 주기 스플라인(per=1)으로 seam 닫기 → 균일 호길이 리샘플(닫는 점 중복 없음).
        trimmed = self._trim_to_one_loop(x_coords, y_coords)
        if trimmed is None:
            return self.smooth_open(x_coords, y_coords)
        tx, ty = trimmed
        if len(tx) < 4:
            rospy.logwarn("path_maker: loop too short (%d pts) -> fallback to open", len(tx))
            return self.smooth_open(x_coords, y_coords)

        # per=1: 첫↔끝을 C2 연속으로 자동 연결해 잔여 gap 을 흡수(끝점=첫점 불일치 문제 해결).
        s_value = len(tx) * (self.smoothing_deviation_m ** 2)
        tck, _ = splprep([tx, ty], s=s_value, per=1)

        # splprep 의 u 는 chord-length 라 정확한 균일 간격 보장 위해 실제 호길이로 2패스 리샘플
        u_dense = np.linspace(0.0, 1.0, max(1000, 10 * len(tx)))
        xd, yd = splev(u_dense, tck)
        arc = np.insert(np.cumsum(np.hypot(np.diff(xd), np.diff(yd))), 0, 0.0)
        total = arc[-1]
        n_out = max(4, int(round(total / self.sample_distance)))
        # endpoint=False → 닫는 점 중복 제거(#4), 끝→첫 간격 = sample_distance(#1 제약)
        s_targets = np.linspace(0.0, total, n_out, endpoint=False)
        u_targets = np.interp(s_targets, arc, u_dense)
        sx, sy = splev(u_targets, tck)
        return np.asarray(sx), np.asarray(sy)

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

        
        x_coords = np.asarray(x_coords, dtype=float)
        y_coords = np.asarray(y_coords, dtype=float)

        # #4(부수) 3차 스플라인은 최소 4점 필요 → 너무 짧으면 스무딩 생략(shutdown 콜백 크래시 방지)
        if len(x_coords) < 4:
            rospy.logwarn("path_maker: only %d raw points, skip smoothing", len(x_coords))
            return

        if self.closed_loop:
            smoothed_x, smoothed_y = self.smooth_closed(x_coords, y_coords)
        else:
            smoothed_x, smoothed_y = self.smooth_open(x_coords, y_coords)

        plt.figure()
        plt.plot(x_coords, y_coords, '-k', lw=self.line_width, label='raw')
        plt.plot(x_coords, y_coords, '.k')
        plt.plot(smoothed_x, smoothed_y, '-b', lw=self.line_width, label='smoothed')
        plt.plot(smoothed_x, smoothed_y, '.b')
        plt.axis('equal')
        plt.grid(True)
        plt.legend()
        plt.title('Waypoints Path')
        plt.xlabel('X Coordinates')
        plt.ylabel('Y Coordinates')

        # #3 plt.show() 는 shutdown 콜백에서 블로킹/행 → 이미지 파일로 저장(Agg 백엔드, headless 안전)
        plot_dir = os.path.join(os.path.dirname(self.full_path), 'plot')
        os.makedirs(plot_dir, exist_ok=True)
        plot_path = os.path.join(
            plot_dir, os.path.basename(self.full_path).replace('.txt', '.png'))
        plt.savefig(plot_path, dpi=150)
        plt.close()
        rospy.loginfo("path_maker: saved plot to %s", plot_path)

        # 자가검증: 소비 노드 제약(seam gap ≈ sample_distance, 균일 간격) 확인용 로그
        seg = np.hypot(np.diff(smoothed_x), np.diff(smoothed_y))
        seam_gap = float(np.hypot(smoothed_x[0] - smoothed_x[-1], smoothed_y[0] - smoothed_y[-1]))
        rospy.loginfo(
            "path_maker: closed=%s  seam(last->first)=%.3fm  step mean/std=%.3f/%.3fm (target %.3f)",
            self.closed_loop, seam_gap,
            float(np.mean(seg)) if seg.size else 0.0,
            float(np.std(seg)) if seg.size else 0.0,
            self.sample_distance)

        sampled_path = self.full_path.replace('.txt', '_sm.txt')
        with open(sampled_path, 'w') as f:
            for x, y in zip(smoothed_x, smoothed_y):
                f.write(f'{x}\t{y}\n')
        rospy.loginfo("path_maker: saved smoothed path to %s (%d pts)", sampled_path, len(smoothed_x))
       


if __name__ == '__main__':
    try:
        test_track = make_path()
    except rospy.ROSInterruptException:
        pass
