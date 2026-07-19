#!/usr/bin/env python3

import math
import threading
from collections import deque

import numpy as np
import rospy
import tf
import tf.transformations as tft
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def angle_lerp(current, target, alpha):
    return normalize_angle(current + alpha * normalize_angle(target - current))


def limited_angle_lerp(current, target, alpha, max_step):
    correction = alpha * normalize_angle(target - current)
    if max_step > 0.0:
        correction = max(-max_step, min(max_step, correction))
    return normalize_angle(current + correction)


def yaw_from_quaternion(q):
    return tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


def is_zero_stamp(stamp):
    return stamp.secs == 0 and stamp.nsecs == 0


class GpsImuFusionNode:
    def __init__(self):
        self.gps_odom_topic = rospy.get_param("~gps_odom_topic", "/gps_utm_odom")
        self.imu_topic = rospy.get_param("~imu_topic", "/imu/data")
        self.output_odom_topic = rospy.get_param("~output_odom_topic", "/odometry/filtered")

        self.frame_id = rospy.get_param("~frame_id", "map")
        self.child_frame_id = rospy.get_param("~child_frame_id", "base_link")
        self.publish_tf = rospy.get_param("~publish_tf", False)

        self.rate_hz = float(rospy.get_param("~rate_hz", 50.0))
        self.use_message_stamp = rospy.get_param("~use_message_stamp", False)
        self.gps_timeout_s = float(rospy.get_param("~gps_timeout_s", 0.5))
        self.imu_timeout_s = float(rospy.get_param("~imu_timeout_s", 0.5))
        self.max_predict_dt_s = float(rospy.get_param("~max_predict_dt_s", 0.1))
        self.gps_delay_comp_s = float(rospy.get_param("~gps_delay_comp_s", 0.0))
        self.use_fixed_lag_gps = rospy.get_param("~use_fixed_lag_gps", False)
        self.gps_measurement_delay_s = max(
            0.0, float(rospy.get_param("~gps_measurement_delay_s", 0.0))
        )
        self.fixed_lag_history_s = max(
            0.1, float(rospy.get_param("~fixed_lag_history_s", 0.5))
        )
        self.use_ekf = rospy.get_param("~use_ekf", False)
        self.ekf_initial_covariance = float(rospy.get_param("~ekf_initial_covariance", 1.0))
        self.ekf_process_noise_x = float(rospy.get_param("~ekf_process_noise_x", 0.08))
        self.ekf_process_noise_y = float(rospy.get_param("~ekf_process_noise_y", 0.08))
        self.ekf_process_noise_yaw = float(rospy.get_param("~ekf_process_noise_yaw", 0.01))
        self.ekf_process_noise_speed = float(rospy.get_param("~ekf_process_noise_speed", 0.4))
        self.ekf_process_noise_yaw_rate = float(
            rospy.get_param("~ekf_process_noise_yaw_rate", 0.2)
        )
        self.ekf_gps_position_variance = float(
            rospy.get_param("~ekf_gps_position_variance", 0.35)
        )
        self.ekf_gps_speed_variance = float(rospy.get_param("~ekf_gps_speed_variance", 0.16))
        self.ekf_imu_yaw_variance = float(rospy.get_param("~ekf_imu_yaw_variance", 0.01))
        self.ekf_imu_yaw_rate_variance = float(
            rospy.get_param("~ekf_imu_yaw_rate_variance", 0.04)
        )
        self.ekf_min_position_variance = float(
            rospy.get_param("~ekf_min_position_variance", 0.0)
        )
        self.ekf_min_yaw_variance = float(rospy.get_param("~ekf_min_yaw_variance", 0.0))
        self.ekf_min_speed_variance = float(rospy.get_param("~ekf_min_speed_variance", 0.0))
        self.ekf_min_yaw_rate_variance = float(
            rospy.get_param("~ekf_min_yaw_rate_variance", 0.0)
        )
        self.use_gps_course_yaw_update = rospy.get_param("~use_gps_course_yaw_update", False)
        self.gps_course_min_speed_mps = float(
            rospy.get_param("~gps_course_min_speed_mps", 8.0)
        )
        self.gps_course_max_yaw_rate_rad_s = float(
            rospy.get_param("~gps_course_max_yaw_rate_rad_s", 0.10)
        )
        self.gps_course_min_distance_m = float(
            rospy.get_param("~gps_course_min_distance_m", 0.5)
        )
        self.gps_course_max_yaw_innovation_deg = float(
            rospy.get_param("~gps_course_max_yaw_innovation_deg", 20.0)
        )
        self.ekf_gps_course_yaw_variance = float(
            rospy.get_param("~ekf_gps_course_yaw_variance", 0.02)
        )
        self.use_ekf_startup_alignment = rospy.get_param(
            "~use_ekf_startup_alignment", False
        )
        self.ekf_startup_min_speed_mps = float(
            rospy.get_param("~ekf_startup_min_speed_mps", 6.0)
        )
        self.ekf_startup_min_distance_m = float(
            rospy.get_param("~ekf_startup_min_distance_m", 0.7)
        )
        self.ekf_startup_max_wait_s = float(rospy.get_param("~ekf_startup_max_wait_s", 0.5))
        self.ekf_startup_max_yaw_rate_rad_s = float(
            rospy.get_param("~ekf_startup_max_yaw_rate_rad_s", 0.12)
        )
        self.ekf_startup_course_yaw_alpha = float(
            rospy.get_param("~ekf_startup_course_yaw_alpha", 0.4)
        )

        self.position_alpha = float(rospy.get_param("~position_alpha", 0.25))
        self.speed_alpha = float(rospy.get_param("~speed_alpha", 0.35))
        self.imu_yaw_alpha = float(rospy.get_param("~imu_yaw_alpha", 0.85))
        self.gps_yaw_alpha = float(rospy.get_param("~gps_yaw_alpha", 0.0))
        self.imu_orientation_lpf_tau_s = float(
            rospy.get_param("~imu_orientation_lpf_tau_s", 0.0)
        )
        self.imu_yaw_correction_max_rate_deg_s = float(
            rospy.get_param("~imu_yaw_correction_max_rate_deg_s", 0.0)
        )
        self.gps_yaw_min_speed_mps = float(rospy.get_param("~gps_yaw_min_speed_mps", 0.0))
        self.gps_yaw_max_step_deg = float(rospy.get_param("~gps_yaw_max_step_deg", 0.0))
        self.use_speed_adaptive_position_alpha = rospy.get_param(
            "~use_speed_adaptive_position_alpha", False
        )
        self.position_alpha_low_speed = float(
            rospy.get_param("~position_alpha_low_speed", self.position_alpha)
        )
        self.position_alpha_mid_speed = float(
            rospy.get_param("~position_alpha_mid_speed", self.position_alpha)
        )
        self.position_alpha_high_speed = float(
            rospy.get_param("~position_alpha_high_speed", self.position_alpha)
        )
        self.low_speed_threshold_mps = float(rospy.get_param("~low_speed_threshold_mps", 3.0))
        self.high_speed_threshold_mps = float(rospy.get_param("~high_speed_threshold_mps", 10.0))
        self.enable_gps_outlier_rejection = rospy.get_param(
            "~enable_gps_outlier_rejection", False
        )
        self.gps_innovation_warn_m = float(rospy.get_param("~gps_innovation_warn_m", 1.0))
        self.gps_innovation_reject_m = float(rospy.get_param("~gps_innovation_reject_m", 3.0))
        self.gps_outlier_alpha_scale = float(rospy.get_param("~gps_outlier_alpha_scale", 0.25))

        self.use_imu_orientation = rospy.get_param("~use_imu_orientation", True)
        self.use_imu_yaw_rate = rospy.get_param("~use_imu_yaw_rate", True)
        self.use_speed_prediction = rospy.get_param("~use_speed_prediction", True)
        self.prediction_speed_scale = float(rospy.get_param("~prediction_speed_scale", 1.0))

        self.imu_yaw_offset = math.radians(float(rospy.get_param("~imu_yaw_offset_deg", 0.0)))
        self.invert_yaw_rate = rospy.get_param("~invert_yaw_rate", False)
        self.yaw_rate_bias = float(rospy.get_param("~yaw_rate_bias", 0.0))
        self.input_speed_scale = float(rospy.get_param("~input_speed_scale", 1.0))
        self.gps_to_base_x = float(rospy.get_param("~gps_to_base_x", 0.0))
        self.gps_to_base_y = float(rospy.get_param("~gps_to_base_y", 0.0))

        self.position_variance = float(rospy.get_param("~position_variance", 0.25))
        self.yaw_variance = float(rospy.get_param("~yaw_variance", 0.0025))
        self.speed_variance = float(rospy.get_param("~speed_variance", 0.04))
        self.debug_print = rospy.get_param("~debug_print", False)

        self.lock = threading.RLock()
        self.initialized = False
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.speed = 0.0
        self.yaw_rate = 0.0
        self.state = np.zeros(5, dtype=float)
        self.P = np.eye(5) * self.ekf_initial_covariance
        self.last_predict_time = None
        self.last_gps_time = None
        self.last_imu_time = None
        self.last_imu_yaw = None
        self.last_gps_base_x = None
        self.last_gps_base_y = None
        self.startup_gps_samples = []
        self.startup_first_time = None
        self.filtered_imu_yaw = None
        self.last_imu_yaw_filter_time = None
        self.gps_reject_count = 0
        self.gps_degraded_count = 0
        self.state_history = deque()
        self.imu_history = deque()

        self.pub = rospy.Publisher(self.output_odom_topic, Odometry, queue_size=20)
        self.tf_broadcaster = tf.TransformBroadcaster() if self.publish_tf else None

        self.gps_sub = rospy.Subscriber(
            self.gps_odom_topic, Odometry, self.gps_callback, queue_size=20
        )
        self.imu_sub = rospy.Subscriber(
            self.imu_topic, Imu, self.imu_callback, queue_size=50
        )

        rospy.loginfo(
            "[gps_imu_fusion] %s + %s -> %s mode=%s frame=%s child=%s gps_to_base=(%.3f, %.3f) fixed_lag=%s delay=%.3fs",
            self.gps_odom_topic,
            self.imu_topic,
            self.output_odom_topic,
            "ekf" if self.use_ekf else "complementary",
            self.frame_id,
            self.child_frame_id,
            self.gps_to_base_x,
            self.gps_to_base_y,
            self.use_fixed_lag_gps,
            self.gps_measurement_delay_s,
        )

    def msg_time(self, msg):
        if self.use_message_stamp and not is_zero_stamp(msg.header.stamp):
            return msg.header.stamp
        return rospy.Time.now()

    def predict_locked(self, stamp):
        if not self.initialized:
            return
        if self.last_predict_time is None:
            self.last_predict_time = stamp
            return

        dt = (stamp - self.last_predict_time).to_sec()
        if dt <= 0.0:
            return
        dt = min(dt, self.max_predict_dt_s)

        if self.use_imu_yaw_rate:
            self.yaw = normalize_angle(self.yaw + self.yaw_rate * dt)

        if self.use_speed_prediction:
            prediction_speed = self.speed * self.prediction_speed_scale
            self.x += prediction_speed * math.cos(self.yaw) * dt
            self.y += prediction_speed * math.sin(self.yaw) * dt

        self.last_predict_time = stamp

    def sync_state_from_ekf_locked(self):
        self.x = float(self.state[0])
        self.y = float(self.state[1])
        self.yaw = normalize_angle(float(self.state[2]))
        self.speed = float(self.state[3])
        self.yaw_rate = float(self.state[4])
        self.state[2] = self.yaw

    def record_state_snapshot_locked(self, stamp):
        if not (self.use_ekf and self.use_fixed_lag_gps and self.initialized):
            return
        stamp_s = stamp.to_sec()
        if self.state_history and stamp_s <= self.state_history[-1][0] + 1e-9:
            if abs(stamp_s - self.state_history[-1][0]) <= 1e-9:
                self.state_history[-1] = (stamp_s, self.state.copy(), self.P.copy())
            return
        self.state_history.append((stamp_s, self.state.copy(), self.P.copy()))

    def prune_fixed_lag_history_locked(self, now_s):
        oldest_s = now_s - self.fixed_lag_history_s
        while len(self.state_history) > 1 and self.state_history[1][0] < oldest_s:
            self.state_history.popleft()
        while self.imu_history and self.imu_history[0][0] < oldest_s:
            self.imu_history.popleft()

    def restore_state_snapshot_locked(self, snapshot):
        stamp_s, state, covariance = snapshot
        self.state = state.copy()
        self.P = covariance.copy()
        self.last_predict_time = rospy.Time.from_sec(stamp_s)
        self.sync_state_from_ekf_locked()

    def apply_ekf_covariance_floor_locked(self):
        if self.ekf_min_position_variance > 0.0:
            self.P[0, 0] = max(self.P[0, 0], self.ekf_min_position_variance)
            self.P[1, 1] = max(self.P[1, 1], self.ekf_min_position_variance)
        if self.ekf_min_yaw_variance > 0.0:
            self.P[2, 2] = max(self.P[2, 2], self.ekf_min_yaw_variance)
        if self.ekf_min_speed_variance > 0.0:
            self.P[3, 3] = max(self.P[3, 3], self.ekf_min_speed_variance)
        if self.ekf_min_yaw_rate_variance > 0.0:
            self.P[4, 4] = max(self.P[4, 4], self.ekf_min_yaw_rate_variance)

    def initialize_ekf_locked(self):
        self.state = np.array([self.x, self.y, self.yaw, self.speed, self.yaw_rate], dtype=float)
        self.P = np.eye(5) * self.ekf_initial_covariance
        self.P[0, 0] = self.ekf_gps_position_variance
        self.P[1, 1] = self.ekf_gps_position_variance
        self.P[2, 2] = self.ekf_imu_yaw_variance
        self.P[3, 3] = self.ekf_gps_speed_variance
        self.P[4, 4] = self.ekf_imu_yaw_rate_variance

    def ekf_predict_locked(self, stamp):
        if not self.initialized:
            return
        if self.last_predict_time is None:
            self.last_predict_time = stamp
            return

        dt = (stamp - self.last_predict_time).to_sec()
        if dt <= 0.0:
            return
        dt = min(dt, self.max_predict_dt_s)

        x, y, yaw, speed, yaw_rate = self.state
        prediction_speed = speed * self.prediction_speed_scale if self.use_speed_prediction else 0.0
        new_yaw = normalize_angle(yaw + yaw_rate * dt) if self.use_imu_yaw_rate else yaw

        self.state[0] = x + prediction_speed * math.cos(new_yaw) * dt
        self.state[1] = y + prediction_speed * math.sin(new_yaw) * dt
        self.state[2] = new_yaw
        self.state[3] = speed
        self.state[4] = yaw_rate

        F = np.eye(5)
        F[0, 2] = -prediction_speed * math.sin(new_yaw) * dt
        F[0, 3] = self.prediction_speed_scale * math.cos(new_yaw) * dt
        F[0, 4] = -prediction_speed * math.sin(new_yaw) * dt * dt
        F[1, 2] = prediction_speed * math.cos(new_yaw) * dt
        F[1, 3] = self.prediction_speed_scale * math.sin(new_yaw) * dt
        F[1, 4] = prediction_speed * math.cos(new_yaw) * dt * dt
        if self.use_imu_yaw_rate:
            F[2, 4] = dt

        q = np.diag(
            [
                self.ekf_process_noise_x,
                self.ekf_process_noise_y,
                self.ekf_process_noise_yaw,
                self.ekf_process_noise_speed,
                self.ekf_process_noise_yaw_rate,
            ]
        ) * dt
        self.P = F.dot(self.P).dot(F.T) + q
        self.apply_ekf_covariance_floor_locked()
        self.last_predict_time = stamp
        self.sync_state_from_ekf_locked()

    def ekf_update_locked(self, z, H, R, angle_index=None):
        z = np.array(z, dtype=float)
        expected = H.dot(self.state)
        innovation = z - expected
        if angle_index is not None:
            innovation[angle_index] = normalize_angle(innovation[angle_index])

        S = H.dot(self.P).dot(H.T) + R
        K = self.P.dot(H.T).dot(np.linalg.inv(S))
        self.state = self.state + K.dot(innovation)
        self.state[2] = normalize_angle(self.state[2])
        identity = np.eye(5)
        self.P = (identity - K.dot(H)).dot(self.P)
        self.apply_ekf_covariance_floor_locked()
        self.sync_state_from_ekf_locked()

    def ekf_update_gps_locked(self, gps_x, gps_y, gps_speed):
        H = np.zeros((3, 5))
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        H[2, 3] = 1.0
        R = np.diag(
            [
                self.ekf_gps_position_variance,
                self.ekf_gps_position_variance,
                self.ekf_gps_speed_variance,
            ]
        )
        self.ekf_update_locked([gps_x, gps_y, gps_speed], H, R)

    def ekf_update_gps_position_locked(self, gps_x, gps_y):
        H = np.zeros((2, 5))
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        R = np.diag([self.ekf_gps_position_variance, self.ekf_gps_position_variance])
        self.ekf_update_locked([gps_x, gps_y], H, R)

    def ekf_update_speed_locked(self, gps_speed):
        H = np.zeros((1, 5))
        H[0, 3] = 1.0
        R = np.array([[self.ekf_gps_speed_variance]])
        self.ekf_update_locked([gps_speed], H, R)

    def ekf_update_imu_locked(self, imu_yaw, yaw_rate):
        if self.use_imu_orientation and self.use_imu_yaw_rate:
            H = np.zeros((2, 5))
            H[0, 2] = 1.0
            H[1, 4] = 1.0
            R = np.diag([self.ekf_imu_yaw_variance, self.ekf_imu_yaw_rate_variance])
            self.ekf_update_locked([imu_yaw, yaw_rate], H, R, angle_index=0)
        elif self.use_imu_orientation:
            H = np.zeros((1, 5))
            H[0, 2] = 1.0
            R = np.array([[self.ekf_imu_yaw_variance]])
            self.ekf_update_locked([imu_yaw], H, R, angle_index=0)
        elif self.use_imu_yaw_rate:
            H = np.zeros((1, 5))
            H[0, 4] = 1.0
            R = np.array([[self.ekf_imu_yaw_rate_variance]])
            self.ekf_update_locked([yaw_rate], H, R)

    def apply_fixed_lag_gps_locked(self, gps_x, gps_y, gps_speed, arrival_stamp):
        if not self.use_fixed_lag_gps or self.gps_measurement_delay_s <= 0.0:
            return None
        if not self.state_history or self.last_predict_time is None:
            return None

        current_s = self.last_predict_time.to_sec()
        target_s = arrival_stamp.to_sec() - self.gps_measurement_delay_s
        target_s = min(target_s, current_s)
        base_snapshot = None
        for snapshot in reversed(self.state_history):
            if snapshot[0] <= target_s + 1e-9:
                base_snapshot = snapshot
                break
        if base_snapshot is None or current_s - target_s > self.fixed_lag_history_s:
            rospy.logwarn_throttle(
                1.0,
                "[gps_imu_fusion] fixed-lag history unavailable delay=%.3fs history=%d",
                current_s - target_s,
                len(self.state_history),
            )
            return None

        base_s = base_snapshot[0]
        replay_events = [
            event for event in self.imu_history if base_s < event[0] <= current_s + 1e-9
        ]
        while self.state_history and self.state_history[-1][0] > base_s + 1e-9:
            self.state_history.pop()
        self.restore_state_snapshot_locked(base_snapshot)

        event_index = 0
        while event_index < len(replay_events) and replay_events[event_index][0] <= target_s:
            event_s, imu_yaw, yaw_rate = replay_events[event_index]
            event_stamp = rospy.Time.from_sec(event_s)
            self.ekf_predict_locked(event_stamp)
            self.ekf_update_imu_locked(imu_yaw, yaw_rate)
            self.record_state_snapshot_locked(event_stamp)
            event_index += 1

        target_stamp = rospy.Time.from_sec(target_s)
        self.ekf_predict_locked(target_stamp)
        gps_base_x, gps_base_y = self.gps_observation_to_base_position(
            gps_x, gps_y, self.yaw
        )
        self.maybe_update_gps_course_yaw_locked(gps_base_x, gps_base_y, gps_speed)
        self.ekf_update_gps_position_locked(gps_base_x, gps_base_y)
        self.record_state_snapshot_locked(target_stamp)

        while event_index < len(replay_events):
            event_s, imu_yaw, yaw_rate = replay_events[event_index]
            event_stamp = rospy.Time.from_sec(event_s)
            self.ekf_predict_locked(event_stamp)
            self.ekf_update_imu_locked(imu_yaw, yaw_rate)
            self.record_state_snapshot_locked(event_stamp)
            event_index += 1

        current_stamp = rospy.Time.from_sec(current_s)
        self.ekf_predict_locked(current_stamp)
        # ll2utm takes speed from the current Vehicle Status packet, so do not delay it.
        self.ekf_update_speed_locked(gps_speed)
        self.record_state_snapshot_locked(current_stamp)
        self.prune_fixed_lag_history_locked(current_s)
        if self.debug_print:
            rospy.loginfo_throttle(
                2.0,
                "[gps_imu_fusion] fixed-lag GPS replay delay=%.3fs imu_events=%d",
                current_s - target_s,
                len(replay_events),
            )
        return gps_base_x, gps_base_y

    def maybe_update_gps_course_yaw_locked(self, gps_x, gps_y, gps_speed):
        if not self.use_ekf or not self.use_gps_course_yaw_update:
            return
        if self.last_gps_base_x is None or self.last_gps_base_y is None:
            return
        if abs(gps_speed) < self.gps_course_min_speed_mps:
            return
        if abs(self.yaw_rate) > self.gps_course_max_yaw_rate_rad_s:
            return

        dx = gps_x - self.last_gps_base_x
        dy = gps_y - self.last_gps_base_y
        distance = math.hypot(dx, dy)
        if distance < self.gps_course_min_distance_m:
            return

        course_yaw = math.atan2(dy, dx)
        yaw_innovation = normalize_angle(course_yaw - self.yaw)
        max_innovation = math.radians(self.gps_course_max_yaw_innovation_deg)
        if abs(yaw_innovation) > max_innovation:
            rospy.logwarn_throttle(
                1.0,
                "[gps_imu_fusion] skipping GPS course yaw innovation=%.2fdeg distance=%.2fm",
                math.degrees(yaw_innovation),
                distance,
            )
            return

        H = np.zeros((1, 5))
        H[0, 2] = 1.0
        R = np.array([[self.ekf_gps_course_yaw_variance]])
        self.ekf_update_locked([course_yaw], H, R, angle_index=0)

    def initial_yaw_from_sensors(self, gps_orientation):
        if self.last_imu_yaw is not None and self.use_imu_orientation:
            return self.last_imu_yaw
        return yaw_from_quaternion(gps_orientation)

    def initialize_state_locked(self, gps_x, gps_y, gps_speed, yaw, stamp):
        self.speed = gps_speed
        self.yaw = normalize_angle(yaw)
        self.x, self.y = self.gps_observation_to_base_position(gps_x, gps_y, self.yaw)
        self.last_gps_base_x = self.x
        self.last_gps_base_y = self.y
        if self.use_ekf:
            self.initialize_ekf_locked()
        self.startup_gps_samples = []
        self.startup_first_time = None
        self.last_predict_time = stamp
        self.initialized = True
        self.state_history.clear()
        self.imu_history.clear()
        self.record_state_snapshot_locked(stamp)

    def try_startup_initialize_locked(self, stamp, gps_x, gps_y, gps_speed, gps_orientation):
        sensor_yaw = self.initial_yaw_from_sensors(gps_orientation)
        if not self.use_ekf or not self.use_ekf_startup_alignment:
            self.initialize_state_locked(gps_x, gps_y, gps_speed, sensor_yaw, stamp)
            return True

        if self.startup_first_time is None:
            self.startup_first_time = stamp

        self.startup_gps_samples.append((gps_x, gps_y, stamp))
        if len(self.startup_gps_samples) > 10:
            self.startup_gps_samples.pop(0)

        elapsed = (stamp - self.startup_first_time).to_sec()
        can_use_course = (
            abs(gps_speed) >= self.ekf_startup_min_speed_mps
            and abs(self.yaw_rate) <= self.ekf_startup_max_yaw_rate_rad_s
            and len(self.startup_gps_samples) >= 2
        )

        if can_use_course:
            first_x, first_y, _ = self.startup_gps_samples[0]
            distance = math.hypot(gps_x - first_x, gps_y - first_y)
            if distance >= self.ekf_startup_min_distance_m:
                course_yaw = math.atan2(gps_y - first_y, gps_x - first_x)
                alpha = max(0.0, min(1.0, self.ekf_startup_course_yaw_alpha))
                startup_yaw = angle_lerp(sensor_yaw, course_yaw, alpha)
                self.initialize_state_locked(gps_x, gps_y, gps_speed, startup_yaw, stamp)
                rospy.loginfo(
                    "[gps_imu_fusion] EKF startup aligned yaw sensor=%.2fdeg course=%.2fdeg final=%.2fdeg distance=%.2fm",
                    math.degrees(sensor_yaw),
                    math.degrees(course_yaw),
                    math.degrees(startup_yaw),
                    distance,
                )
                return True

        if elapsed >= self.ekf_startup_max_wait_s:
            self.initialize_state_locked(gps_x, gps_y, gps_speed, sensor_yaw, stamp)
            rospy.loginfo(
                "[gps_imu_fusion] EKF startup fallback yaw=%.2fdeg elapsed=%.2fs speed=%.2fmps yaw_rate=%.3f",
                math.degrees(sensor_yaw),
                elapsed,
                gps_speed,
                self.yaw_rate,
            )
            return True

        return False

    def position_alpha_for_speed(self):
        if not self.use_speed_adaptive_position_alpha:
            return self.position_alpha

        speed = abs(self.speed)
        if speed < self.low_speed_threshold_mps:
            return self.position_alpha_low_speed
        if speed > self.high_speed_threshold_mps:
            return self.position_alpha_high_speed
        return self.position_alpha_mid_speed

    def gps_position_alpha(self, innovation):
        alpha = self.position_alpha_for_speed()
        if not self.enable_gps_outlier_rejection:
            return alpha, False, False

        if innovation >= self.gps_innovation_reject_m:
            return 0.0, True, False
        if innovation >= self.gps_innovation_warn_m:
            return alpha * self.gps_outlier_alpha_scale, False, True
        return alpha, False, False

    def gps_observation_to_base_position(self, gps_x, gps_y, yaw):
        if self.gps_to_base_x == 0.0 and self.gps_to_base_y == 0.0:
            return gps_x, gps_y

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        base_x = gps_x - (cos_yaw * self.gps_to_base_x - sin_yaw * self.gps_to_base_y)
        base_y = gps_y - (sin_yaw * self.gps_to_base_x + cos_yaw * self.gps_to_base_y)
        return base_x, base_y

    def gps_callback(self, msg):
        stamp = self.msg_time(msg)
        gps_x = msg.pose.pose.position.x
        gps_y = msg.pose.pose.position.y
        gps_speed = msg.twist.twist.linear.x * self.input_speed_scale

        with self.lock:
            if (
                not self.use_message_stamp
                and self.last_predict_time is not None
                and stamp < self.last_predict_time
            ):
                stamp = rospy.Time.now()
            if not self.initialized:
                if not self.try_startup_initialize_locked(
                    stamp, gps_x, gps_y, gps_speed, msg.pose.pose.orientation
                ):
                    self.last_gps_time = stamp
                    return
            else:
                if self.use_ekf:
                    self.ekf_predict_locked(stamp)
                    self.record_state_snapshot_locked(stamp)
                    self.prune_fixed_lag_history_locked(stamp.to_sec())
                    lagged_position = self.apply_fixed_lag_gps_locked(
                        gps_x, gps_y, gps_speed, stamp
                    )
                    if lagged_position is not None:
                        gps_x, gps_y = lagged_position
                    else:
                        if self.gps_delay_comp_s > 0.0:
                            comp_dt = min(self.gps_delay_comp_s, self.max_predict_dt_s)
                            comp_speed = gps_speed if gps_speed > 0.1 else self.speed
                            gps_x += comp_speed * math.cos(self.yaw) * comp_dt
                            gps_y += comp_speed * math.sin(self.yaw) * comp_dt
                        gps_x, gps_y = self.gps_observation_to_base_position(
                            gps_x, gps_y, self.yaw
                        )
                        self.maybe_update_gps_course_yaw_locked(gps_x, gps_y, gps_speed)
                        self.ekf_update_gps_locked(gps_x, gps_y, gps_speed)
                        self.record_state_snapshot_locked(stamp)
                else:
                    self.predict_locked(stamp)
                    if self.gps_delay_comp_s > 0.0:
                        comp_dt = min(self.gps_delay_comp_s, self.max_predict_dt_s)
                        comp_speed = gps_speed if gps_speed > 0.1 else self.speed
                        gps_x += comp_speed * math.cos(self.yaw) * comp_dt
                        gps_y += comp_speed * math.sin(self.yaw) * comp_dt
                    gps_x, gps_y = self.gps_observation_to_base_position(
                        gps_x, gps_y, self.yaw
                    )
                    innovation = math.hypot(gps_x - self.x, gps_y - self.y)
                    position_alpha, reject_gps, degraded_gps = self.gps_position_alpha(innovation)
                    if reject_gps:
                        self.gps_reject_count += 1
                        rospy.logwarn_throttle(
                            1.0,
                            "[gps_imu_fusion] rejecting GPS position innovation=%.3fm threshold=%.3fm count=%d",
                            innovation,
                            self.gps_innovation_reject_m,
                            self.gps_reject_count,
                        )
                    else:
                        if degraded_gps:
                            self.gps_degraded_count += 1
                            rospy.logwarn_throttle(
                                1.0,
                                "[gps_imu_fusion] down-weighting GPS position innovation=%.3fm alpha=%.3f count=%d",
                                innovation,
                                position_alpha,
                                self.gps_degraded_count,
                            )
                        self.x += position_alpha * (gps_x - self.x)
                        self.y += position_alpha * (gps_y - self.y)
                    self.speed += self.speed_alpha * (gps_speed - self.speed)
                    if self.gps_yaw_alpha > 0.0 and abs(self.speed) >= self.gps_yaw_min_speed_mps:
                        gps_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
                        max_step = math.radians(self.gps_yaw_max_step_deg)
                        self.yaw = limited_angle_lerp(
                            self.yaw, gps_yaw, self.gps_yaw_alpha, max_step
                        )

                self.last_gps_base_x = gps_x
                self.last_gps_base_y = gps_y

            self.last_gps_time = stamp

    def filtered_imu_yaw_locked(self, imu_yaw, stamp):
        if self.filtered_imu_yaw is None or self.last_imu_yaw_filter_time is None:
            self.filtered_imu_yaw = imu_yaw
            self.last_imu_yaw_filter_time = stamp
            return self.filtered_imu_yaw

        dt = (stamp - self.last_imu_yaw_filter_time).to_sec()
        if dt <= 0.0:
            return self.filtered_imu_yaw

        if self.imu_orientation_lpf_tau_s > 0.0:
            alpha = dt / (self.imu_orientation_lpf_tau_s + dt)
            alpha = max(0.0, min(1.0, alpha))
            self.filtered_imu_yaw = angle_lerp(self.filtered_imu_yaw, imu_yaw, alpha)
        else:
            self.filtered_imu_yaw = imu_yaw

        self.last_imu_yaw_filter_time = stamp
        return self.filtered_imu_yaw

    def imu_callback(self, msg):
        stamp = self.msg_time(msg)
        imu_yaw = normalize_angle(yaw_from_quaternion(msg.orientation) + self.imu_yaw_offset)
        yaw_rate = msg.angular_velocity.z - self.yaw_rate_bias
        if self.invert_yaw_rate:
            yaw_rate = -yaw_rate

        with self.lock:
            if (
                not self.use_message_stamp
                and self.last_predict_time is not None
                and stamp < self.last_predict_time
            ):
                stamp = rospy.Time.now()
            last_predict_time = self.last_predict_time
            if self.initialized:
                if self.use_ekf:
                    if self.use_fixed_lag_gps:
                        self.imu_history.append((stamp.to_sec(), imu_yaw, yaw_rate))
                    self.ekf_predict_locked(stamp)
                    self.ekf_update_imu_locked(imu_yaw, yaw_rate)
                    self.record_state_snapshot_locked(stamp)
                    self.prune_fixed_lag_history_locked(stamp.to_sec())
                else:
                    self.predict_locked(stamp)
                    if self.use_imu_orientation:
                        filtered_yaw = self.filtered_imu_yaw_locked(imu_yaw, stamp)
                        dt = 1.0 / self.rate_hz
                        if last_predict_time is not None:
                            measured_dt = (stamp - last_predict_time).to_sec()
                            if measured_dt > 0.0:
                                dt = min(measured_dt, self.max_predict_dt_s)
                        max_step = math.radians(self.imu_yaw_correction_max_rate_deg_s) * dt
                        self.yaw = limited_angle_lerp(
                            self.yaw, filtered_yaw, self.imu_yaw_alpha, max_step
                        )
                    if self.use_imu_yaw_rate:
                        self.yaw_rate = yaw_rate
            else:
                self.yaw_rate = yaw_rate

            self.last_imu_yaw = imu_yaw
            self.last_imu_time = stamp

    def healthy_locked(self, now):
        if not self.initialized or self.last_gps_time is None:
            return False
        if (now - self.last_gps_time).to_sec() > self.gps_timeout_s:
            return False
        if (self.use_imu_orientation or self.use_imu_yaw_rate) and self.last_imu_time is None:
            return False
        if self.last_imu_time is not None and (now - self.last_imu_time).to_sec() > self.imu_timeout_s:
            return False
        return True

    def build_odom_locked(self, stamp):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        q = tft.quaternion_from_euler(0.0, 0.0, self.yaw)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = self.speed
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.yaw_rate

        odom.pose.covariance = [0.0] * 36
        odom.pose.covariance[0] = self.P[0, 0] if self.use_ekf else self.position_variance
        odom.pose.covariance[7] = self.P[1, 1] if self.use_ekf else self.position_variance
        odom.pose.covariance[14] = 999.0
        odom.pose.covariance[21] = 999.0
        odom.pose.covariance[28] = 999.0
        odom.pose.covariance[35] = self.P[2, 2] if self.use_ekf else self.yaw_variance

        odom.twist.covariance = [0.0] * 36
        odom.twist.covariance[0] = self.P[3, 3] if self.use_ekf else self.speed_variance
        odom.twist.covariance[7] = 999.0
        odom.twist.covariance[14] = 999.0
        odom.twist.covariance[21] = 999.0
        odom.twist.covariance[28] = 999.0
        odom.twist.covariance[35] = self.P[4, 4] if self.use_ekf else self.yaw_variance
        return odom

    def publish_tf_msg(self, odom):
        self.tf_broadcaster.sendTransform(
            (
                odom.pose.pose.position.x,
                odom.pose.pose.position.y,
                odom.pose.pose.position.z,
            ),
            (
                odom.pose.pose.orientation.x,
                odom.pose.pose.orientation.y,
                odom.pose.pose.orientation.z,
                odom.pose.pose.orientation.w,
            ),
            odom.header.stamp,
            self.child_frame_id,
            self.frame_id,
        )

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            odom = None
            with self.lock:
                if self.healthy_locked(now):
                    if self.use_ekf:
                        self.ekf_predict_locked(now)
                        self.record_state_snapshot_locked(now)
                        self.prune_fixed_lag_history_locked(now.to_sec())
                    else:
                        self.predict_locked(now)
                    odom = self.build_odom_locked(now)
                    if self.debug_print:
                        rospy.loginfo_throttle(
                            1.0,
                            "[gps_imu_fusion] x=%.3f y=%.3f yaw=%.2fdeg speed=%.3f yaw_rate=%.4f",
                            self.x,
                            self.y,
                            math.degrees(self.yaw),
                            self.speed,
                            self.yaw_rate,
                        )
                else:
                    rospy.logwarn_throttle(
                        1.0,
                        "[gps_imu_fusion] waiting/stale input initialized=%s gps_age=%s imu_age=%s",
                        self.initialized,
                        "none" if self.last_gps_time is None else "%.3f" % (now - self.last_gps_time).to_sec(),
                        "none" if self.last_imu_time is None else "%.3f" % (now - self.last_imu_time).to_sec(),
                    )

            if odom is not None:
                self.pub.publish(odom)
                if self.tf_broadcaster is not None:
                    self.publish_tf_msg(odom)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("gps_imu_fusion")
    GpsImuFusionNode().spin()
