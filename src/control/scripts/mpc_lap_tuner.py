#!/usr/bin/env python3

from __future__ import annotations

import json
import math
import os
import random
import time
import csv
from typing import Dict, List, Optional, Tuple

import rospy
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64, String


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(value, upper))


class TunableParam:
    def __init__(self, config: Dict[str, object], namespace: str, min_positive: float) -> None:
        self.name = str(config["name"])
        self.param_name = self.name if self.name.startswith("/") else namespace.rstrip("/") + "/" + self.name
        self.min_value = float(config.get("min", min_positive))
        self.max_value = float(config.get("max", 1.0e6))
        self.learning_rate = float(config.get("learning_rate", config.get("lr", 0.10)))
        self.perturbation = float(config.get("perturbation", config.get("delta", 0.08)))
        self.min_positive = min_positive

        self.min_value = max(self.min_value, 0.0)
        self.max_value = max(self.max_value, self.min_value + min_positive)
        self.learning_rate = max(0.0, self.learning_rate)
        self.perturbation = max(1.0e-4, self.perturbation)

    def clamp_value(self, value: float) -> float:
        return clamp(value, self.min_value, self.max_value)

    def to_theta(self, value: float) -> float:
        return math.log(max(self.clamp_value(value), self.min_positive))

    def from_theta(self, theta: float) -> float:
        return self.clamp_value(math.exp(theta))

    @property
    def min_theta(self) -> float:
        return math.log(max(self.min_value, self.min_positive))

    @property
    def max_theta(self) -> float:
        return math.log(max(self.max_value, self.min_positive))


class LapMetrics:
    def __init__(self) -> None:
        self.reset(rospy.Time.now())

    def reset(self, stamp: rospy.Time) -> None:
        self.start_time = stamp
        self.cte_count = 0
        self.cte_abs_sum = 0.0
        self.cte_sq_sum = 0.0
        self.cte_abs_samples = []
        self.heading_count = 0
        self.heading_abs_sum = 0.0
        self.heading_sq_sum = 0.0
        self.penalty_count = 0
        self.penalty_sum = 0.0
        self.curvature_count = 0
        self.curvature_abs_sum = 0.0
        self.curve_cte_count = 0
        self.curve_cte_sum = 0.0
        self.curve_cte_curvature_abs_sum = 0.0
        self.curve_error_count = 0
        self.curve_error_abs_sum = 0.0
        self.curve_error_sq_sum = 0.0
        self.curve_error_abs_samples = []
        self.sharp_curve_count = 0
        self.sharp_curve_abs_sum = 0.0
        self.sharp_curve_sq_sum = 0.0
        self.sharp_curve_abs_samples = []
        self.left_curve_count = 0
        self.left_curve_sq_sum = 0.0
        self.right_curve_count = 0
        self.right_curve_sq_sum = 0.0
        self.straight_count = 0
        self.straight_sq_sum = 0.0


class MpcLapTuner:
    def __init__(self) -> None:
        self.odom_topic = rospy.get_param("~odom_topic", "/gps_utm_odom")
        self.global_path_topic = rospy.get_param("~global_path_topic", "/global_path1")
        self.cross_track_error_topic = rospy.get_param("~cross_track_error_topic", "/cross_track_error")
        self.heading_error_topic = rospy.get_param("~heading_error_topic", "/heading_error")
        self.curvature_topic = rospy.get_param("~curvature_topic", "/curvature")
        self.penalty_topic = rospy.get_param("~penalty_topic", "/penalty")
        self.mpc_param_namespace = rospy.get_param("~mpc_param_namespace", "/mpc_control")

        self.wrap_high_ratio = float(rospy.get_param("~wrap_high_ratio", 0.75))
        self.wrap_low_ratio = float(rospy.get_param("~wrap_low_ratio", 0.25))
        self.lap_wrap_direction = str(rospy.get_param("~lap_wrap_direction", "forward")).lower()
        self.min_lap_time_s = float(rospy.get_param("~min_lap_time_s", 10.0))
        self.min_samples_per_lap = int(rospy.get_param("~min_samples_per_lap", 80))
        self.nearest_search_window = int(rospy.get_param("~nearest_search_window", 500))
        self.max_nearest_dist_m = float(rospy.get_param("~max_nearest_dist_m", 30.0))

        self.initial_wait_laps = int(rospy.get_param("~initial_wait_laps", 1))
        self.loss_cte_weight = float(rospy.get_param("~loss_cte_weight", 1.0))
        self.loss_heading_deg_weight = float(rospy.get_param("~loss_heading_deg_weight", 0.02))
        self.loss_penalty_weight = float(rospy.get_param("~loss_penalty_weight", 1.0))
        self.loss_curve_cte_weight = float(rospy.get_param("~loss_curve_cte_weight", 0.70))
        self.loss_sharp_curve_cte_weight = float(rospy.get_param("~loss_sharp_curve_cte_weight", 0.80))
        self.loss_cte_p95_weight = float(rospy.get_param("~loss_cte_p95_weight", 0.75))
        self.loss_cte_max_weight = float(rospy.get_param("~loss_cte_max_weight", 0.20))
        self.loss_curve_cte_p95_weight = float(rospy.get_param("~loss_curve_cte_p95_weight", 0.80))
        self.loss_sharp_curve_cte_p95_weight = float(rospy.get_param("~loss_sharp_curve_cte_p95_weight", 1.10))
        self.loss_sharp_curve_cte_max_weight = float(rospy.get_param("~loss_sharp_curve_cte_max_weight", 0.35))
        self.loss_left_right_balance_weight = float(rospy.get_param("~loss_left_right_balance_weight", 0.20))
        self.loss_curve_bias_weight = float(rospy.get_param("~loss_curve_bias_weight", 0.20))
        self.curve_error_curvature_min = abs(float(rospy.get_param("~curve_error_curvature_min", 0.015)))
        self.sharp_curve_curvature_min = abs(float(rospy.get_param("~sharp_curve_curvature_min", 0.035)))
        self.min_segment_samples = int(rospy.get_param("~min_segment_samples", 20))
        self.learning_rate_decay = float(rospy.get_param("~learning_rate_decay", 0.06))
        self.perturbation_decay = float(rospy.get_param("~perturbation_decay", 0.03))
        self.max_log_step = float(rospy.get_param("~max_log_step", 0.18))
        self.rollback_on_bad_lap = bool(rospy.get_param("~rollback_on_bad_lap", True))
        self.bad_lap_loss_factor = float(rospy.get_param("~bad_lap_loss_factor", 3.0))
        self.min_positive_param = float(rospy.get_param("~min_positive_param", 1.0e-6))
        self.debug_publish_hz = max(0.0, float(rospy.get_param("~debug_publish_hz", 2.0)))

        random_seed = int(rospy.get_param("~random_seed", 0))
        if random_seed != 0:
            random.seed(random_seed)

        self.state_file = rospy.get_param("~state_file", "")
        self.restore_state = bool(rospy.get_param("~restore_state", False))
        self.summary_jsonl_file = rospy.get_param("~summary_jsonl_file", "/tmp/mpc_lap_tuner_laps.jsonl")
        self.summary_csv_file = rospy.get_param("~summary_csv_file", "/tmp/mpc_lap_tuner_laps.csv")
        self.summary_svg_file = rospy.get_param("~summary_svg_file", "/tmp/mpc_lap_tuner_summary.svg")
        self.best_txt_file = rospy.get_param("~best_txt_file", "/tmp/mpc_lap_tuner_best.txt")
        self.graph_max_laps = int(rospy.get_param("~graph_max_laps", 240))
        graph_param_names = rospy.get_param(
            "~graph_param_names",
            ["steer_model_gain_per_curvature", "weight_heading", "weight_cte", "weight_steer_rate"],
        )
        if isinstance(graph_param_names, str):
            graph_param_names = [name.strip() for name in graph_param_names.split(",") if name.strip()]
        if not isinstance(graph_param_names, list):
            graph_param_names = []
        self.graph_param_names = [str(name) for name in graph_param_names]

        self.tunables = self._load_tunables()
        self.theta: Dict[str, float] = {}
        self.active_theta: Dict[str, float] = {}
        self.center_theta: Dict[str, float] = {}
        self.best_theta: Dict[str, float] = {}
        self.best_loss = float("inf")

        self.phase = "center"
        self.iteration = 0
        self.completed_laps = 0
        self.delta_sign: Dict[str, float] = {}
        self.plus_loss: Optional[float] = None
        self.previous_summary: Optional[Dict[str, float]] = None
        self.lap_history: List[Dict[str, object]] = []

        self.path_points: List[Tuple[float, float]] = []
        self.cumulative_s: List[float] = []
        self.last_path_index: Optional[int] = None
        self.last_progress_ratio: Optional[float] = None
        self.last_curvature: Optional[float] = None
        self.last_curvature_stamp = rospy.Time(0)
        self.last_debug_publish_time = rospy.Time(0)
        self.metrics = LapMetrics()
        self.has_started_lap = False

        self._read_initial_theta()
        self.best_theta = dict(self.theta)
        self._restore_state_if_requested()
        self._load_existing_history()
        self._apply_theta(self.theta, "initial")
        self.active_theta = dict(self.theta)
        self.center_theta = dict(self.theta)

        self.status_pub = rospy.Publisher("~status", String, queue_size=10)
        self.lap_loss_pub = rospy.Publisher("~lap_loss", Float64, queue_size=10)
        self.debug_text_pub = rospy.Publisher("~debug_text", String, queue_size=10)

        self.path_sub = rospy.Subscriber(self.global_path_topic, Path, self.path_cb, queue_size=1)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=10)
        self.cte_sub = rospy.Subscriber(self.cross_track_error_topic, Float64, self.cte_cb, queue_size=100)
        self.heading_sub = rospy.Subscriber(self.heading_error_topic, Float64, self.heading_cb, queue_size=100)
        self.curvature_sub = rospy.Subscriber(self.curvature_topic, Float64, self.curvature_cb, queue_size=100)
        self.penalty_sub = rospy.Subscriber(self.penalty_topic, Float64, self.penalty_cb, queue_size=100)

        rospy.loginfo("[mpc_lap_tuner] watching path=%s odom=%s params=%s",
                      self.global_path_topic, self.odom_topic, self.mpc_param_namespace)
        rospy.loginfo("[mpc_lap_tuner] tunables: %s", ", ".join(p.name for p in self.tunables))
        rospy.loginfo("[mpc_lap_tuner] summary jsonl: %s", self.summary_jsonl_file or "disabled")
        rospy.loginfo("[mpc_lap_tuner] summary csv: %s", self.summary_csv_file or "disabled")
        rospy.loginfo("[mpc_lap_tuner] summary svg: %s", self.summary_svg_file or "disabled")
        rospy.loginfo("[mpc_lap_tuner] best txt: %s", self.best_txt_file or "disabled")

    def _load_tunables(self) -> List[TunableParam]:
        defaults = [
            {"name": "steer_model_gain_straight", "min": 0.70, "max": 1.30, "learning_rate": 0.04, "perturbation": 0.04},
            {"name": "steer_model_gain_per_curvature", "min": 0.10, "max": 30.0, "learning_rate": 0.16, "perturbation": 0.07},
            {"name": "weight_heading", "min": 2.0, "max": 300.0, "learning_rate": 0.10, "perturbation": 0.06},
            {"name": "weight_cte", "min": 20.0, "max": 800.0, "learning_rate": 0.09, "perturbation": 0.05},
            {"name": "weight_speed", "min": 2.0, "max": 120.0, "learning_rate": 0.05, "perturbation": 0.05},
            {"name": "weight_steer", "min": 0.02, "max": 3.0, "learning_rate": 0.05, "perturbation": 0.05},
            {"name": "weight_steer_rate", "min": 2.0, "max": 120.0, "learning_rate": 0.07, "perturbation": 0.05},
            {"name": "weight_steer_accel", "min": 5.0, "max": 300.0, "learning_rate": 0.07, "perturbation": 0.05},
            {"name": "weight_accel", "min": 0.01, "max": 5.0, "learning_rate": 0.04, "perturbation": 0.05},
            {"name": "weight_jerk", "min": 0.05, "max": 20.0, "learning_rate": 0.04, "perturbation": 0.05},
            {"name": "weight_cte_speed", "min": 0.10, "max": 30.0, "learning_rate": 0.05, "perturbation": 0.05},
            {"name": "terminal_weight", "min": 2.0, "max": 120.0, "learning_rate": 0.05, "perturbation": 0.05},
            {"name": "weight_speed_profile_slack", "min": 1000.0, "max": 80000.0, "learning_rate": 0.03, "perturbation": 0.04},
            {"name": "preview_time_s", "min": 0.60, "max": 2.40, "learning_rate": 0.05, "perturbation": 0.05},
            {"name": "preview_min_distance_m", "min": 2.0, "max": 10.0, "learning_rate": 0.05, "perturbation": 0.05},
            {"name": "preview_max_distance_m", "min": 10.0, "max": 35.0, "learning_rate": 0.05, "perturbation": 0.05},
            {"name": "path_resolution_min", "min": 0.20, "max": 1.20, "learning_rate": 0.03, "perturbation": 0.04},
            {"name": "curvature_smoothing_window_m", "min": 0.30, "max": 3.00, "learning_rate": 0.05, "perturbation": 0.05},
            {"name": "measurement_noise", "min": 0.004, "max": 0.080, "learning_rate": 0.03, "perturbation": 0.05},
            {"name": "process_noise", "min": 0.001, "max": 0.030, "learning_rate": 0.03, "perturbation": 0.05},
            {"name": "curve_speed_safety_factor", "min": 0.65, "max": 1.00, "learning_rate": 0.04, "perturbation": 0.04},
            {"name": "minimum_curve_speed_kph", "min": 2.0, "max": 12.0, "learning_rate": 0.03, "perturbation": 0.04},
            {"name": "decision_slow_down_speed_kph", "min": 10.0, "max": 35.0, "learning_rate": 0.03, "perturbation": 0.04},
            {"name": "speed_profile_tolerance_kph", "min": 0.001, "max": 4.0, "learning_rate": 0.03, "perturbation": 0.04},
            {"name": "low_speed_max_steer_rate_degps", "min": 35.0, "max": 85.0, "learning_rate": 0.04, "perturbation": 0.04},
            {"name": "high_speed_max_steer_rate_degps", "min": 10.0, "max": 30.0, "learning_rate": 0.04, "perturbation": 0.04},
            {"name": "low_speed_steering_time_constant", "min": 0.20, "max": 0.55, "learning_rate": 0.04, "perturbation": 0.04},
            {"name": "high_speed_steering_time_constant", "min": 0.65, "max": 1.45, "learning_rate": 0.04, "perturbation": 0.04},
        ]
        raw = rospy.get_param("~tunable_params", defaults)
        if not isinstance(raw, list):
            rospy.logwarn("[mpc_lap_tuner] ~tunable_params must be a list; using defaults")
            raw = defaults

        tunables: List[TunableParam] = []
        for item in raw:
            if not isinstance(item, dict) or "name" not in item:
                rospy.logwarn("[mpc_lap_tuner] skipping invalid tunable entry: %s", str(item))
                continue
            tunables.append(TunableParam(item, self.mpc_param_namespace, self.min_positive_param))

        if not tunables:
            raise RuntimeError("No valid tunable parameters configured")
        return tunables

    def _read_initial_theta(self) -> None:
        for param in self.tunables:
            if not rospy.has_param(param.param_name):
                raise RuntimeError("Missing MPC parameter: %s" % param.param_name)
            value = float(rospy.get_param(param.param_name))
            value = param.clamp_value(value)
            self.theta[param.name] = param.to_theta(value)

    def _restore_state_if_requested(self) -> None:
        if not self.restore_state or not self.state_file:
            return
        if not os.path.isfile(self.state_file):
            rospy.logwarn("[mpc_lap_tuner] restore requested but state file does not exist: %s", self.state_file)
            return
        try:
            with open(self.state_file, "r") as state_in:
                data = json.load(state_in)
        except (IOError, ValueError) as exc:
            rospy.logwarn("[mpc_lap_tuner] failed to read state file %s: %s", self.state_file, str(exc))
            return

        current_params = data.get("current_params", {})
        best_params = data.get("best_params", {})
        if not isinstance(current_params, dict):
            return

        for param in self.tunables:
            if param.name in current_params:
                self.theta[param.name] = param.to_theta(float(current_params[param.name]))
            if isinstance(best_params, dict) and param.name in best_params:
                self.best_theta[param.name] = param.to_theta(float(best_params[param.name]))
        self.best_loss = float(data.get("best_loss", self.best_loss))
        rospy.loginfo("[mpc_lap_tuner] restored state from %s", self.state_file)

    def _theta_to_values(self, theta: Dict[str, float]) -> Dict[str, float]:
        values = {}
        for param in self.tunables:
            values[param.name] = param.from_theta(theta[param.name])
        return values

    def _apply_theta(self, theta: Dict[str, float], reason: str) -> None:
        values = self._theta_to_values(theta)
        for param in self.tunables:
            rospy.set_param(param.param_name, values[param.name])
        self.active_theta = dict(theta)
        rospy.loginfo("[mpc_lap_tuner] apply %s params: %s",
                      reason,
                      ", ".join("%s=%.4g" % (name, value) for name, value in values.items()))
        self._save_state()

    def _save_state(self) -> None:
        if not self.state_file:
            return
        directory = os.path.dirname(self.state_file)
        if directory and not os.path.isdir(directory):
            try:
                os.makedirs(directory)
            except OSError:
                pass
        data = {
            "time": time.time(),
            "iteration": self.iteration,
            "phase": self.phase,
            "best_loss": self.best_loss,
            "current_params": self._theta_to_values(self.active_theta if self.active_theta else self.theta),
            "best_params": self._theta_to_values(self.best_theta if self.best_theta else self.theta),
        }
        try:
            with open(self.state_file, "w") as state_out:
                json.dump(data, state_out, indent=2, sort_keys=True)
        except IOError as exc:
            rospy.logwarn_throttle(5.0, "[mpc_lap_tuner] failed to write state: %s", str(exc))

    def path_cb(self, msg: Path) -> None:
        points = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        if len(points) < 3:
            rospy.logwarn("[mpc_lap_tuner] global path is too short: %d", len(points))
            return

        cumulative = [0.0]
        for idx in range(1, len(points)):
            prev = points[idx - 1]
            cur = points[idx]
            cumulative.append(cumulative[-1] + math.hypot(cur[0] - prev[0], cur[1] - prev[1]))

        self.path_points = points
        self.cumulative_s = cumulative
        self.last_path_index = None
        self.last_progress_ratio = None
        self.has_started_lap = False
        self.metrics.reset(rospy.Time.now())

        first = points[0]
        last = points[-1]
        close_dist = math.hypot(first[0] - last[0], first[1] - last[1])
        rospy.loginfo("[mpc_lap_tuner] loaded path points=%d length=%.1fm close_dist=%.2fm",
                      len(points), cumulative[-1], close_dist)

    def odom_cb(self, msg: Odometry) -> None:
        if len(self.path_points) < 3:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        nearest_index, nearest_dist = self._nearest_path_index(x, y)
        if nearest_dist > self.max_nearest_dist_m:
            rospy.logwarn_throttle(
                2.0,
                "[mpc_lap_tuner] nearest path point is %.1fm away; lap detection paused",
                nearest_dist,
            )
            return

        ratio = float(nearest_index) / float(max(1, len(self.path_points) - 1))
        now = msg.header.stamp if msg.header.stamp != rospy.Time(0) else rospy.Time.now()

        if not self.has_started_lap:
            self.metrics.reset(now)
            self.has_started_lap = True

        if self.last_progress_ratio is not None and self._is_lap_wrap(self.last_progress_ratio, ratio):
            self._finish_lap(now)
            self.metrics.reset(now)

        self.last_path_index = nearest_index
        self.last_progress_ratio = ratio
        self._maybe_publish_live_debug(now, ratio, nearest_dist)

    def _nearest_path_index(self, x: float, y: float) -> Tuple[int, float]:
        if self.last_path_index is None or self.nearest_search_window <= 0:
            return self._nearest_path_index_full(x, y)

        size = len(self.path_points)
        best_index = self.last_path_index
        best_dist_sq = float("inf")
        window = min(self.nearest_search_window, size)
        for offset in range(-window, window + 1):
            idx = (self.last_path_index + offset) % size
            px, py = self.path_points[idx]
            dist_sq = (px - x) * (px - x) + (py - y) * (py - y)
            if dist_sq < best_dist_sq:
                best_index = idx
                best_dist_sq = dist_sq

        if math.sqrt(best_dist_sq) <= self.max_nearest_dist_m:
            return best_index, math.sqrt(best_dist_sq)
        return self._nearest_path_index_full(x, y)

    def _nearest_path_index_full(self, x: float, y: float) -> Tuple[int, float]:
        best_index = 0
        best_dist_sq = float("inf")
        for idx, (px, py) in enumerate(self.path_points):
            dist_sq = (px - x) * (px - x) + (py - y) * (py - y)
            if dist_sq < best_dist_sq:
                best_index = idx
                best_dist_sq = dist_sq
        return best_index, math.sqrt(best_dist_sq)

    def _is_lap_wrap(self, previous_ratio: float, current_ratio: float) -> bool:
        if self.lap_wrap_direction == "reverse":
            return previous_ratio < self.wrap_low_ratio and current_ratio > self.wrap_high_ratio
        return previous_ratio > self.wrap_high_ratio and current_ratio < self.wrap_low_ratio

    def cte_cb(self, msg: Float64) -> None:
        value = float(msg.data)
        abs_value = abs(value)
        self.metrics.cte_count += 1
        self.metrics.cte_abs_sum += abs_value
        self.metrics.cte_sq_sum += value * value
        self.metrics.cte_abs_samples.append(abs_value)
        if self.last_curvature is not None:
            age = (rospy.Time.now() - self.last_curvature_stamp).to_sec()
            if age < 0.5:
                curvature = self.last_curvature
                abs_curvature = abs(curvature)
                self.metrics.curve_cte_count += 1
                self.metrics.curve_cte_sum += curvature * value
                self.metrics.curve_cte_curvature_abs_sum += abs_curvature

                if abs_curvature >= self.curve_error_curvature_min:
                    self.metrics.curve_error_count += 1
                    self.metrics.curve_error_abs_sum += abs_value
                    self.metrics.curve_error_sq_sum += value * value
                    self.metrics.curve_error_abs_samples.append(abs_value)

                    if abs_curvature >= self.sharp_curve_curvature_min:
                        self.metrics.sharp_curve_count += 1
                        self.metrics.sharp_curve_abs_sum += abs_value
                        self.metrics.sharp_curve_sq_sum += value * value
                        self.metrics.sharp_curve_abs_samples.append(abs_value)

                    if curvature >= 0.0:
                        self.metrics.left_curve_count += 1
                        self.metrics.left_curve_sq_sum += value * value
                    else:
                        self.metrics.right_curve_count += 1
                        self.metrics.right_curve_sq_sum += value * value
                else:
                    self.metrics.straight_count += 1
                    self.metrics.straight_sq_sum += value * value

    def heading_cb(self, msg: Float64) -> None:
        value = float(msg.data)
        self.metrics.heading_count += 1
        self.metrics.heading_abs_sum += abs(value)
        self.metrics.heading_sq_sum += value * value

    def curvature_cb(self, msg: Float64) -> None:
        value = float(msg.data)
        self.last_curvature = value
        self.last_curvature_stamp = rospy.Time.now()
        self.metrics.curvature_count += 1
        self.metrics.curvature_abs_sum += abs(value)

    def penalty_cb(self, msg: Float64) -> None:
        value = max(0.0, float(msg.data))
        self.metrics.penalty_count += 1
        self.metrics.penalty_sum += value

    def _finish_lap(self, stamp: rospy.Time) -> None:
        lap_time = max(0.0, (stamp - self.metrics.start_time).to_sec())
        min_count = min(self.metrics.cte_count, self.metrics.heading_count)
        if lap_time < self.min_lap_time_s or min_count < self.min_samples_per_lap:
            rospy.loginfo("[mpc_lap_tuner] skip partial lap: time=%.2fs samples=%d", lap_time, min_count)
            return

        summary = self._build_lap_summary(lap_time)
        self.completed_laps += 1

        loss_msg = Float64()
        loss_msg.data = summary["loss"]
        self.lap_loss_pub.publish(loss_msg)

        self._remember_best(summary["loss"], self.active_theta)
        self._publish_status(summary)
        self._publish_debug_text(summary, completed=True)
        self._append_summary_files(summary)
        rospy.loginfo(
            "[mpc_lap_tuner] lap=%d phase=%s loss=%.5f cte_rmse=%.3fm cte95=%.3fm cte_max=%.3fm "
            "heading_rmse=%.2fdeg curve95=%.3fm sharp95=%.3fm sharp_max=%.3fm lr_balance=%.3fm curve_bias=%.3fm "
            "penalty=%.3f time=%.2fs",
            self.completed_laps,
            self.phase,
            summary["loss"],
            summary["cte_rmse_m"],
            summary["cte_abs_p95_m"],
            summary["cte_abs_max_m"],
            summary["heading_rmse_deg"],
            summary["curve_cte_abs_p95_m"],
            summary["sharp_curve_cte_abs_p95_m"],
            summary["sharp_curve_cte_abs_max_m"],
            summary["left_right_curve_balance_m"],
            summary["curve_bias_m"],
            summary["penalty_mean"],
            lap_time,
        )

        self._optimizer_observe(summary["loss"])
        self.previous_summary = dict(summary)

    def _build_lap_summary(self, lap_time: float) -> Dict[str, float]:
        cte_count = max(1, self.metrics.cte_count)
        heading_count = max(1, self.metrics.heading_count)
        penalty_count = max(1, self.metrics.penalty_count)
        curve_cte_count = max(1, self.metrics.curve_cte_count)
        curve_error_count = max(1, self.metrics.curve_error_count)
        sharp_curve_count = max(1, self.metrics.sharp_curve_count)
        curvature_count = max(1, self.metrics.curvature_count)

        cte_rmse = math.sqrt(self.metrics.cte_sq_sum / cte_count)
        heading_rmse_rad = math.sqrt(self.metrics.heading_sq_sum / heading_count)
        heading_rmse_deg = heading_rmse_rad * 180.0 / math.pi
        penalty_mean = self.metrics.penalty_sum / penalty_count
        curve_cte_mean = self.metrics.curve_cte_sum / curve_cte_count
        curve_bias = self.metrics.curve_cte_sum / max(1.0e-9, self.metrics.curve_cte_curvature_abs_sum)
        cte_abs_p95 = self._percentile(self.metrics.cte_abs_samples, 0.95)
        cte_abs_max = max(self.metrics.cte_abs_samples) if self.metrics.cte_abs_samples else 0.0
        curve_cte_rmse = math.sqrt(self.metrics.curve_error_sq_sum / curve_error_count)
        curve_cte_abs_mean = self.metrics.curve_error_abs_sum / curve_error_count
        curve_cte_abs_p95 = self._percentile(self.metrics.curve_error_abs_samples, 0.95)
        curve_cte_abs_max = max(self.metrics.curve_error_abs_samples) if self.metrics.curve_error_abs_samples else 0.0
        sharp_curve_cte_rmse = math.sqrt(self.metrics.sharp_curve_sq_sum / sharp_curve_count)
        sharp_curve_cte_abs_mean = self.metrics.sharp_curve_abs_sum / sharp_curve_count
        sharp_curve_cte_abs_p95 = self._percentile(self.metrics.sharp_curve_abs_samples, 0.95)
        sharp_curve_cte_abs_max = max(self.metrics.sharp_curve_abs_samples) if self.metrics.sharp_curve_abs_samples else 0.0
        left_curve_cte_rmse = self._segment_rmse(self.metrics.left_curve_sq_sum, self.metrics.left_curve_count)
        right_curve_cte_rmse = self._segment_rmse(self.metrics.right_curve_sq_sum, self.metrics.right_curve_count)
        straight_cte_rmse = self._segment_rmse(self.metrics.straight_sq_sum, self.metrics.straight_count)
        left_right_curve_balance = 0.0
        if (self.metrics.left_curve_count >= self.min_segment_samples and
                self.metrics.right_curve_count >= self.min_segment_samples):
            left_right_curve_balance = abs(left_curve_cte_rmse - right_curve_cte_rmse)
        curvature_abs_mean = self.metrics.curvature_abs_sum / curvature_count
        loss = (
            self.loss_cte_weight * cte_rmse
            + self.loss_heading_deg_weight * heading_rmse_deg
            + self.loss_penalty_weight * penalty_mean
            + self.loss_curve_cte_weight * curve_cte_rmse
            + self.loss_sharp_curve_cte_weight * sharp_curve_cte_rmse
            + self.loss_cte_p95_weight * cte_abs_p95
            + self.loss_cte_max_weight * cte_abs_max
            + self.loss_curve_cte_p95_weight * curve_cte_abs_p95
            + self.loss_sharp_curve_cte_p95_weight * sharp_curve_cte_abs_p95
            + self.loss_sharp_curve_cte_max_weight * sharp_curve_cte_abs_max
            + self.loss_left_right_balance_weight * left_right_curve_balance
            + self.loss_curve_bias_weight * abs(curve_bias)
        )

        return {
            "lap": float(self.completed_laps + 1),
            "lap_time_s": lap_time,
            "loss": loss,
            "cte_rmse_m": cte_rmse,
            "cte_abs_mean_m": self.metrics.cte_abs_sum / cte_count,
            "cte_abs_p95_m": cte_abs_p95,
            "cte_abs_max_m": cte_abs_max,
            "heading_rmse_deg": heading_rmse_deg,
            "heading_abs_mean_deg": (self.metrics.heading_abs_sum / heading_count) * 180.0 / math.pi,
            "penalty_mean": penalty_mean,
            "curve_cte_mean": curve_cte_mean,
            "curve_bias_m": curve_bias,
            "curve_cte_rmse_m": curve_cte_rmse,
            "curve_cte_abs_mean_m": curve_cte_abs_mean,
            "curve_cte_abs_p95_m": curve_cte_abs_p95,
            "curve_cte_abs_max_m": curve_cte_abs_max,
            "sharp_curve_cte_rmse_m": sharp_curve_cte_rmse,
            "sharp_curve_cte_abs_mean_m": sharp_curve_cte_abs_mean,
            "sharp_curve_cte_abs_p95_m": sharp_curve_cte_abs_p95,
            "sharp_curve_cte_abs_max_m": sharp_curve_cte_abs_max,
            "left_curve_cte_rmse_m": left_curve_cte_rmse,
            "right_curve_cte_rmse_m": right_curve_cte_rmse,
            "left_right_curve_balance_m": left_right_curve_balance,
            "straight_cte_rmse_m": straight_cte_rmse,
            "curvature_abs_mean": curvature_abs_mean,
            "cte_samples": float(self.metrics.cte_count),
            "heading_samples": float(self.metrics.heading_count),
            "curve_samples": float(self.metrics.curve_error_count),
            "sharp_curve_samples": float(self.metrics.sharp_curve_count),
            "left_curve_samples": float(self.metrics.left_curve_count),
            "right_curve_samples": float(self.metrics.right_curve_count),
            "straight_samples": float(self.metrics.straight_count),
        }

    def _segment_rmse(self, square_sum: float, count: int) -> float:
        if count < self.min_segment_samples:
            return 0.0
        return math.sqrt(square_sum / float(max(1, count)))

    def _percentile(self, values: List[float], quantile: float) -> float:
        if not values:
            return 0.0
        sorted_values = sorted(values)
        if len(sorted_values) == 1:
            return sorted_values[0]
        q = clamp(quantile, 0.0, 1.0)
        position = q * (len(sorted_values) - 1)
        lower_index = int(math.floor(position))
        upper_index = int(math.ceil(position))
        if lower_index == upper_index:
            return sorted_values[lower_index]
        ratio = position - lower_index
        return sorted_values[lower_index] * (1.0 - ratio) + sorted_values[upper_index] * ratio

    def _publish_status(self, summary: Dict[str, float]) -> None:
        payload = dict(summary)
        payload["phase"] = self.phase
        payload["iteration"] = self.iteration
        payload["completed_laps"] = self.completed_laps
        payload["best_loss"] = self.best_loss
        payload["params"] = self._theta_to_values(self.active_theta)
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(msg)

    def _ensure_parent_dir(self, file_path: str) -> None:
        directory = os.path.dirname(file_path)
        if directory and not os.path.isdir(directory):
            os.makedirs(directory)

    def _load_existing_history(self) -> None:
        if not self.summary_jsonl_file or not os.path.isfile(self.summary_jsonl_file):
            return

        loaded: List[Dict[str, object]] = []
        try:
            with open(self.summary_jsonl_file, "r") as jsonl_in:
                for line in jsonl_in:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        record = json.loads(line)
                    except ValueError:
                        continue
                    if isinstance(record, dict) and self._finite_float(record.get("loss")) is not None:
                        loaded.append(record)
        except IOError as exc:
            rospy.logwarn("[mpc_lap_tuner] failed to read existing history: %s", str(exc))
            return

        if self.graph_max_laps > 0 and len(loaded) > self.graph_max_laps:
            loaded = loaded[-self.graph_max_laps:]
        self.lap_history = loaded
        if loaded:
            rospy.loginfo("[mpc_lap_tuner] loaded %d previous lap records for graphs", len(loaded))
            self._write_visual_summaries()

    def _summary_deltas(self, summary: Dict[str, float]) -> Dict[str, float]:
        deltas: Dict[str, float] = {}
        if self.previous_summary is None:
            return deltas

        for key in [
            "loss",
            "cte_rmse_m",
            "cte_abs_p95_m",
            "cte_abs_max_m",
            "heading_rmse_deg",
            "curve_cte_rmse_m",
            "curve_cte_abs_p95_m",
            "curve_cte_abs_max_m",
            "sharp_curve_cte_rmse_m",
            "sharp_curve_cte_abs_p95_m",
            "sharp_curve_cte_abs_max_m",
            "penalty_mean",
            "left_right_curve_balance_m",
            "curve_bias_m",
        ]:
            if key not in summary or key not in self.previous_summary:
                continue
            previous = self.previous_summary[key]
            delta = summary[key] - previous
            deltas["delta_" + key] = delta
            if abs(previous) > 1.0e-9:
                deltas["delta_pct_" + key] = 100.0 * delta / abs(previous)
        return deltas

    def _append_summary_files(self, summary: Dict[str, float]) -> None:
        params = self._theta_to_values(self.active_theta)
        deltas = self._summary_deltas(summary)
        record = dict(summary)
        record.update(deltas)
        record["wall_time"] = time.time()
        record["ros_time"] = rospy.Time.now().to_sec()
        record["phase"] = self.phase
        record["iteration"] = self.iteration
        record["completed_laps"] = self.completed_laps
        record["best_loss"] = self.best_loss
        record["params"] = params

        if self.summary_jsonl_file:
            try:
                self._ensure_parent_dir(self.summary_jsonl_file)
                with open(self.summary_jsonl_file, "a") as jsonl_out:
                    jsonl_out.write(json.dumps(record, sort_keys=True) + "\n")
            except IOError as exc:
                rospy.logwarn_throttle(5.0, "[mpc_lap_tuner] failed to write jsonl summary: %s", str(exc))

        if self.summary_csv_file:
            try:
                self._ensure_parent_dir(self.summary_csv_file)
                param_headers = ["param_" + param.name for param in self.tunables]
                headers = [
                    "wall_time",
                    "ros_time",
                    "lap",
                    "phase",
                    "iteration",
                    "completed_laps",
                    "best_loss",
                    "lap_time_s",
                    "loss",
                    "delta_loss",
                    "delta_pct_loss",
                    "cte_rmse_m",
                    "delta_cte_rmse_m",
                    "delta_pct_cte_rmse_m",
                    "cte_abs_p95_m",
                    "delta_cte_abs_p95_m",
                    "delta_pct_cte_abs_p95_m",
                    "cte_abs_max_m",
                    "delta_cte_abs_max_m",
                    "delta_pct_cte_abs_max_m",
                    "heading_rmse_deg",
                    "delta_heading_rmse_deg",
                    "delta_pct_heading_rmse_deg",
                    "curve_cte_rmse_m",
                    "curve_cte_abs_p95_m",
                    "delta_curve_cte_abs_p95_m",
                    "delta_pct_curve_cte_abs_p95_m",
                    "curve_cte_abs_max_m",
                    "delta_curve_cte_abs_max_m",
                    "delta_pct_curve_cte_abs_max_m",
                    "sharp_curve_cte_rmse_m",
                    "sharp_curve_cte_abs_p95_m",
                    "delta_sharp_curve_cte_abs_p95_m",
                    "delta_pct_sharp_curve_cte_abs_p95_m",
                    "sharp_curve_cte_abs_max_m",
                    "delta_sharp_curve_cte_abs_max_m",
                    "delta_pct_sharp_curve_cte_abs_max_m",
                    "left_curve_cte_rmse_m",
                    "right_curve_cte_rmse_m",
                    "left_right_curve_balance_m",
                    "curve_bias_m",
                    "penalty_mean",
                    "cte_samples",
                    "heading_samples",
                    "curve_samples",
                    "sharp_curve_samples",
                ] + param_headers
                csv_exists = os.path.isfile(self.summary_csv_file) and os.path.getsize(self.summary_csv_file) > 0
                csv_needs_header = not csv_exists
                if csv_exists:
                    try:
                        with open(self.summary_csv_file, "r", newline="") as csv_in:
                            first_row = next(csv.reader(csv_in), [])
                            csv_needs_header = first_row != headers
                    except (IOError, StopIteration):
                        csv_needs_header = True
                row = {key: record.get(key, "") for key in headers}
                for param in self.tunables:
                    row["param_" + param.name] = params.get(param.name, "")
                with open(self.summary_csv_file, "a", newline="") as csv_out:
                    writer = csv.DictWriter(csv_out, fieldnames=headers)
                    if csv_needs_header:
                        writer.writeheader()
                    writer.writerow(row)
            except IOError as exc:
                rospy.logwarn_throttle(5.0, "[mpc_lap_tuner] failed to write csv summary: %s", str(exc))

        self._remember_lap_record(record)

    def _remember_lap_record(self, record: Dict[str, object]) -> None:
        self.lap_history.append(record)
        if self.graph_max_laps > 0 and len(self.lap_history) > self.graph_max_laps:
            self.lap_history = self.lap_history[-self.graph_max_laps:]
        self._write_visual_summaries()

    def _write_visual_summaries(self) -> None:
        if self.best_txt_file:
            try:
                self._write_best_txt()
            except (IOError, OSError, ValueError) as exc:
                rospy.logwarn_throttle(5.0, "[mpc_lap_tuner] failed to write best txt: %s", str(exc))

        if self.summary_svg_file:
            try:
                self._write_summary_svg()
            except (IOError, OSError, ValueError) as exc:
                rospy.logwarn_throttle(5.0, "[mpc_lap_tuner] failed to write summary svg: %s", str(exc))

    def _write_best_txt(self) -> None:
        records = [record for record in self.lap_history if self._record_value(record, "loss") is not None]
        latest = records[-1] if records else None
        best_record = min(records, key=lambda record: self._record_value(record, "loss")) if records else None
        best_params = self._theta_to_values(self.best_theta if self.best_theta else self.theta)

        lines = [
            "MPC lap tuner summary",
            "updated_wall_time: %.3f" % time.time(),
            "recorded_laps: %d" % len(records),
            "best_state_loss: %s" % self._format_metric(self.best_loss),
        ]

        if latest is not None:
            lines += [
                "",
                "latest_lap:",
                "  lap: %s" % self._format_metric(self._record_value(latest, "lap")),
                "  phase: %s" % latest.get("phase", ""),
                "  loss: %s" % self._format_metric(self._record_value(latest, "loss")),
                "  cte_rmse_m: %s" % self._format_metric(self._record_value(latest, "cte_rmse_m")),
                "  cte_abs_p95_m: %s" % self._format_metric(self._record_value(latest, "cte_abs_p95_m")),
                "  cte_abs_max_m: %s" % self._format_metric(self._record_value(latest, "cte_abs_max_m")),
                "  curve_cte_abs_p95_m: %s" % self._format_metric(self._record_value(latest, "curve_cte_abs_p95_m")),
                "  sharp_curve_cte_abs_p95_m: %s" % self._format_metric(self._record_value(latest, "sharp_curve_cte_abs_p95_m")),
                "  sharp_curve_cte_abs_max_m: %s" % self._format_metric(self._record_value(latest, "sharp_curve_cte_abs_max_m")),
            ]

        if best_record is not None:
            lines += [
                "",
                "best_recorded_lap:",
                "  lap: %s" % self._format_metric(self._record_value(best_record, "lap")),
                "  phase: %s" % best_record.get("phase", ""),
                "  loss: %s" % self._format_metric(self._record_value(best_record, "loss")),
                "  cte_rmse_m: %s" % self._format_metric(self._record_value(best_record, "cte_rmse_m")),
                "  cte_abs_p95_m: %s" % self._format_metric(self._record_value(best_record, "cte_abs_p95_m")),
                "  cte_abs_max_m: %s" % self._format_metric(self._record_value(best_record, "cte_abs_max_m")),
                "  curve_cte_abs_p95_m: %s" % self._format_metric(self._record_value(best_record, "curve_cte_abs_p95_m")),
                "  sharp_curve_cte_abs_p95_m: %s" % self._format_metric(self._record_value(best_record, "sharp_curve_cte_abs_p95_m")),
                "  sharp_curve_cte_abs_max_m: %s" % self._format_metric(self._record_value(best_record, "sharp_curve_cte_abs_max_m")),
            ]

        best_metric_specs = [
            ("best_by_loss", "loss"),
            ("best_by_cte_rmse", "cte_rmse_m"),
            ("best_by_cte_p95", "cte_abs_p95_m"),
            ("best_by_cte_max", "cte_abs_max_m"),
            ("best_by_curve_p95", "curve_cte_abs_p95_m"),
            ("best_by_curve_max", "curve_cte_abs_max_m"),
            ("best_by_sharp_p95", "sharp_curve_cte_abs_p95_m"),
            ("best_by_sharp_max", "sharp_curve_cte_abs_max_m"),
        ]
        for title, metric_key in best_metric_specs:
            metric_records = [
                record for record in records
                if self._record_value(record, metric_key) is not None and isinstance(record.get("params"), dict)
            ]
            if not metric_records:
                continue
            metric_record = min(metric_records, key=lambda record: self._record_value(record, metric_key))
            lines += [
                "",
                title + ":",
                "  selected_metric: %s" % metric_key,
                "  selected_value: %s" % self._format_metric(self._record_value(metric_record, metric_key)),
                "  lap: %s" % self._format_metric(self._record_value(metric_record, "lap")),
                "  phase: %s" % metric_record.get("phase", ""),
                "  loss: %s" % self._format_metric(self._record_value(metric_record, "loss")),
                "  cte_rmse_m: %s" % self._format_metric(self._record_value(metric_record, "cte_rmse_m")),
                "  cte_abs_p95_m: %s" % self._format_metric(self._record_value(metric_record, "cte_abs_p95_m")),
                "  cte_abs_max_m: %s" % self._format_metric(self._record_value(metric_record, "cte_abs_max_m")),
                "  curve_cte_abs_p95_m: %s" % self._format_metric(self._record_value(metric_record, "curve_cte_abs_p95_m")),
                "  sharp_curve_cte_abs_p95_m: %s" % self._format_metric(self._record_value(metric_record, "sharp_curve_cte_abs_p95_m")),
                "  sharp_curve_cte_abs_max_m: %s" % self._format_metric(self._record_value(metric_record, "sharp_curve_cte_abs_max_m")),
                "  params:",
            ]
            metric_params = metric_record["params"]
            for param in self.tunables:
                value = self._finite_float(metric_params.get(param.name))
                lines.append("    %s: %s" % (param.name, self._format_metric(value)))

        lines += ["", "best_state_params:"]
        for param in self.tunables:
            lines.append("  %s: %.8g" % (param.name, best_params[param.name]))

        if latest is not None and isinstance(latest.get("params"), dict):
            lines += ["", "latest_params:"]
            latest_params = latest["params"]
            for param in self.tunables:
                value = self._finite_float(latest_params.get(param.name))
                lines.append("  %s: %s" % (param.name, self._format_metric(value)))

        lines += [
            "",
            "files:",
            "  jsonl: %s" % (self.summary_jsonl_file or "disabled"),
            "  csv: %s" % (self.summary_csv_file or "disabled"),
            "  svg: %s" % (self.summary_svg_file or "disabled"),
        ]

        self._ensure_parent_dir(self.best_txt_file)
        with open(self.best_txt_file, "w") as best_out:
            best_out.write("\n".join(lines) + "\n")

    def _write_summary_svg(self) -> None:
        records = [record for record in self.lap_history if self._record_value(record, "loss") is not None]
        width = 1180
        panel_height = 190
        gap = 18
        margin_top = 64

        param_series = []
        palette = ["#2563eb", "#dc2626", "#16a34a", "#9333ea", "#ea580c", "#0891b2", "#4f46e5"]
        for idx, name in enumerate(self.graph_param_names):
            key = "param." + name
            if any(self._record_value(record, key) is not None for record in records):
                param_series.append((key, self._short_param_label(name), palette[idx % len(palette)]))

        panels = [
            (
                "Loss",
                [("loss", "loss", "#111827"), ("best_loss", "best", "#16a34a")],
                False,
            ),
            (
                "Tracking error m",
                [
                    ("cte_rmse_m", "CTE rmse", "#2563eb"),
                    ("cte_abs_p95_m", "CTE95", "#dc2626"),
                    ("cte_abs_max_m", "CTE max", "#ea580c"),
                ],
                False,
            ),
            (
                "Curve peak error m",
                [
                    ("curve_cte_abs_p95_m", "curve95", "#7c3aed"),
                    ("curve_cte_abs_max_m", "curve max", "#9333ea"),
                    ("sharp_curve_cte_abs_p95_m", "sharp95", "#dc2626"),
                    ("sharp_curve_cte_abs_max_m", "sharp max", "#991b1b"),
                ],
                False,
            ),
        ]
        if param_series:
            panels.append(("Tuned params normalized", param_series, True))

        height = margin_top + len(panels) * panel_height + max(0, len(panels) - 1) * gap + 28
        svg = [
            '<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" viewBox="0 0 %d %d">' %
            (width, height, width, height),
            '<rect width="100%" height="100%" fill="#ffffff"/>',
            '<text x="24" y="32" font-family="Arial, sans-serif" font-size="22" font-weight="700" fill="#111827">MPC lap tuner summary</text>',
            '<text x="24" y="52" font-family="Arial, sans-serif" font-size="12" fill="#6b7280">records=%d, latest saved %.0f</text>' %
            (len(records), time.time()),
        ]

        top = margin_top
        for title, series_defs, normalized in panels:
            self._append_svg_panel(svg, records, width, top, panel_height, title, series_defs, normalized)
            top += panel_height + gap

        svg.append("</svg>")
        self._ensure_parent_dir(self.summary_svg_file)
        with open(self.summary_svg_file, "w") as svg_out:
            svg_out.write("\n".join(svg) + "\n")

    def _append_svg_panel(self,
                          svg: List[str],
                          records: List[Dict[str, object]],
                          width: int,
                          top: int,
                          panel_height: int,
                          title: str,
                          series_defs: List[Tuple[str, str, str]],
                          normalized: bool) -> None:
        left = 76
        right = 24
        plot_top = top + 58
        plot_height = panel_height - 78
        plot_width = width - left - right
        plot_bottom = plot_top + plot_height

        svg.append('<rect x="16" y="%d" width="%d" height="%d" rx="4" fill="#f9fafb" stroke="#e5e7eb"/>' %
                   (top, width - 32, panel_height))
        svg.append('<text x="28" y="%d" font-family="Arial, sans-serif" font-size="15" font-weight="700" fill="#111827">%s</text>' %
                   (top + 24, self._xml_escape(title)))

        if not records:
            svg.append('<text x="%d" y="%d" font-family="Arial, sans-serif" font-size="12" fill="#9ca3af">no data yet</text>' %
                       (left, plot_top + 28))
            return

        if normalized:
            y_min = 0.0
            y_max = 1.0
        else:
            values = []
            for key, _, _ in series_defs:
                for record in records:
                    value = self._record_value(record, key)
                    if value is not None:
                        values.append(value)
            if not values:
                svg.append('<text x="%d" y="%d" font-family="Arial, sans-serif" font-size="12" fill="#9ca3af">no data yet</text>' %
                           (left, plot_top + 28))
                return
            y_min = min(values)
            y_max = max(values)
            if y_min > 0.0:
                y_min = 0.0
            if abs(y_max - y_min) < 1.0e-9:
                pad = max(1.0, abs(y_max) * 0.10)
                y_min -= pad
                y_max += pad
            else:
                pad = 0.08 * (y_max - y_min)
                y_min -= pad
                y_max += pad

        for grid_idx in range(5):
            fraction = float(grid_idx) / 4.0
            y = plot_bottom - fraction * plot_height
            label_value = y_min + fraction * (y_max - y_min)
            svg.append('<line x1="%d" y1="%.2f" x2="%d" y2="%.2f" stroke="#e5e7eb" stroke-width="1"/>' %
                       (left, y, width - right, y))
            svg.append('<text x="%d" y="%.2f" font-family="Arial, sans-serif" font-size="10" text-anchor="end" fill="#6b7280">%s</text>' %
                       (left - 8, y + 3, self._xml_escape(self._format_metric(label_value))))

        svg.append('<line x1="%d" y1="%d" x2="%d" y2="%d" stroke="#9ca3af" stroke-width="1"/>' %
                   (left, plot_bottom, width - right, plot_bottom))
        svg.append('<text x="%d" y="%d" font-family="Arial, sans-serif" font-size="10" fill="#6b7280">1</text>' %
                   (left, plot_bottom + 15))
        svg.append('<text x="%d" y="%d" font-family="Arial, sans-serif" font-size="10" text-anchor="end" fill="#6b7280">%d</text>' %
                   (width - right, plot_bottom + 15, len(records)))

        legend_x = left
        legend_y = top + 42
        for key, label, color in series_defs:
            raw_values = [self._record_value(record, key) for record in records]
            values = [value for value in raw_values if value is not None]
            if not values:
                continue

            latest_value = values[-1]
            legend = "%s %s" % (label, self._format_metric(latest_value))
            estimated_width = 22 + 7 * len(legend)
            if legend_x + estimated_width > width - right:
                legend_x = left
                legend_y += 16

            svg.append('<line x1="%d" y1="%d" x2="%d" y2="%d" stroke="%s" stroke-width="2.5"/>' %
                       (legend_x, legend_y - 4, legend_x + 14, legend_y - 4, color))
            svg.append('<text x="%d" y="%d" font-family="Arial, sans-serif" font-size="11" fill="#374151">%s</text>' %
                       (legend_x + 18, legend_y, self._xml_escape(legend)))
            legend_x += estimated_width

            if normalized:
                series_min = min(values)
                series_max = max(values)
            else:
                series_min = y_min
                series_max = y_max

            points = []
            for idx, record in enumerate(records):
                value = self._record_value(record, key)
                if value is None:
                    continue
                x = left + (float(idx) / float(max(1, len(records) - 1))) * plot_width
                if abs(series_max - series_min) < 1.0e-9:
                    y_fraction = 0.5
                else:
                    y_fraction = clamp((value - series_min) / (series_max - series_min), 0.0, 1.0)
                y = plot_bottom - y_fraction * plot_height
                points.append((x, y))

            if len(points) >= 2:
                svg.append('<polyline fill="none" stroke="%s" stroke-width="2" points="%s"/>' %
                           (color, " ".join("%.2f,%.2f" % (x, y) for x, y in points)))
            for x, y in points[-min(18, len(points)):]:
                svg.append('<circle cx="%.2f" cy="%.2f" r="2.2" fill="%s"/>' % (x, y, color))

    def _record_value(self, record: Dict[str, object], key: str) -> Optional[float]:
        if key.startswith("param."):
            param_name = key.split(".", 1)[1]
            params = record.get("params")
            if isinstance(params, dict):
                return self._finite_float(params.get(param_name))
            return self._finite_float(record.get("param_" + param_name))
        return self._finite_float(record.get(key))

    def _finite_float(self, value: object) -> Optional[float]:
        try:
            result = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(result):
            return None
        return result

    def _format_metric(self, value: Optional[float]) -> str:
        if value is None or not math.isfinite(value):
            return "n/a"
        abs_value = abs(value)
        if abs_value >= 1000.0:
            return "%.0f" % value
        if abs_value >= 100.0:
            return "%.1f" % value
        if abs_value >= 10.0:
            return "%.2f" % value
        if abs_value >= 1.0:
            return "%.3f" % value
        return "%.4f" % value

    def _short_param_label(self, name: str) -> str:
        labels = {
            "steer_model_gain_per_curvature": "steer_gain/curv",
            "weight_heading": "heading_w",
            "weight_cte": "cte_w",
            "weight_steer_rate": "steer_rate_w",
            "weight_steer_accel": "steer_accel_w",
            "weight_steer": "steer_w",
            "terminal_weight": "terminal_w",
        }
        return labels.get(name, name)

    def _xml_escape(self, text: object) -> str:
        return str(text).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")

    def _maybe_publish_live_debug(self, now: rospy.Time, progress_ratio: float, nearest_dist: float) -> None:
        if self.debug_publish_hz <= 0.0 or not self.has_started_lap:
            return
        if (self.last_debug_publish_time != rospy.Time(0) and
                (now - self.last_debug_publish_time).to_sec() < 1.0 / self.debug_publish_hz):
            return

        elapsed = max(0.0, (now - self.metrics.start_time).to_sec())
        summary = self._build_lap_summary(elapsed)
        self._publish_debug_text(
            summary,
            completed=False,
            progress_ratio=progress_ratio,
            nearest_dist=nearest_dist,
        )

    def _publish_debug_text(self,
                            summary: Dict[str, float],
                            completed: bool,
                            progress_ratio: Optional[float] = None,
                            nearest_dist: Optional[float] = None) -> None:
        fields = [
            ("loss", "loss", "", 3),
            ("cte_rmse_m", "CTE", "m", 3),
            ("cte_abs_p95_m", "CTE95", "m", 3),
            ("cte_abs_max_m", "CTEmax", "m", 3),
            ("heading_rmse_deg", "heading", "deg", 2),
            ("curve_cte_abs_p95_m", "curve95", "m", 3),
            ("sharp_curve_cte_abs_p95_m", "sharp95", "m", 3),
            ("sharp_curve_cte_abs_max_m", "sharpMax", "m", 3),
            ("penalty_mean", "penalty", "", 3),
        ]

        if completed:
            parts = [
                "lap %d done" % self.completed_laps,
                "phase=%s" % self.phase,
                "best=%.3f" % self.best_loss if math.isfinite(self.best_loss) else "best=none",
            ]
        else:
            parts = [
                "lap %d running" % (self.completed_laps + 1),
                "progress=%.1f%%" % (100.0 * clamp(progress_ratio if progress_ratio is not None else 0.0, 0.0, 1.0)),
                "elapsed=%.1fs" % summary["lap_time_s"],
                "phase=%s" % self.phase,
            ]
            if nearest_dist is not None:
                parts.append("path_dist=%.2fm" % nearest_dist)

        improved = []
        worsened = []

        for key, label, unit, decimals in fields:
            value = summary[key]
            text = "%s=%.*f%s" % (label, decimals, value, unit)
            if completed and self.previous_summary is not None and key in self.previous_summary:
                previous = self.previous_summary[key]
                delta = value - previous
                text += " (%+.*f%s" % (decimals, delta, unit)
                if abs(previous) > 1.0e-9:
                    text += ", %+0.1f%%" % (100.0 * delta / abs(previous))
                text += ")"
                if delta < -1.0e-6:
                    improved.append(label)
                elif delta > 1.0e-6:
                    worsened.append(label)
            parts.append(text)

        if not completed:
            parts.append("samples=%d" % int(summary["cte_samples"]))
            if self.previous_summary is not None:
                parts.append(
                    "prev_lap CTE=%.3fm CTE95=%.3fm heading=%.2fdeg loss=%.3f" %
                    (self.previous_summary["cte_rmse_m"],
                     self.previous_summary["cte_abs_p95_m"],
                     self.previous_summary["heading_rmse_deg"],
                     self.previous_summary["loss"])
                )
        elif self.previous_summary is None:
            parts.append("prev=none")
        else:
            if improved:
                parts.append("down=" + ",".join(improved))
            if worsened:
                parts.append("up=" + ",".join(worsened))

        msg = String()
        msg.data = " | ".join(parts)
        self.debug_text_pub.publish(msg)
        self.last_debug_publish_time = rospy.Time.now()

    def _remember_best(self, loss: float, theta: Dict[str, float]) -> None:
        if loss < self.best_loss:
            self.best_loss = loss
            self.best_theta = dict(theta)
            rospy.loginfo("[mpc_lap_tuner] new best loss %.5f", loss)
            self._save_state()

    def _optimizer_observe(self, loss: float) -> None:
        if self.completed_laps <= self.initial_wait_laps:
            rospy.loginfo("[mpc_lap_tuner] warmup lap %d/%d; no update",
                          self.completed_laps, self.initial_wait_laps)
            if self.completed_laps == self.initial_wait_laps:
                self._start_spsa_pair()
            return

        if (
            self.rollback_on_bad_lap
            and math.isfinite(self.best_loss)
            and self.best_loss > 0.0
            and loss > self.best_loss * self.bad_lap_loss_factor
        ):
            rospy.logwarn("[mpc_lap_tuner] bad lap loss %.5f > %.2fx best; rolling back",
                          loss, self.bad_lap_loss_factor)
            self.theta = dict(self.best_theta)
            self.phase = "center"
            self.plus_loss = None
            self._apply_theta(self.theta, "rollback")
            self._start_spsa_pair()
            return

        if self.phase == "plus":
            self.plus_loss = loss
            minus_theta = self._perturbed_theta(self.center_theta, -1.0)
            self.phase = "minus"
            self._apply_theta(minus_theta, "spsa-minus")
            return

        if self.phase == "minus" and self.plus_loss is not None:
            self._finish_spsa_pair(self.plus_loss, loss)
            self._start_spsa_pair()
            return

        self._start_spsa_pair()

    def _start_spsa_pair(self) -> None:
        self.iteration += 1
        self.phase = "plus"
        self.center_theta = dict(self.theta)
        self.delta_sign = {param.name: random.choice([-1.0, 1.0]) for param in self.tunables}
        self.plus_loss = None
        plus_theta = self._perturbed_theta(self.center_theta, 1.0)
        self._apply_theta(plus_theta, "spsa-plus")

    def _perturbed_theta(self, center: Dict[str, float], direction: float) -> Dict[str, float]:
        decay = 1.0 / (1.0 + self.perturbation_decay * max(0, self.iteration - 1))
        theta = {}
        for param in self.tunables:
            sign = self.delta_sign.get(param.name, 1.0)
            value = center[param.name] + direction * param.perturbation * decay * sign
            theta[param.name] = clamp(value, param.min_theta, param.max_theta)
        return theta

    def _finish_spsa_pair(self, plus_loss: float, minus_loss: float) -> None:
        lr_decay = 1.0 / (1.0 + self.learning_rate_decay * max(0, self.iteration - 1))
        perturb_decay = 1.0 / (1.0 + self.perturbation_decay * max(0, self.iteration - 1))
        new_theta = dict(self.center_theta)
        updates = []

        for param in self.tunables:
            sign = self.delta_sign.get(param.name, 1.0)
            c_i = param.perturbation * perturb_decay
            gradient = (plus_loss - minus_loss) / max(1.0e-9, 2.0 * c_i * sign)
            raw_step = -param.learning_rate * lr_decay * gradient
            step = clamp(raw_step, -self.max_log_step, self.max_log_step)
            new_theta[param.name] = clamp(self.center_theta[param.name] + step, param.min_theta, param.max_theta)
            updates.append("%s:%+.4f" % (param.name, step))

        self.theta = new_theta
        self.phase = "center"
        self.plus_loss = None
        rospy.loginfo("[mpc_lap_tuner] SPSA update iter=%d plus=%.5f minus=%.5f steps=[%s]",
                      self.iteration, plus_loss, minus_loss, ", ".join(updates))
        self._apply_theta(self.theta, "spsa-update")


def main() -> None:
    rospy.init_node("mpc_lap_tuner")
    try:
        _ = MpcLapTuner()
    except Exception as exc:
        rospy.logerr("[mpc_lap_tuner] failed to start: %s", str(exc))
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
