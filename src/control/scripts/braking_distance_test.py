#!/usr/bin/env python3

import math
import threading

import rospy
from morai_msgs.msg import EgoVehicleStatus


class BrakingDistanceTest:
    def __init__(self):
        self.ego_topic = rospy.get_param("~ego_topic", "/morai/ego_topic")
        self.speed_scale_to_kph = float(rospy.get_param("~speed_scale_to_kph", 1.0))
        self.brake_threshold = float(rospy.get_param("~brake_threshold", 0.9))
        self.minimum_start_speed_kph = float(
            rospy.get_param("~minimum_start_speed_kph", 5.0)
        )
        self.stop_speed_kph = float(rospy.get_param("~stop_speed_kph", 0.2))
        self.stop_hold_sec = float(rospy.get_param("~stop_hold_sec", 0.5))
        self.max_sample_step_m = float(rospy.get_param("~max_sample_step_m", 5.0))

        self.lock = threading.Lock()
        self.armed = False
        self.measuring = False
        self.saw_brake_released = False
        self.latest_sample = None
        self.test_count = 0

        self.start_sample = None
        self.previous_sample = None
        self.path_distance_m = 0.0
        self.minimum_brake = 1.0
        self.stop_candidate = None

        rospy.Subscriber(
            self.ego_topic,
            EgoVehicleStatus,
            self.ego_callback,
            queue_size=20,
        )

    @staticmethod
    def message_stamp(msg):
        if msg.header.stamp != rospy.Time(0):
            return msg.header.stamp
        return rospy.Time.now()

    def make_sample(self, msg):
        raw_speed = math.hypot(msg.velocity.x, msg.velocity.y)
        return {
            "stamp": self.message_stamp(msg),
            "x": float(msg.position.x),
            "y": float(msg.position.y),
            "speed_kph": raw_speed * self.speed_scale_to_kph,
            "brake": float(msg.brake),
        }

    def arm(self):
        with self.lock:
            if self.latest_sample is None:
                rospy.logwarn("아직 %s 데이터가 없습니다.", self.ego_topic)
                return
            if self.measuring:
                rospy.logwarn("이미 제동거리를 측정하고 있습니다.")
                return

            self.armed = True
            self.saw_brake_released = self.latest_sample["brake"] < self.brake_threshold
            speed = self.latest_sample["speed_kph"]
            if self.saw_brake_released:
                rospy.loginfo(
                    "측정 대기: %.1f km/h 이상에서 풀브레이크를 밟으세요.",
                    self.minimum_start_speed_kph,
                )
            else:
                rospy.logwarn("브레이크를 먼저 놓아야 측정이 시작됩니다.")
            rospy.loginfo("현재 속도: %.2f km/h", speed)

    def start_measurement(self, sample):
        self.armed = False
        self.measuring = True
        self.start_sample = sample.copy()
        self.previous_sample = sample.copy()
        self.path_distance_m = 0.0
        self.minimum_brake = sample["brake"]
        self.stop_candidate = None

        rospy.loginfo(
            "제동 시작 | 속도 %.2f km/h | 위치 (%.3f, %.3f) | brake %.2f",
            sample["speed_kph"],
            sample["x"],
            sample["y"],
            sample["brake"],
        )

    def update_measurement(self, sample):
        dx = sample["x"] - self.previous_sample["x"]
        dy = sample["y"] - self.previous_sample["y"]
        step_distance = math.hypot(dx, dy)
        if step_distance <= self.max_sample_step_m:
            self.path_distance_m += step_distance
        else:
            rospy.logwarn(
                "위치가 한 번에 %.2fm 이동하여 해당 샘플을 거리 계산에서 제외했습니다.",
                step_distance,
            )
        self.previous_sample = sample.copy()
        # 정지 판정 후의 hold 시간에는 운전자가 브레이크를 놓아도 시험은 이미 끝난
        # 상태이므로, 실제 제동 구간에서만 최소 브레이크 입력을 기록한다.
        if self.stop_candidate is None:
            self.minimum_brake = min(self.minimum_brake, sample["brake"])

        if sample["speed_kph"] <= self.stop_speed_kph:
            if self.stop_candidate is None:
                self.stop_candidate = {
                    "sample": sample.copy(),
                    "path_distance_m": self.path_distance_m,
                }
            stopped_for = (sample["stamp"] - self.stop_candidate["sample"]["stamp"]).to_sec()
            if stopped_for >= self.stop_hold_sec:
                self.finish_measurement()
        else:
            self.stop_candidate = None

    def finish_measurement(self):
        end = self.stop_candidate["sample"]
        start = self.start_sample
        path_distance = self.stop_candidate["path_distance_m"]
        straight_distance = math.hypot(end["x"] - start["x"], end["y"] - start["y"])
        stop_time = max(0.0, (end["stamp"] - start["stamp"]).to_sec())
        start_speed_mps = start["speed_kph"] / 3.6
        average_decel = (
            start_speed_mps * start_speed_mps / (2.0 * path_distance)
            if path_distance > 1e-3
            else 0.0
        )

        self.test_count += 1
        rospy.loginfo("============================================================")
        rospy.loginfo("제동거리 측정 #%d 완료", self.test_count)
        rospy.loginfo("시작 속도      : %.2f km/h", start["speed_kph"])
        rospy.loginfo("주행 제동거리  : %.3f m", path_distance)
        rospy.loginfo("시작-정지 직선거리: %.3f m", straight_distance)
        rospy.loginfo("정지 시간      : %.3f s", stop_time)
        rospy.loginfo("평균 감속도    : %.3f m/s^2", average_decel)
        rospy.loginfo("측정 중 최소 brake: %.2f", self.minimum_brake)
        if self.minimum_brake < self.brake_threshold:
            rospy.logwarn("정지 전에 브레이크가 풀렸습니다. 풀브레이크 시험으로는 부정확할 수 있습니다.")
        rospy.loginfo("============================================================")
        rospy.loginfo("다시 측정하려면 이 터미널에서 Enter를 누르세요.")

        self.measuring = False
        self.start_sample = None
        self.previous_sample = None
        self.stop_candidate = None

    def ego_callback(self, msg):
        sample = self.make_sample(msg)
        with self.lock:
            self.latest_sample = sample

            if self.armed and not self.measuring:
                if sample["brake"] < self.brake_threshold:
                    self.saw_brake_released = True
                elif (
                    self.saw_brake_released
                    and sample["speed_kph"] >= self.minimum_start_speed_kph
                ):
                    self.start_measurement(sample)

            if self.measuring:
                self.update_measurement(sample)


def main():
    rospy.init_node("braking_distance_test")
    tester = BrakingDistanceTest()

    rospy.loginfo("브레이크 거리 측정기 시작: %s 구독 중", tester.ego_topic)
    rospy.loginfo("Enter: 측정 대기 | q + Enter: 종료")

    while not rospy.is_shutdown():
        try:
            command = input().strip().lower()
        except (EOFError, KeyboardInterrupt):
            break
        if command == "q":
            break
        tester.arm()


if __name__ == "__main__":
    main()
