#!/usr/bin/env python3

import math
import threading

import rospy
from morai_msgs.msg import EgoVehicleStatus


class AccelerationTest:
    def __init__(self):
        self.ego_topic = rospy.get_param("~ego_topic", "/morai/ego_topic")
        self.speed_scale_to_kph = float(rospy.get_param("~speed_scale_to_kph", 1.0))
        self.accel_threshold = float(rospy.get_param("~accel_threshold", 0.9))
        self.maximum_start_speed_kph = float(
            rospy.get_param("~maximum_start_speed_kph", 1.0)
        )
        self.target_speed_kph = float(rospy.get_param("~target_speed_kph", 60.0))
        self.milestone_step_kph = float(rospy.get_param("~milestone_step_kph", 10.0))
        self.max_sample_step_m = float(rospy.get_param("~max_sample_step_m", 5.0))

        self.lock = threading.Lock()
        self.armed = False
        self.measuring = False
        self.saw_accel_released = False
        self.latest_sample = None
        self.test_count = 0

        self.start_sample = None
        self.previous_sample = None
        self.path_distance_m = 0.0
        self.minimum_accel = 1.0
        self.milestones = []
        self.next_milestone_index = 0
        self.results = []

        rospy.Subscriber(
            self.ego_topic,
            EgoVehicleStatus,
            self.ego_callback,
            queue_size=50,
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
            "accel": float(msg.accel),
        }

    def build_milestones(self):
        milestones = []
        speed = self.milestone_step_kph
        while speed < self.target_speed_kph - 1e-6:
            milestones.append(speed)
            speed += self.milestone_step_kph
        milestones.append(self.target_speed_kph)
        return milestones

    def arm(self):
        with self.lock:
            if self.latest_sample is None:
                rospy.logwarn("아직 %s 데이터가 없습니다.", self.ego_topic)
                return
            if self.measuring:
                rospy.logwarn("이미 가속 성능을 측정하고 있습니다.")
                return

            self.armed = True
            self.saw_accel_released = self.latest_sample["accel"] < self.accel_threshold
            speed = self.latest_sample["speed_kph"]
            if speed > self.maximum_start_speed_kph:
                rospy.logwarn(
                    "현재 속도가 %.2f km/h입니다. %.2f km/h 이하로 정지한 뒤 풀엑셀을 밟으세요.",
                    speed,
                    self.maximum_start_speed_kph,
                )
            elif self.saw_accel_released:
                rospy.loginfo(
                    "측정 대기: 정지 상태에서 풀엑셀을 밟고 %.0f km/h까지 유지하세요.",
                    self.target_speed_kph,
                )
            else:
                rospy.logwarn("엑셀을 먼저 놓아야 측정이 시작됩니다.")

    def start_measurement(self, sample):
        self.armed = False
        self.measuring = True
        self.start_sample = sample.copy()
        self.previous_sample = sample.copy()
        self.path_distance_m = 0.0
        self.minimum_accel = sample["accel"]
        self.milestones = self.build_milestones()
        self.next_milestone_index = 0
        self.results = []

        rospy.loginfo(
            "가속 측정 시작 | 속도 %.2f km/h | 위치 (%.3f, %.3f) | accel %.2f",
            sample["speed_kph"],
            sample["x"],
            sample["y"],
            sample["accel"],
        )

    def record_crossed_milestones(self, previous, sample, previous_distance, current_distance):
        speed_delta = sample["speed_kph"] - previous["speed_kph"]
        while self.next_milestone_index < len(self.milestones):
            target = self.milestones[self.next_milestone_index]
            if sample["speed_kph"] < target:
                break

            if abs(speed_delta) > 1e-6:
                ratio = (target - previous["speed_kph"]) / speed_delta
                ratio = max(0.0, min(1.0, ratio))
            else:
                ratio = 1.0

            sample_dt = (sample["stamp"] - previous["stamp"]).to_sec()
            stamp = previous["stamp"] + rospy.Duration(sample_dt * ratio)
            distance = previous_distance + (current_distance - previous_distance) * ratio
            elapsed = max(0.0, (stamp - self.start_sample["stamp"]).to_sec())
            average_accel = 0.0
            if distance > 1e-3:
                start_speed_mps = self.start_sample["speed_kph"] / 3.6
                target_speed_mps = target / 3.6
                average_accel = max(
                    0.0,
                    (target_speed_mps * target_speed_mps - start_speed_mps * start_speed_mps)
                    / (2.0 * distance),
                )

            self.results.append(
                {
                    "speed_kph": target,
                    "elapsed_s": elapsed,
                    "distance_m": distance,
                    "average_accel_mps2": average_accel,
                }
            )
            rospy.loginfo(
                "%.0f km/h 도달 | %.3f s | %.3f m",
                target,
                elapsed,
                distance,
            )
            self.next_milestone_index += 1

    def update_measurement(self, sample):
        previous = self.previous_sample
        previous_distance = self.path_distance_m
        step_distance = math.hypot(sample["x"] - previous["x"], sample["y"] - previous["y"])
        if step_distance <= self.max_sample_step_m:
            self.path_distance_m += step_distance
        else:
            rospy.logwarn(
                "위치가 한 번에 %.2fm 이동하여 해당 샘플을 거리 계산에서 제외했습니다.",
                step_distance,
            )

        self.minimum_accel = min(self.minimum_accel, sample["accel"])
        self.record_crossed_milestones(
            previous,
            sample,
            previous_distance,
            self.path_distance_m,
        )
        self.previous_sample = sample.copy()

        if self.next_milestone_index >= len(self.milestones):
            self.finish_measurement()

    def finish_measurement(self):
        self.test_count += 1
        rospy.loginfo("============================================================")
        rospy.loginfo("풀가속 측정 #%d 완료", self.test_count)
        rospy.loginfo(" 목표속도 | 누적시간(s) | 누적거리(m) | 평균가속도(m/s^2)")
        for result in self.results:
            rospy.loginfo(
                " %7.1f | %11.3f | %11.3f | %17.3f",
                result["speed_kph"],
                result["elapsed_s"],
                result["distance_m"],
                result["average_accel_mps2"],
            )
        rospy.loginfo("측정 중 최소 accel: %.2f", self.minimum_accel)
        if self.minimum_accel < self.accel_threshold:
            rospy.logwarn("목표속도 도달 전에 엑셀이 풀렸습니다. 풀가속 시험으로는 부정확합니다.")
        rospy.loginfo("============================================================")
        rospy.loginfo("다시 측정하려면 정지한 뒤 이 터미널에서 Enter를 누르세요.")

        self.measuring = False
        self.start_sample = None
        self.previous_sample = None
        self.results = []

    def ego_callback(self, msg):
        sample = self.make_sample(msg)
        with self.lock:
            self.latest_sample = sample

            if self.armed and not self.measuring:
                if sample["accel"] < self.accel_threshold:
                    self.saw_accel_released = True
                elif (
                    self.saw_accel_released
                    and sample["speed_kph"] <= self.maximum_start_speed_kph
                ):
                    self.start_measurement(sample)

            if self.measuring:
                self.update_measurement(sample)


def main():
    rospy.init_node("acceleration_test")
    tester = AccelerationTest()

    rospy.loginfo("풀가속 측정기 시작: %s 구독 중", tester.ego_topic)
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
