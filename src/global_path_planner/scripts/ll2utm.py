#!/usr/bin/env python3
 
import rospy
import numpy as np
import tf

from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from math import atan2, cos, sin, sqrt

# MORAI GPS 메시지: 이 노드는 GPSMessage만 위치 입력으로 사용한다.
from morai_msgs.msg import GPSMessage


# =========================
# UTM 계수 계산
# =========================
# 아래 두 함수는 기존 UTM 변환 수식에서 사용하는 계수 계산부다.
# localization 로직과 독립적이므로 수식 구조를 그대로 유지한다.
def proj_coef_0(e):
    c0_transverse_mercator = np.array([
        [ -175 / 16384.0, 0.0,  -5 / 2560.0, 0.0, -3 / 64.0 , 0.0, -1 / 4.0, 0.0, 1.0],
        [ -105 / 40960.0, 0.0, -45 / 1024.0, 0.0, -3 / 32.0 , 0.0, -3 / 8.0, 0.0, 0.0],
        [  525 / 16384.0, 0.0,  45 / 1024.0, 0.0, 15 / 256.0, 0.0,      0.0, 0.0, 0.0],
        [ -175 / 12288.0, 0.0, -35 / 3072.0, 0.0,        0.0, 0.0,      0.0, 0.0, 0.0],
        [ 315 / 131072.0, 0.0,          0.0, 0.0,        0.0, 0.0,      0.0, 0.0, 0.0]
    ])

    c_out = np.zeros(5)

    for i in range(0,5):
        c_out[i] = np.poly1d(c0_transverse_mercator[i,:])(e)

    return c_out


def proj_coef_2(e):
    c0_merdian_arc = np.array([
        [ -175 / 16384.0    , 0.0, -5 / 256.0  , 0.0,  -3 / 64.0, 0.0, -1 / 4.0, 0.0, 1.0 ],
        [ -901 / 184320.0   , 0.0, -9 / 1024.0 , 0.0,  -1 / 96.0, 0.0,  1 / 8.0, 0.0, 0.0 ],
        [ -311 / 737280.0   , 0.0, 17 / 5120.0 , 0.0, 13 / 768.0, 0.0,      0.0, 0.0, 0.0 ],
        [ 899 / 430080.0    , 0.0, 61 / 15360.0, 0.0,        0.0, 0.0,      0.0, 0.0, 0.0 ],
        [ 49561 / 41287680.0, 0.0,          0.0, 0.0,        0.0, 0.0,      0.0, 0.0, 0.0 ]
    ])

    c_out = np.zeros(5)

    for i in range(0,5):
        c_out[i] = np.poly1d(c0_merdian_arc[i,:])(e)

    return c_out


class LocationSensor:
    def __init__(self, zone=52):
        # =========================
        # ROS 파라미터
        # =========================
        # ROS 파라미터: 토픽/frame/UTM offset과 GPS-only 추정기 튜닝값을 읽는다.
        self.gps_topic = rospy.get_param("~gps_topic", "/gps")
        self.odom_topic = rospy.get_param("~odom_topic", "/gps_utm_odom")
        self.map_frame_id = rospy.get_param("~map_frame_id", "map")
        self.base_frame_id = rospy.get_param("~base_frame_id", "base_link")
        self.zone = int(rospy.get_param("~utm_zone", zone))
        self.kcity_east_offset = 302459.942
        self.kcity_north_offset = 4122635.537
        self.east_offset = float(rospy.get_param("~east_offset", 0.0))
        self.north_offset = float(rospy.get_param("~north_offset", 0.0))
        self.use_msg_offset = rospy.get_param("~use_msg_offset", False)
        self.require_valid_status = rospy.get_param("~require_valid_status", False)
        self.min_gps_status = int(rospy.get_param("~min_gps_status", 0))
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 50.0))
        self.enable_prediction = rospy.get_param("~enable_prediction", True)
        self.max_prediction_time_s = max(0.0, float(rospy.get_param("~max_prediction_time_s", 0.10)))
        self.gps_timeout_s = float(rospy.get_param("~gps_timeout_s", 0.5))
        self.heading_filter_alpha = min(1.0, max(0.0, float(rospy.get_param("~heading_filter_alpha", 1.0))))
        self.velocity_filter_alpha = min(1.0, max(0.0, float(rospy.get_param("~velocity_filter_alpha", 1.0))))
        self.initial_heading_distance = float(rospy.get_param("~initial_heading_distance", 0.02))
        self.min_heading_distance = float(rospy.get_param("~min_heading_distance", 0.01))
        self.initial_heading_distance_sq = self.initial_heading_distance * self.initial_heading_distance
        self.min_heading_distance_sq = self.min_heading_distance * self.min_heading_distance
        self.min_heading_speed_mps = float(rospy.get_param("~min_heading_speed_mps", 0.03))
        self.enable_heading_prediction = rospy.get_param("~enable_heading_prediction", True)
        self.heading_prediction_time_s = max(0.0, float(rospy.get_param("~heading_prediction_time_s", 0.10)))
        self.yaw_rate_filter_alpha = min(1.0, max(0.0, float(rospy.get_param("~yaw_rate_filter_alpha", 1.0))))
        self.max_yaw_rate_radps = float(rospy.get_param("~max_yaw_rate_radps", 2.5))
        self.apply_gps_offset = rospy.get_param("~apply_gps_offset", True)
        self.gps_offset_x = float(rospy.get_param("~gps_offset_x", 0.0))
        self.gps_offset_y = float(rospy.get_param("~gps_offset_y", 0.0))
        self.has_gps_offset = self.gps_offset_x != 0.0 or self.gps_offset_y != 0.0
        self.max_speed_mps = float(rospy.get_param("~max_speed_mps", 30.0))

        # =========================
        # ROS 입출력
        # =========================
        # ROS 입출력: 대회 조건상 /gps만 구독하고 odom/tf만 발행한다.
        self.gps_sub = rospy.Subscriber(self.gps_topic, GPSMessage, self.gps_callback, queue_size=1)

        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
        self.tf_br = tf.TransformBroadcaster()

        # =========================
        # 추정기 상태
        # =========================
        # Runtime 상태: raw GPS UTM position, 속도, heading, yaw-rate 추정값을 저장한다.
        self.vel = 0.0
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_heading = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.speed = 0.0
        self.has_fix = False
        self.has_heading = False
        self.heading_initialized = False
        self.has_velocity = False
        self.heading_ref_x = None
        self.heading_ref_y = None
        self.prev_measured_heading = None
        self.last_heading_stamp = None
        self.yaw_rate = 0.0
        self.has_yaw_rate = False
        self.last_gps_stamp = None
        self.last_gps_x = None
        self.last_gps_y = None
        self.publish_timer = None

        # =========================
        # UTM 모델
        # =========================
        # UTM 변환 상수: WGS84/scale/origin 값. 변환 수식은 기존 구현 그대로 둔다.
        self.D0 = 180 / np.pi

        self.A1 = 6378137.0
        self.F1 = 298.257223563

        self.K0 = 0.9996

        self.X0 = 500000
        if (self.zone > 0):
            self.Y0 = 0.0
        else:
            self.Y0 = 1e7

        self.P0 = 0 / self.D0
        self.L0 = (6 * abs(self.zone) - 183) / self.D0

        self.B1 = self.A1 * (1 - 1 / self.F1)
        self.E1 = np.sqrt((self.A1**2 - self.B1**2) / (self.A1**2))
        self.N = self.K0 * self.A1

        self.C = np.zeros(5)
        self.C = proj_coef_0(self.E1)

        self.YS = self.Y0 - self.N * (
            self.C[0] * self.P0
            + self.C[1] * np.sin(2 * self.P0)
            + self.C[2] * np.sin(4 * self.P0)
            + self.C[3] * np.sin(6 * self.P0)
            + self.C[4] * np.sin(8 * self.P0))

        self.C2 = proj_coef_2(self.E1)

        self.x, self.y, self.heading, self.velocity, self.gps_status = None, None, None, None, None

        # Timer publish: GPS 10 Hz 입력이어도 odom/tf는 publish_rate_hz로 반복 발행한다.
        if self.publish_rate_hz > 0.0:
            period = 1.0 / self.publish_rate_hz
            self.publish_timer = rospy.Timer(rospy.Duration(period), self.publish_timer_callback)

    # =========================
    # 좌표 변환
    # =========================
    def convertLL2UTM(self, lat, lon):
        # GPS 위경도(lat/lon)를 UTM east/north로 변환한다.
        # 기존 검증된 수식이므로 내부 계산 순서는 수정하지 않는다.
        p1 = lat / self.D0  # Phi = Latitude(rad)
        l1 = lon / self.D0  # Lambda = Longitude(rad)

        es = self.E1 * np.sin(p1)
        L = np.log( np.tan(np.pi/4.0 + p1/2.0) * 
                    np.power( ((1 - es) / (1 + es)), (self.E1 / 2)))

        z = np.arctan(np.sinh(L) / np.cos(l1 - self.L0)) + 1j * np.log(np.tan(np.pi / 4.0 + np.arcsin(np.sin(l1 - self.L0) / np.cosh(L)) / 2.0))

        Z = self.N * self.C2[0] * z \
            + self.N * (self.C2[1] * np.sin(2.0 * z)
            + self.C2[2] * np.sin(4.0 * z)
            + self.C2[3] * np.sin(6.0 * z)
            + self.C2[4] * np.sin(8.0 * z))

        east = Z.imag + self.X0
        north = Z.real + self.YS

        return east, north

    @staticmethod
    def normalize_angle(angle):
        # yaw 차이/누적값을 [-pi, pi] 범위로 정규화한다.
        return atan2(sin(angle), cos(angle))

    # =========================
    # GPS 메시지 처리
    # =========================
    def get_msg_stamp(self, gps_msg):
        # GPS message timestamp가 비어 있으면 현재 ROS time을 사용한다.
        stamp = gps_msg.header.stamp
        if stamp == rospy.Time(0):
            stamp = rospy.Time.now()
        return stamp

    def get_offsets(self, gps_msg):
        # UTM offset은 기본적으로 파라미터를 쓰고, 옵션이 켜진 경우 GPSMessage offset을 우선한다.
        msg_east_offset = getattr(gps_msg, "eastOffset", 0.0)
        msg_north_offset = getattr(gps_msg, "northOffset", 0.0)
        if self.use_msg_offset and (abs(msg_east_offset) > 1e-6 or abs(msg_north_offset) > 1e-6):
            return msg_east_offset, msg_north_offset
        return self.east_offset, self.north_offset

    def is_valid_gps(self, gps_msg):
        # lat/lon 값과 optional status gate를 검사해서 잘못된 GPS sample을 버린다.
        lat = gps_msg.latitude
        lon = gps_msg.longitude

        if not np.isfinite(lat) or not np.isfinite(lon):
            rospy.logwarn_throttle(1.0, "ll2utm: invalid GPS lat/lon nan or inf")
            return False
        if abs(lat) < 1e-12 and abs(lon) < 1e-12:
            rospy.logwarn_throttle(1.0, "ll2utm: invalid GPS lat/lon zero")
            return False
        if self.require_valid_status and gps_msg.status < self.min_gps_status:
            rospy.logwarn_throttle(1.0, "ll2utm: invalid GPS status %d", gps_msg.status)
            return False
        return True

    # =========================
    # 상태 초기화 helper
    # =========================
    def reset_velocity(self):
        # GPS timeout 등으로 prediction을 멈춰야 할 때 선속도 상태를 0으로 초기화한다.
        self.vx = 0.0
        self.vy = 0.0
        self.speed = 0.0
        self.vel = 0.0
        self.has_velocity = False

    def reset_yaw_rate(self, reset_history=False):
        # heading 자체는 유지하고, heading prediction에 쓰는 yaw-rate만 초기화한다.
        self.yaw_rate = 0.0
        self.has_yaw_rate = False
        if reset_history:
            self.prev_measured_heading = None
            self.last_heading_stamp = None

    # =========================
    # GPS-only motion 추정
    # =========================
    def update_yaw_rate(self, measured_heading, stamp):
        # 새 GPS motion heading과 직전 measured heading의 차이로 yaw-rate를 추정한다.
        if self.prev_measured_heading is None or self.last_heading_stamp is None:
            self.prev_measured_heading = measured_heading
            self.last_heading_stamp = stamp
            self.yaw_rate = 0.0
            self.has_yaw_rate = False
            return

        dt_heading = (stamp - self.last_heading_stamp).to_sec()
        if dt_heading <= 0.0:
            return

        heading_delta = self.normalize_angle(measured_heading - self.prev_measured_heading)
        measured_yaw_rate = heading_delta / dt_heading

        if self.max_yaw_rate_radps > 0.0 and abs(measured_yaw_rate) > self.max_yaw_rate_radps:
            rospy.logwarn_throttle(
                1.0,
                "ll2utm: clamped yaw rate %.3f rad/s to max %.3f rad/s",
                measured_yaw_rate,
                self.max_yaw_rate_radps,
            )
            if measured_yaw_rate > 0.0:
                measured_yaw_rate = self.max_yaw_rate_radps
            else:
                measured_yaw_rate = -self.max_yaw_rate_radps

        if not self.has_yaw_rate:
            self.yaw_rate = measured_yaw_rate
            self.has_yaw_rate = True
        else:
            self.yaw_rate += self.yaw_rate_filter_alpha * (measured_yaw_rate - self.yaw_rate)

        self.prev_measured_heading = measured_heading
        self.last_heading_stamp = stamp

    def update_heading_from_gps_motion(self, x, y, stamp):
        # heading 계산 전용 기준점에서 충분히 이동했을 때만 GPS motion heading을 갱신한다.
        if self.heading_ref_x is None:
            self.heading_ref_x = x
            self.heading_ref_y = y
            return

        dx = x - self.heading_ref_x
        dy = y - self.heading_ref_y
        distance_sq = dx * dx + dy * dy

        if not self.heading_initialized:
            threshold = self.initial_heading_distance
            threshold_sq = self.initial_heading_distance_sq
        else:
            threshold = self.min_heading_distance
            threshold_sq = self.min_heading_distance_sq

        if threshold > 0.0 and distance_sq < threshold_sq:
            return

        if self.speed < self.min_heading_speed_mps:
            return

        measured_heading = atan2(dy, dx)
        if not self.heading_initialized:
            self.prev_heading = measured_heading
            self.has_heading = True
            self.heading_initialized = True
            self.prev_measured_heading = measured_heading
            self.last_heading_stamp = stamp
            self.yaw_rate = 0.0
            self.has_yaw_rate = False
            distance = sqrt(distance_sq)
            rospy.loginfo_throttle(
                1.0,
                "ll2utm: first heading lock-on heading=%.4f rad distance=%.3f speed=%.3f",
                self.prev_heading,
                distance,
                self.speed,
            )
        else:
            self.update_yaw_rate(measured_heading, stamp)
            heading_error = self.normalize_angle(measured_heading - self.prev_heading)
            self.prev_heading = self.normalize_angle(self.prev_heading + self.heading_filter_alpha * heading_error)
            self.has_heading = True

        self.heading_ref_x = x
        self.heading_ref_y = y

    def update_velocity(self, dx, dy, distance_sq, dt):
        # 연속 GPS sample의 raw UTM 차분으로 velocity를 추정하고, 비정상 jump를 거부한다.
        if dt <= 0.0:
            return True

        if self.max_speed_mps > 0.0:
            max_distance = self.max_speed_mps * dt
            if distance_sq > max_distance * max_distance:
                measured_speed = sqrt(distance_sq) / dt
                rospy.logwarn_throttle(
                    1.0,
                    "ll2utm: rejected GPS jump, speed %.2f m/s over dt %.3f",
                    measured_speed,
                    dt,
                )
                return False

        measured_vx = dx / dt
        measured_vy = dy / dt

        if not self.has_velocity:
            self.vx = measured_vx
            self.vy = measured_vy
            self.has_velocity = True
        else:
            self.vx += (measured_vx - self.vx) * self.velocity_filter_alpha
            self.vy += (measured_vy - self.vy) * self.velocity_filter_alpha

        self.speed = sqrt(self.vx * self.vx + self.vy * self.vy)
        self.vel = self.speed
        return True

    def update_gps_estimate(self, x, y, stamp):
        # raw GPS antenna UTM position을 기준으로 velocity/heading state를 갱신한다.
        # publish 위치 보정은 publish_latest()에서 수행해 계산 기준이 섞이지 않게 한다.
        if self.last_gps_x is not None and self.last_gps_stamp is not None:
            dt = (stamp - self.last_gps_stamp).to_sec()

            if dt > self.gps_timeout_s:
                self.reset_velocity()
                self.reset_yaw_rate(reset_history=True)
                self.heading_ref_x = x
                self.heading_ref_y = y
                rospy.logwarn_throttle(
                    1.0,
                    "ll2utm: GPS timeout %.3f s, velocity reset and heading held at %.4f rad",
                    dt,
                    self.prev_heading,
                )
            elif dt > 0.0:
                dx = x - self.last_gps_x
                dy = y - self.last_gps_y
                distance_sq = dx * dx + dy * dy
                if not self.update_velocity(dx, dy, distance_sq, dt):
                    return
                self.update_heading_from_gps_motion(x, y, stamp)
        else:
            self.update_heading_from_gps_motion(x, y, stamp)

        self.last_gps_x = x
        self.last_gps_y = y
        self.last_gps_stamp = stamp
        self.prev_x = x
        self.prev_y = y
        self.x = x
        self.y = y
        self.heading = self.prev_heading
        self.velocity = self.speed
        self.has_fix = True

        rospy.loginfo_throttle(
            1.0,
            "ll2utm: heading_initialized=%s x=%.3f y=%.3f heading=%.4f rad speed=%.3f",
            self.heading_initialized,
            self.x,
            self.y,
            self.prev_heading,
            self.speed,
        )

    # =========================
    # base_link pose publish helper
    # =========================
    def apply_gps_antenna_offset(self, x, y, heading):
        # GPS antenna 좌표를 base_link 좌표로 변환한다. heading lock-on 전에는 raw GPS를 그대로 쓴다.
        if not self.apply_gps_offset or not self.has_gps_offset:
            return x, y

        if not self.heading_initialized:
            if abs(self.gps_offset_x) > 1e-6 or abs(self.gps_offset_y) > 1e-6:
                rospy.logwarn_throttle(
                    2.0,
                    "ll2utm: GPS antenna offset not applied before heading lock-on",
                )
            return x, y

        c = cos(heading)
        s = sin(heading)
        base_x = x - (c * self.gps_offset_x - s * self.gps_offset_y)
        base_y = y - (s * self.gps_offset_x + c * self.gps_offset_y)
        return base_x, base_y

    def publish_odom(self, x, y, heading, speed, stamp):
        # base_link 기준 odometry와 map -> base_link TF를 같은 timestamp로 발행한다.
        q = quaternion_from_euler(0, 0, heading)

        odom_msg = Odometry()
        odom_msg.child_frame_id = self.base_frame_id
        odom_msg.header.frame_id = self.map_frame_id
        odom_msg.header.stamp = stamp
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        odom_msg.twist.twist.linear.x = speed
        odom_msg.twist.twist.angular.z = self.yaw_rate if self.has_yaw_rate else 0.0

        self.odom_pub.publish(odom_msg)

        self.tf_br.sendTransform(
            (x, y, 0),
            q,
            stamp,
            self.base_frame_id,
            self.map_frame_id,
        )

    def publish_latest(self, stamp):
        # 최신 GPS state를 publish 시각까지 짧게 예측하고, 보정된 base_link pose를 발행한다.
        if not self.has_fix or self.last_gps_stamp is None:
            return

        age = (stamp - self.last_gps_stamp).to_sec()
        if age < 0.0:
            age = 0.0
        if age > self.gps_timeout_s:
            self.reset_velocity()
            self.reset_yaw_rate()
            rospy.logwarn_throttle(
                1.0,
                "ll2utm: GPS timeout %.3f s, publishing paused and heading held at %.4f rad",
                age,
                self.prev_heading,
            )
            return

        predict_dt = 0.0
        if self.enable_prediction:
            predict_dt = min(age, self.max_prediction_time_s)

        x = self.last_gps_x + self.vx * predict_dt
        y = self.last_gps_y + self.vy * predict_dt

        heading = self.prev_heading
        if self.enable_heading_prediction and self.has_yaw_rate:
            heading_predict_dt = min(age, self.heading_prediction_time_s)
            heading = self.normalize_angle(self.prev_heading + self.yaw_rate * heading_predict_dt)

        x, y = self.apply_gps_antenna_offset(x, y, heading)
        self.publish_odom(x, y, heading, self.speed, stamp)

    def publish_timer_callback(self, event):
        # ROS timer callback: GPS callback보다 높은 rate로 odom/tf를 반복 발행한다.
        stamp = event.current_real
        if stamp == rospy.Time(0):
            stamp = rospy.Time.now()
        self.publish_latest(stamp)

    def gps_callback(self, gps_msg):
        # /gps callback: GPSMessage를 검증하고 UTM raw position으로 변환한 뒤 추정기를 갱신한다.
        if not self.is_valid_gps(gps_msg):
            return

        lat = gps_msg.latitude
        lon = gps_msg.longitude
        stamp = self.get_msg_stamp(gps_msg)
        e_o, n_o = self.get_offsets(gps_msg)

        e_global, n_global = self.convertLL2UTM(lat, lon)
        x, y = e_global - e_o, n_global - n_o

        self.gps_status = gps_msg.status
        self.update_gps_estimate(x, y, stamp)

        if self.publish_timer is None:
            self.publish_latest(stamp)

if __name__ == '__main__':
    rospy.init_node('ll2utm', anonymous=True)

    loc_sensor = LocationSensor()
    rospy.spin()
