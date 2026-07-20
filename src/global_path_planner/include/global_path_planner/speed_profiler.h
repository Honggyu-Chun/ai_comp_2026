// speed_profiler.h
// 판단(decision) 속도 프로파일링 순수 코어 (ROS 의존 없음).
//   경로 base_link 점열 P + (선택) 장애물/자차속도 → 점별 목표속도[kph].
//   Phase1: 곡률 감속 + 역방향 제동 포락선 (mpc_control 의 buildPathData/applyBrakingSpeedProfile 이식).
//   Phase2: ADAS 앞차 추종(CTG) + 전방 정지장애물 을 min 으로 합성 (acc_node 의 accTargetSpeed 이식).
// 좌표: base_link (x 전방, y 좌측+). 내부 계산 m/s, 출력 kph.

#ifndef GLOBAL_PATH_PLANNER_SPEED_PROFILER_H
#define GLOBAL_PATH_PLANNER_SPEED_PROFILER_H

#include <cmath>
#include <vector>

#include "global_path_planner/path_shift_planner.h"  // gpp::Vec2, gpp::Obstacle 재사용

namespace gpp {

struct SpeedProfileConfig {
  // --- Phase1: 곡률 + 제동 (제어에서 이식) ---
  double max_speed_kph = 60.0;               // 상한(60 하드클램프)
  double lateral_accel_limit_mps2 = 2.2;     // 횡가속 한계
  double curve_speed_safety_factor = 0.82;   // 곡률감속 안전계수 (0.3~1.0)
  double minimum_curve_speed_kph = 2.3;      // 곡률감속 하한
  double brake_decel_mps2 = 8.99;            // 역방향 제동 감속도
  double path_resolution_min = 0.5;          // 곡률계산용 리샘플 최소간격[m]
  double curvature_smoothing_window_m = 0.98;// 호길이 삼각가중 평활 창

  // --- Phase2: ADAS 앞차 추종 (CTG) ---
  bool   use_following = true;
  double acc_headway_s = 1.5;                // 시간간격 tau
  double acc_min_gap_m = 6.0;                // 정지 최소간격 d0
  double acc_gain_k = 0.4;                   // 간격오차 비례게인
  double acc_lane_gate_m = 1.5;              // |y_m| 초과면 다른 차로로 무시
  double acc_standoff_m = 2.5;               // base_link 원점→앞범퍼 오프셋

  // --- Phase2: 전방 정지 장애물 ---
  bool   use_obstacle_stop = true;
  double stop_distance_m = 6.0;              // 전방 이 s[m] 안 정적장애물이면 그 앞에서 정지
  double v_static_max_mps = 0.5;             // 정적 판정 속도

  // 유효 경로 최소 점 수
  int min_points = 3;
};

struct SpeedProfileResult {
  bool valid = false;
  std::vector<double> speed_kph;   // P 와 같은 길이(원본 점 기준). 각 점 목표속도[kph]
  double v_follow_kph = -1.0;      // ADAS 추종속도(디버그, 선행차 없으면 <0)
  int lead_id = -1;                // 선행차 id(디버그, 없으면 -1)
  double min_kph = 0.0;            // 프로파일 최소값(디버그)
};

class SpeedProfiler {
 public:
  SpeedProfileConfig cfg;

  // 메인: 경로 P(base_link) + 장애물 + 자차속도[m/s] → 점별 목표속도[kph] (P와 동일 길이).
  // obstacles/ego_speed 는 Phase2(ADAS/정지)용. 비우면 순수 곡률+제동 프로파일.
  SpeedProfileResult plan(const std::vector<Vec2>& P,
                          const std::vector<Obstacle>& obstacles,
                          double ego_speed_mps) const;

  // 외부 제약(로터리 등)을 이미 계산된 점별 속도[kph]에 적용하고 정지선 앞 감속을 재전파한다.
  //   stop_wall_s: base_link 종거리 정지벽[m] (s>=이 값 구간 0). <0 이면 무시.
  //   cap_kph: 전구간 속도상한[kph]. <0 이면 무시.
  // speed_kph 는 P 와 동일 길이(plan() 출력)이며 in-place 갱신된다.
  void applyExternalConstraints(const std::vector<Vec2>& P, std::vector<double>& speed_kph,
                                double stop_wall_s, double cap_kph) const;

 private:
  // 리샘플된 작업점 (곡률/속도 계산용)
  struct WP { double x, y, s, curvature, v_mps; };

  std::vector<WP> resampleAndCurvature(const std::vector<Vec2>& P) const;
  void brakingProfile(std::vector<WP>& wp, double v_cap_mps) const;     // 곡률한계 + 역방향 제동
  void backwardBrakePass(std::vector<WP>& wp) const;                    // 역방향 제동 1패스(v만 하향)
  double followingSpeedMps(const std::vector<Obstacle>& obs, double ego_speed_mps,
                           int* lead_id) const;                         // CTG ACC (선행차 없으면 <0)
  double obstacleStopS(const std::vector<Obstacle>& obs) const;         // 정지시켜야 할 s (없으면 <0)
};

}  // namespace gpp

#endif  // GLOBAL_PATH_PLANNER_SPEED_PROFILER_H
