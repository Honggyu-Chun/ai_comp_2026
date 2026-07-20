// rotary_gate.h
// 로터리(회전교차로) 진입 판단 순수 코어 (ROS 의존 없음).
//   등속 원운동 NPC의 충돌점 도달시각(TTC)과 자차 발진 도달/통과 시각을 비교해 진입 타이밍을 판단.
//   방법: 충돌점 시간창 갭수용(gap acceptance). 근거: Muffert et al. IV2012, HCM 임계갭.
//   출력: 정지벽 s[m](base_link 종거리) + 속도상한[kph]. speed_profiler 가 min/정지로 반영.
// 좌표: 전부 MORAI 월드 프레임(자차·NPC 절대좌표). 시간은 now_sec 주입(no ros::).

#ifndef GLOBAL_PATH_PLANNER_ROTARY_GATE_H
#define GLOBAL_PATH_PLANNER_ROTARY_GATE_H

#include <cmath>
#include <string>
#include <vector>

namespace gpp {

struct RotaryGateConfig {
  // --- 기하 (MORAI 월드 프레임) ---
  double center_x = 0.0;             // 로터리 중심 x [m]
  double center_y = 0.0;             // 로터리 중심 y [m]
  double ring_radius_m = 15.0;       // 순환 차로 중심선 반경 R [m]
  double ring_band_m = 3.0;          // |r_i - R| < band 이면 순환 차량으로 판정
  double theta_entry_rad = -M_PI / 2.0;  // 진입 충돌각 (6시, atan2 기준)
  double theta_exit_rad = 0.0;       // 진출각 (3시)
  int    dir = +1;                   // 순환 방향: +1=반시계(CCW, 한국 우측통행)

  // --- 자차 운동 모델 (사다리꼴 발진) ---
  double s_stopline_to_conflict_m = 6.0;  // 정지선→충돌점 경로거리 [m]
  double a_launch_mps2 = 1.5;             // 정지→발진 가속도 (보수적)
  double v_ring_kph = 20.0;               // 로터리 내부 목표(상한) 속도
  double ego_length_m = 4.6;              // 자차 길이 (통과 완료 판정)

  // --- 갭 수용 판정 ---
  double tau_lead_s = 1.0;   // 자차 도착 '전' 여유
  double tau_lag_s = 1.5;    // 자차 통과 '후' 여유
  double v_agent_min_mps = 0.5;   // 이보다 느리면 순환차 아님(정지물은 다른 로직)
  int    go_confirm_frames = 5;   // GO 연속 확정 프레임(채터링 방지)
  double deadlock_timeout_s = 20.0;  // HOLD 지속 시 마진 완화 시작
  double relaxed_scale = 0.7;        // 데드락 시 tau 완화 계수(1회)

  // --- FSM 활성 ---
  double activate_dist_m = 40.0;  // 자차→진입점 유클리드 거리 이하일 때 활성
  double commit_dist_m = 20.0;    // 이 거리 이내 + 갭 열림에서만 COMMIT 잠금(멀리서 조기진입 방지)
  double approach_cap_kph = 25.0; // APPROACH 접근 상한
  bool   allow_rearm = false;     // DONE 후 재무장 허용(테스트/다랩용)
};

struct AgentState { double x = 0.0, y = 0.0, vx = 0.0, vy = 0.0; int id = -1; };  // 월드
struct EgoState   { double x = 0.0, y = 0.0, yaw_rad = 0.0, v_mps = 0.0; };       // 월드

struct RotaryDecision {
  enum Phase { FAR, APPROACH, HOLD, COMMIT, INSIDE, DONE } phase = FAR;
  double stop_wall_s = -1.0;     // base_link 종거리 정지벽[m], 없으면 -1
  double speed_cap_kph = -1.0;   // 속도상한[kph], 없으면 -1
  // 디버그
  double t_ego_arrive_s = -1.0, t_ego_clear_s = -1.0;
  double nearest_agent_eta_s = -1.0;
  int    blocking_agent_id = -1;
  bool   go_ready = false;       // 이번 프레임 accept 여부
  int    n_ring_agents = 0;      // 순환차로(링 밴드) 안에서 움직이는 에이전트 수 (방향무관)
};

class RotaryGate {
 public:
  RotaryGateConfig cfg;

  // 메인: 자차(월드) + 순환 NPC(월드) + 자차→진입충돌점 경로거리 s + 현재시각 → 결정.
  RotaryDecision update(const EgoState& ego, const std::vector<AgentState>& agents,
                        double s_ego_to_conflict_m, double now_sec);

  RotaryDecision::Phase phase() const { return phase_; }
  void reset();

  // 링 밴드 안에서 움직이는가(|r-R|<band && speed>v_min). 방향무관 — 공간적 "로터리 안" 판정.
  // 로그/마커에서 순환 에이전트만 한정하는 데 사용(evaluateGap 과 동일 필터).
  bool isRingAgent(const AgentState& a) const;

 private:
  // 상태 (프레임 간 유지)
  RotaryDecision::Phase phase_ = RotaryDecision::FAR;
  int  go_streak_ = 0;
  double hold_start_sec_ = -1.0;
  bool   relaxed_applied_ = false;
  bool   done_latched_ = false;
  double prev_ego_theta_ = 0.0;     // 자차 각도(충돌점 통과 판정용)
  bool   has_prev_theta_ = false;
  double ego_theta_progress_ = 0.0; // COMMIT 이후 누적 각도진행

  // 자차 사다리꼴 발진: (도달시각, 통과시각) 반환
  void egoArriveClear(double v0, double s, double* t_arrive, double* t_clear) const;
  // 순환차 필터 + ETA. accept 여부와 디버그 채움.
  bool evaluateGap(const std::vector<AgentState>& agents, double t_arrive, double t_clear,
                   double tau_lead, double tau_lag, double* nearest_eta, int* block_id) const;
};

}  // namespace gpp

#endif  // GLOBAL_PATH_PLANNER_ROTARY_GATE_H
