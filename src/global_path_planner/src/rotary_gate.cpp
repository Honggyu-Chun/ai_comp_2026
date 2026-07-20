// rotary_gate.cpp — 로터리 진입 판단 코어 (ROS 무관).

#include "global_path_planner/rotary_gate.h"

#include <algorithm>
#include <limits>

namespace gpp {

namespace {
constexpr double kPi = M_PI, k2Pi = 2.0 * M_PI;
inline double wrapToPi(double a) { return std::atan2(std::sin(a), std::cos(a)); }
inline double wrapTo2Pi(double a) {  // [0, 2pi)
  a = std::fmod(a, k2Pi);
  if (a < 0.0) a += k2Pi;
  return a;
}
}  // namespace

void RotaryGate::reset() {
  phase_ = RotaryDecision::FAR;
  go_streak_ = 0;
  hold_start_sec_ = -1.0;
  relaxed_applied_ = false;
  done_latched_ = false;
  has_prev_theta_ = false;
  ego_theta_progress_ = 0.0;
}

// 사다리꼴 발진: 현재속도 v0, 남은거리 s → 충돌점 도달/통과 시각.
void RotaryGate::egoArriveClear(double v0, double s, double* t_arrive, double* t_clear) const {
  const double v_ring = std::max(0.1, cfg.v_ring_kph / 3.6);
  const double a = std::max(0.1, cfg.a_launch_mps2);
  s = std::max(0.0, s);
  v0 = std::max(0.0, v0);
  double t_a;
  if (v0 >= v_ring) {
    t_a = s / v_ring;  // 이미 링속도 이상 → 순항 근사(v0>v_ring 엣지 클램프)
  } else {
    const double s_acc = (v_ring * v_ring - v0 * v0) / (2.0 * a);  // >0
    if (s <= s_acc) {
      t_a = (std::sqrt(v0 * v0 + 2.0 * a * s) - v0) / a;  // 가속만
    } else {
      t_a = (v_ring - v0) / a + (s - s_acc) / v_ring;     // 가속 + 순항
    }
  }
  t_a = std::max(0.0, t_a);
  *t_arrive = t_a;
  *t_clear = t_a + (cfg.ego_length_m + 2.0) / v_ring;  // 충돌점 통과 여유 2m
}

// 순환차 필터 + ETA. 도착창 [t_arrive-tau_lead, t_clear+tau_lag] 를 침범하는 차가 없으면 accept.
bool RotaryGate::isRingAgent(const AgentState& a) const {
  const double r = std::hypot(a.x - cfg.center_x, a.y - cfg.center_y);
  if (std::abs(r - cfg.ring_radius_m) > cfg.ring_band_m) return false;  // 링 밖
  if (std::hypot(a.vx, a.vy) < cfg.v_agent_min_mps) return false;       // 정지물
  return true;
}

bool RotaryGate::evaluateGap(const std::vector<AgentState>& agents, double t_arrive, double t_clear,
                             double tau_lead, double tau_lag, double* nearest_eta,
                             int* block_id) const {
  const double win_lo = t_arrive - tau_lead;
  const double win_hi = t_clear + tau_lag;
  bool accept = true;
  double nearest = -1.0;
  int block = -1;
  double block_eta = std::numeric_limits<double>::infinity();

  for (const auto& ag : agents) {
    if (!isRingAgent(ag)) continue;  // 링 밖 or 정지물
    const double px = ag.x - cfg.center_x, py = ag.y - cfg.center_y;
    const double r = std::hypot(px, py);
    const double sp = std::hypot(ag.vx, ag.vy);
    // 회전방향 데이터 역산: cross(p-C, v) 부호
    const double cross = px * ag.vy - py * ag.vx;
    const int agent_dir = (cross > 0.0) ? +1 : -1;
    if (agent_dir != cfg.dir) continue;  // 반대방향(진출차/오설정) 제외
    if (r < 1e-3) continue;
    const double omega = sp / r;
    const double theta_i = std::atan2(py, px);
    const double dtheta = wrapTo2Pi(cfg.dir * (cfg.theta_entry_rad - theta_i));
    const double eta = dtheta / std::max(1e-6, omega);

    if (nearest < 0.0 || eta < nearest) nearest = eta;
    if (eta >= win_lo && eta <= win_hi) {  // 도착창 침범 → 거부
      accept = false;
      if (eta < block_eta) { block_eta = eta; block = ag.id; }
    }
  }
  if (nearest_eta) *nearest_eta = nearest;
  if (block_id) *block_id = block;
  return accept;
}

RotaryDecision RotaryGate::update(const EgoState& ego, const std::vector<AgentState>& agents,
                                  double s_ego_to_conflict_m, double now_sec) {
  RotaryDecision out;

  // DONE 래치 (재무장 금지)
  if (done_latched_ && !cfg.allow_rearm) {
    out.phase = RotaryDecision::DONE;
    return out;  // 전 제약 -1
  }

  // 자차가 로터리 footprint(링 밴드 내부까지) 안에 물리적으로 있는가.
  const double ego_r = std::hypot(ego.x - cfg.center_x, ego.y - cfg.center_y);
  const bool ego_in_ring = ego_r <= cfg.ring_radius_m + cfg.ring_band_m;
  // 진입점(충돌점)까지 유클리드 거리 — heading 무관, 전방투영 s 노이즈에 강인.
  // 활성/COMMIT 게이팅에 사용(s 는 정지벽 위치·타이밍에만).
  const double pcx = cfg.center_x + cfg.ring_radius_m * std::cos(cfg.theta_entry_rad);
  const double pcy = cfg.center_y + cfg.ring_radius_m * std::sin(cfg.theta_entry_rad);
  const double dist_to_entry = std::hypot(ego.x - pcx, ego.y - pcy);
  // 자차 각도 진행 누적 (진출점 통과 판정). 링 안에 있을 때만 누적 —
  // 접근/진출 직선 주행이 각도를 오염시켜 조기 INSIDE/DONE 되는 것을 방지.
  const double theta_ego = std::atan2(ego.y - cfg.center_y, ego.x - cfg.center_x);
  if (has_prev_theta_ && ego_in_ring)
    ego_theta_progress_ += cfg.dir * wrapToPi(theta_ego - prev_ego_theta_);
  prev_ego_theta_ = theta_ego;
  has_prev_theta_ = true;
  const double arc_to_exit = wrapTo2Pi(cfg.dir * (cfg.theta_exit_rad - cfg.theta_entry_rad));

  const double stopline_dist = std::max(0.0, s_ego_to_conflict_m - cfg.s_stopline_to_conflict_m);

  switch (phase_) {
    case RotaryDecision::FAR: {
      // 진입점까지 유클리드 거리(강인) 이내일 때만 활성.
      if (dist_to_entry > cfg.activate_dist_m) break;
      phase_ = RotaryDecision::APPROACH;
      // fallthrough: 활성 프레임에서도 즉시 갭 판정/출력
    }
    // fall through
    case RotaryDecision::APPROACH: {
      // 노이즈 blip 으로 잘못 활성됐다가 실제로 멀면 FAR 복귀(히스테리시스).
      if (dist_to_entry > cfg.activate_dist_m * 1.5) { phase_ = RotaryDecision::FAR; go_streak_ = 0; break; }
      double ta, tc;
      egoArriveClear(ego.v_mps, s_ego_to_conflict_m, &ta, &tc);
      double neta; int bid;
      const bool ok = evaluateGap(agents, ta, tc, cfg.tau_lead_s, cfg.tau_lag_s, &neta, &bid);
      go_streak_ = ok ? go_streak_ + 1 : 0;
      out.t_ego_arrive_s = ta; out.t_ego_clear_s = tc;
      out.nearest_agent_eta_s = neta; out.blocking_agent_id = bid; out.go_ready = ok;

      // COMMIT 은 진입점 근처(commit zone)에서 + 갭이 연속 열림일 때만 잠금.
      // → 멀리서 조기 COMMIT 방지. 실제 yield 지점에서 TTC 가 결정.
      if (go_streak_ >= cfg.go_confirm_frames && dist_to_entry <= cfg.commit_dist_m) {
        phase_ = RotaryDecision::COMMIT;
        ego_theta_progress_ = 0.0;
      } else if (dist_to_entry <= cfg.s_stopline_to_conflict_m + 1.0) {
        phase_ = RotaryDecision::HOLD;                    // 정지선 도달, 미확정 → 정지
        hold_start_sec_ = now_sec;
        go_streak_ = 0;
      } else {
        out.stop_wall_s = stopline_dist;                  // 항상 정지선 앞 감속 준비
        out.speed_cap_kph = cfg.approach_cap_kph;
      }
      break;
    }
    case RotaryDecision::HOLD: {
      // 정지 발진(v0=0) 기준 재판정 + 데드락 완화
      double tau_lead = cfg.tau_lead_s, tau_lag = cfg.tau_lag_s;
      if (hold_start_sec_ >= 0.0 && (now_sec - hold_start_sec_) > cfg.deadlock_timeout_s) {
        tau_lead *= cfg.relaxed_scale; tau_lag *= cfg.relaxed_scale;  // 1회성(값만 완화)
        relaxed_applied_ = true;
      }
      double ta, tc;
      egoArriveClear(0.0, cfg.s_stopline_to_conflict_m, &ta, &tc);  // 정지선→충돌점
      double neta; int bid;
      const bool ok = evaluateGap(agents, ta, tc, tau_lead, tau_lag, &neta, &bid);
      go_streak_ = ok ? go_streak_ + 1 : 0;
      out.t_ego_arrive_s = ta; out.t_ego_clear_s = tc;
      out.nearest_agent_eta_s = neta; out.blocking_agent_id = bid; out.go_ready = ok;

      if (go_streak_ >= cfg.go_confirm_frames) {
        phase_ = RotaryDecision::COMMIT;
        ego_theta_progress_ = 0.0;
      } else {
        out.stop_wall_s = stopline_dist;  // 정지선 정지
      }
      break;
    }
    case RotaryDecision::COMMIT: {
      // 래치: 정지벽 절대 재생성 안 함. 아직 링 밖(접근 직선) → 접근속도 유지.
      // 링속도(20)는 자차가 물리적으로 링에 진입(INSIDE)해야만. 그래야 멀리서부터 기지 않음.
      out.speed_cap_kph = cfg.approach_cap_kph;
      if (ego_in_ring) phase_ = RotaryDecision::INSIDE;
      break;
    }
    case RotaryDecision::INSIDE: {
      out.speed_cap_kph = cfg.v_ring_kph;
      // 진출: 진출각까지 각도진행 완료, 또는 진행 후 링 밖으로 나감(출구 통과).
      if (ego_theta_progress_ >= arc_to_exit ||
          (!ego_in_ring && ego_theta_progress_ > 0.15)) {
        phase_ = RotaryDecision::DONE;
        if (!cfg.allow_rearm) done_latched_ = true;
      }
      break;
    }
    case RotaryDecision::DONE: {
      if (cfg.allow_rearm && dist_to_entry > cfg.activate_dist_m) reset();
      break;
    }
  }

  for (const auto& ag : agents) if (isRingAgent(ag)) ++out.n_ring_agents;  // 링 안 에이전트 수(디버그)
  out.phase = phase_;
  return out;
}

}  // namespace gpp
