// speed_profiler.cpp — 판단 속도 프로파일 코어 구현 (ROS 무관).
// mpc_control.cpp(buildPathData/applyBrakingSpeedProfile)와 acc_node.cpp(accTargetSpeed)를 이식.

#include "global_path_planner/speed_profiler.h"

#include <algorithm>
#include <limits>

namespace gpp {

namespace {
inline double clampd(double v, double lo, double hi) { return std::max(lo, std::min(v, hi)); }
inline double normalizeAngle(double a) { return std::atan2(std::sin(a), std::cos(a)); }
}  // namespace

// mpc_control.cpp:664-745 이식: path_resolution_min 리샘플 → yaw → 중심차분 곡률 → 삼각가중 평활.
std::vector<SpeedProfiler::WP> SpeedProfiler::resampleAndCurvature(const std::vector<Vec2>& P) const {
  std::vector<WP> wp;
  wp.reserve(P.size());
  for (std::size_t i = 0; i < P.size(); ++i) {
    if (!wp.empty()) {
      const double ds = std::hypot(P[i].x - wp.back().x, P[i].y - wp.back().y);
      if (ds < cfg.path_resolution_min) continue;
    }
    WP p;
    p.x = P[i].x;
    p.y = P[i].y;
    p.s = wp.empty() ? 0.0 : wp.back().s + std::hypot(P[i].x - wp.back().x, P[i].y - wp.back().y);
    p.curvature = 0.0;
    p.v_mps = cfg.max_speed_kph / 3.6;
    wp.push_back(p);
  }
  const std::size_t n = wp.size();
  if (n < 2) return wp;

  std::vector<double> yaw(n, 0.0);
  for (std::size_t i = 0; i + 1 < n; ++i)
    yaw[i] = std::atan2(wp[i + 1].y - wp[i].y, wp[i + 1].x - wp[i].x);
  yaw[n - 1] = yaw[n - 2];

  std::vector<double> raw(n, 0.0);
  for (std::size_t i = 1; i + 1 < n; ++i) {
    const double dyaw = normalizeAngle(yaw[i + 1] - yaw[i - 1]);
    const double ds = wp[i + 1].s - wp[i - 1].s;
    if (ds > 1e-6) raw[i] = dyaw / ds;
  }
  if (n > 2) { raw.front() = raw[1]; raw.back() = raw[n - 2]; }

  // 호길이 삼각가중 평활 (슬라이딩 윈도우)
  std::size_t wb = 0, we = 0;
  for (std::size_t i = 0; i < n; ++i) {
    const double s_i = wp[i].s;
    while (wb < n && s_i - wp[wb].s > cfg.curvature_smoothing_window_m) ++wb;
    we = std::max(we, wb);
    while (we < n && wp[we].s - s_i <= cfg.curvature_smoothing_window_m) ++we;
    double wsum = 0.0, weight_sum = 0.0;
    for (std::size_t j = wb; j < we; ++j) {
      const double ds_abs = std::abs(wp[j].s - s_i);
      const double w = 1.0 - ds_abs / std::max(1e-3, cfg.curvature_smoothing_window_m);
      wsum += w * raw[j];
      weight_sum += w;
    }
    wp[i].curvature = (weight_sum > 1e-6) ? wsum / weight_sum : raw[i];
  }
  return wp;
}

// 역방향 제동 1패스: v_i = min(v_i, sqrt(v_{i+1}^2 + 2·a·ds)). v 를 하향만 시킴.
void SpeedProfiler::backwardBrakePass(std::vector<WP>& wp) const {
  if (wp.size() < 2) return;
  const double a = std::max(0.1, cfg.brake_decel_mps2);
  for (std::size_t i = wp.size() - 1; i-- > 0;) {
    const double ds = std::max(0.0, wp[i + 1].s - wp[i].s);
    const double vn = wp[i + 1].v_mps;
    const double reachable = std::sqrt(std::max(0.0, vn * vn + 2.0 * a * ds));
    wp[i].v_mps = std::min(wp[i].v_mps, reachable);
  }
}

// mpc_control.cpp:1003-1040 이식: 곡률한계 + 역방향 제동 포락선.
void SpeedProfiler::brakingProfile(std::vector<WP>& wp, double v_cap_mps) const {
  if (wp.empty()) return;
  const double a_lat = std::max(0.1, cfg.curve_speed_safety_factor * cfg.lateral_accel_limit_mps2);
  const double v_min = cfg.minimum_curve_speed_kph / 3.6;
  for (auto& p : wp) {
    const double k = std::abs(p.curvature);
    double v = v_cap_mps;
    if (k > 1e-5) v = std::sqrt(a_lat / k);
    p.v_mps = clampd(v, v_min, v_cap_mps);
  }
  backwardBrakePass(wp);
}

// acc_node.cpp accTargetSpeed 이식: CTG 선형 ACC. 선행차 없으면 <0 반환(무제한).
double SpeedProfiler::followingSpeedMps(const std::vector<Obstacle>& obs, double ego_speed_mps,
                                        int* lead_id) const {
  if (lead_id) *lead_id = -1;
  // 선행차 = 전방(x>0) & |y|<lane_gate & "이동" 중 x 최소.
  // 정적 장애물(speed<v_static)은 ACC 대상 아님 → obstacleStopS(정지+제동)가 처리. (역할 분리)
  const Obstacle* lead = NULL;
  for (const auto& o : obs) {
    if (o.x <= 0.0) continue;
    if (std::abs(o.y) > cfg.acc_lane_gate_m) continue;
    if (o.speed < cfg.v_static_max_mps) continue;  // 정적은 stop 이 담당
    if (lead == NULL || o.x < lead->x) lead = &o;
  }
  if (lead == NULL) return -1.0;  // 무선행차 → 무제한(60 클램프는 곡률프로파일이 담당)
  if (lead_id) *lead_id = lead->id;
  const double gap = lead->x - cfg.acc_standoff_m - 0.5 * lead->length;
  const double desired_gap = cfg.acc_min_gap_m + cfg.acc_headway_s * ego_speed_mps;
  const double v = lead->speed + cfg.acc_gain_k * (gap - desired_gap);
  return std::max(0.0, v);  // v_set 캡은 곡률프로파일 max_speed 가 min 으로 처리
}

// 전방 정적 장애물(차로 내 근접)의 정지 s 반환 (없으면 <0).
double SpeedProfiler::obstacleStopS(const std::vector<Obstacle>& obs) const {
  double best_s = -1.0;
  for (const auto& o : obs) {
    if (o.speed >= cfg.v_static_max_mps) continue;         // 정적만
    if (o.x <= 0.0) continue;                              // 전방
    if (std::abs(o.y) > cfg.acc_lane_gate_m) continue;     // 차로 내
    const double s_obs = o.x - cfg.acc_standoff_m - 0.5 * o.length;
    if (s_obs > cfg.stop_distance_m) continue;             // 근접만
    const double stop_s = std::max(0.0, s_obs);
    if (best_s < 0.0 || stop_s < best_s) best_s = stop_s;  // 가장 가까운 곳에서 정지
  }
  return best_s;
}

void SpeedProfiler::applyExternalConstraints(const std::vector<Vec2>& P,
                                             std::vector<double>& speed_kph, double stop_wall_s,
                                             double cap_kph) const {
  if (P.size() != speed_kph.size() || P.size() < 2) return;
  if (stop_wall_s < 0.0 && cap_kph < 0.0) return;
  // 원본 P 로 WP(x,y,s,v) 구성 (중복 s 재적산 — backwardBrakePass 재사용 목적)
  std::vector<WP> wp(P.size());
  double acc = 0.0;
  for (std::size_t i = 0; i < P.size(); ++i) {
    if (i > 0) acc += std::hypot(P[i].x - P[i - 1].x, P[i].y - P[i - 1].y);
    wp[i].x = P[i].x; wp[i].y = P[i].y; wp[i].s = acc; wp[i].curvature = 0.0;
    double v = speed_kph[i];
    if (cap_kph >= 0.0) v = std::min(v, cap_kph);                 // 속도상한
    if (stop_wall_s >= 0.0 && acc >= stop_wall_s) v = 0.0;        // 정지벽 이후 0
    wp[i].v_mps = v / 3.6;
  }
  backwardBrakePass(wp);  // 정지선 앞 감속 램프 재전파
  for (std::size_t i = 0; i < P.size(); ++i) speed_kph[i] = wp[i].v_mps * 3.6;
}

SpeedProfileResult SpeedProfiler::plan(const std::vector<Vec2>& P,
                                       const std::vector<Obstacle>& obstacles,
                                       double ego_speed_mps) const {
  SpeedProfileResult res;
  const double v_cap = cfg.max_speed_kph / 3.6;
  if (static_cast<int>(P.size()) < cfg.min_points) {
    res.valid = false;
    res.speed_kph.assign(P.size(), cfg.max_speed_kph);  // 안전: 상한(제어 fallback 유도용은 아님)
    return res;
  }

  std::vector<WP> wp = resampleAndCurvature(P);
  if (wp.size() < 2) {
    res.valid = false;
    res.speed_kph.assign(P.size(), cfg.max_speed_kph);
    return res;
  }

  // Phase1: 곡률한계 + 제동 (baseline)
  brakingProfile(wp, v_cap);

  // Phase2: ADAS 앞차 추종(min cap) + 전방 정지장애물(v=0) 을 추가 제약으로 하향 후 제동 재전파.
  double v_follow = -1.0;
  int lead_id = -1;
  bool changed = false;
  if (cfg.use_following) {
    v_follow = followingSpeedMps(obstacles, ego_speed_mps, &lead_id);
    if (v_follow >= 0.0) {
      for (auto& p : wp) p.v_mps = std::min(p.v_mps, v_follow);
      changed = true;
    }
  }
  if (cfg.use_obstacle_stop) {
    const double stop_s = obstacleStopS(obstacles);
    if (stop_s >= 0.0) {
      for (auto& p : wp)
        if (p.s >= stop_s) p.v_mps = 0.0;  // 정지점 이후 0
      changed = true;
    }
  }
  if (changed) backwardBrakePass(wp);  // 추가 하향 제약을 앞 구간으로 제동 전파(곡률한계 유지)

  // 원본 P 각 점의 s 로 wp 속도 선형보간 → kph
  res.speed_kph.resize(P.size());
  double min_kph = cfg.max_speed_kph;
  double acc_s = 0.0;
  std::size_t j = 0;
  for (std::size_t i = 0; i < P.size(); ++i) {
    if (i > 0) acc_s += std::hypot(P[i].x - P[i - 1].x, P[i].y - P[i - 1].y);
    const double s = acc_s;
    while (j + 1 < wp.size() && wp[j + 1].s < s) ++j;
    double v;
    if (s <= wp.front().s) v = wp.front().v_mps;
    else if (s >= wp.back().s) v = wp.back().v_mps;
    else {
      const std::size_t j2 = std::min(j + 1, wp.size() - 1);
      const double s1 = wp[j].s, s2 = wp[j2].s;
      const double t = (s2 > s1) ? (s - s1) / (s2 - s1) : 0.0;
      v = wp[j].v_mps + t * (wp[j2].v_mps - wp[j].v_mps);
    }
    const double kph = clampd(v * 3.6, 0.0, cfg.max_speed_kph);
    res.speed_kph[i] = kph;
    min_kph = std::min(min_kph, kph);
  }

  res.valid = true;
  res.v_follow_kph = (v_follow >= 0.0) ? v_follow * 3.6 : -1.0;
  res.lead_id = lead_id;
  res.min_kph = min_kph;
  return res;
}

}  // namespace gpp
