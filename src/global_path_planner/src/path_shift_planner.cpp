// path_shift_planner.cpp — 정적 장애물 회피 offset 코어 구현 (ROS 무관).
// 원본 avoid_algorithm.cpp 의 알고리즘을 그대로 이식(ros::Time → now_sec, 멤버 param → cfg).

#include "global_path_planner/path_shift_planner.h"

#include <algorithm>

namespace gpp {

namespace {
inline double dot(const Vec2& a, const Vec2& b) { return a.x * b.x + a.y * b.y; }
inline double norm2(const Vec2& a) { return dot(a, a); }
inline double vnorm(const Vec2& a) { return std::sqrt(norm2(a)); }
inline double clampd(double v, double lo, double hi) { return std::max(lo, std::min(v, hi)); }
inline double smoothstep(double u) {
  u = clampd(u, 0.0, 1.0);
  return u * u * (3.0 - 2.0 * u);
}
}  // namespace

void PathShiftPlanner::reset() {
  state_ = PlannerState::NORMAL;
  return_start_time_ = 0.0;
  last_now_ = -1.0;
  offset_current_.clear();
  prev_S_.clear();
  side_lock_map_.clear();
}

std::vector<double> PathShiftPlanner::cumulativeS(const std::vector<Vec2>& P) {
  std::vector<double> S(P.size(), 0.0);
  double acc = 0.0;
  for (std::size_t i = 1; i < P.size(); ++i) {
    acc += vnorm(P[i] - P[i - 1]);
    S[i] = acc;
  }
  return S;
}

PathShiftPlanner::ClosestInfo PathShiftPlanner::computeClosestToPath(
    const Vec2& O, const std::vector<Vec2>& P, const std::vector<double>& S) const {
  ClosestInfo best;
  if (P.size() < 2) return best;
  // s 는 단조증가 → 계획 범위(s_plan_max)를 여유 두고 넘으면 이후 세그먼트는 후보 될 수 없어 조기 종료.
  const double s_scan_limit = cfg.s_plan_max + cfg.r_obs_max + cfg.max_shift;
  for (std::size_t i = 0; i + 1 < P.size(); ++i) {
    if (!S.empty() && S[i] > s_scan_limit) break;
    Vec2 A = P[i], B = P[i + 1];
    Vec2 AB = B - A;
    double ab2 = norm2(AB);
    if (ab2 < 1e-9) continue;
    Vec2 AO = O - A;
    double t = clampd(dot(AO, AB) / ab2, 0.0, 1.0);
    Vec2 Q = A + AB * t;
    double d = vnorm(O - Q);
    if (d < best.d_min) {
      best.d_min = d;
      best.seg_idx = static_cast<int>(i);
      best.t = t;
      best.q = Q;
      double seg_len = std::sqrt(ab2);
      double sA = S.empty() ? 0.0 : S[i];
      best.s_star = sA + t * seg_len;
    }
  }
  return best;
}

Vec2 PathShiftPlanner::tangentAt(std::size_t i, const std::vector<Vec2>& P) {
  if (P.size() < 2) return Vec2(1.0, 0.0);
  Vec2 t;
  if (i == 0) {
    t = P[1] - P[0];
  } else if (i + 1 >= P.size()) {
    t = P[P.size() - 1] - P[P.size() - 2];
  } else {
    t = P[i + 1] - P[i - 1];
  }
  double nrm = vnorm(t);
  if (nrm < 1e-9) return Vec2(1.0, 0.0);
  return Vec2(t.x / nrm, t.y / nrm);
}

std::vector<PathShiftPlanner::Candidate> PathShiftPlanner::buildCandidates(
    const std::vector<Obstacle>& obs, const std::vector<Vec2>& P,
    const std::vector<double>& S, double now_sec) {
  std::vector<Candidate> out;
  for (const auto& o : obs) {
    Candidate c;
    c.unique_id = o.id;
    c.pos = Vec2(o.x, o.y);
    c.speed = o.speed;

    // 전방 게이팅
    if (c.pos.x <= cfg.x_front_min) continue;
    if (c.pos.x >= cfg.x_plan_max) continue;
    // 정적 게이팅
    if (c.speed >= cfg.v_static_max) continue;

    // 장애물 반경(대각 기반): 원본은 size.x/size.y → 여기선 length/width
    double sx = o.length;
    double sy = o.width;
    c.r_obs = 0.5 * std::sqrt(sx * sx + sy * sy);
    c.r_obs = clampd(c.r_obs, cfg.r_obs_min, cfg.r_obs_max);

    // 안전 반경(필수 이격). avoid_offset>0 이면 사용자 지정값(물리 최소치 보장), 아니면 자동.
    const double phys_min = cfg.veh_half_width + cfg.obs_radius_scale * c.r_obs;  // 마진 제외 최소
    if (cfg.avoid_offset > 0.0)
      c.d_block = std::max(cfg.avoid_offset, phys_min);
    else
      c.d_block = phys_min + cfg.margin_obs + std::max(0.0, cfg.extra_clearance);

    c.cinfo = computeClosestToPath(c.pos, P, S);
    if (!std::isfinite(c.cinfo.d_min) || !std::isfinite(c.cinfo.s_star)) continue;

    bool ahead = (c.cinfo.s_star > 0.0 && c.cinfo.s_star < cfg.s_plan_max);
    if (!ahead) continue;

    if (c.cinfo.d_min > c.d_block + std::max(0.0, cfg.soften_margin)) continue;

    if (cfg.soften_margin <= 1e-6) {
      c.gain = (c.cinfo.d_min < c.d_block) ? 1.0 : 0.0;
    } else {
      double u = (c.d_block + cfg.soften_margin - c.cinfo.d_min) / cfg.soften_margin;
      c.gain = smoothstep(u);
    }

    // 경로 기준 side_value = (O-Q)·n, n=(-t.y,t.x) 좌측
    {
      std::size_t i_idx = 0;
      if (c.cinfo.seg_idx >= 0) i_idx = static_cast<std::size_t>(c.cinfo.seg_idx);
      if (P.empty()) continue;
      if (i_idx >= P.size()) i_idx = P.size() - 1;
      Vec2 t = tangentAt(i_idx, P);
      Vec2 n(-t.y, t.x);
      Vec2 OQ = c.pos - c.cinfo.q;
      c.side_value = dot(OQ, n);
    }

    c.obstacle_side = getLockedObstacleSide(c.unique_id, c.side_value, now_sec);
    const int desired_pass = preferredPass(c.side_value, c.d_block, c.obstacle_side);
    c.pass_sign = getLockedPass(c.unique_id, desired_pass, now_sec);
    c.pass_sign_raw = c.pass_sign;

    out.push_back(c);
  }
  return out;
}

int PathShiftPlanner::getLockedObstacleSide(int uid, double side_value, double now_sec) {
  int side_now = 0;
  if (std::abs(side_value) < cfg.side_deadband)
    side_now = 0;
  else
    side_now = (side_value > 0.0) ? +1 : -1;

  auto it = side_lock_map_.find(uid);
  if (it == side_lock_map_.end()) {
    SideLockInfo info;
    info.obstacle_side = side_now;
    info.side_value0 = side_value;
    info.last_seen = now_sec;
    info.lock_until = now_sec + cfg.side_lock_sec;
    side_lock_map_[uid] = info;
    return info.obstacle_side;
  }
  it->second.last_seen = now_sec;
  if (now_sec < it->second.lock_until) return it->second.obstacle_side;
  if (side_now == 0) {
    it->second.lock_until = now_sec + cfg.side_lock_sec;
    return it->second.obstacle_side;
  }
  it->second.obstacle_side = side_now;
  it->second.side_value0 = side_value;
  it->second.lock_until = now_sec + cfg.side_lock_sec;
  return it->second.obstacle_side;
}

int PathShiftPlanner::preferredPass(double side_value, double d_block, int obstacle_side) const {
  (void)d_block;
  if (!cfg.prefer_right) {
    // 레거시: 장애물 반대편(빈 쪽)으로 회피
    if (obstacle_side > 0) return -1;
    if (obstacle_side < 0) return +1;
    return -1;
  }
  // 우측 우선: 장애물이 경로보다 right_switch_side 이상 오른쪽(side_value<0)이면 좌측, 아니면 우측.
  if (side_value < -cfg.right_switch_side) return +1;  // 확실히 오른쪽 → 좌측
  return -1;                                           // 왼쪽/중앙/약간 오른쪽 → 우측(선호)
}

int PathShiftPlanner::getLockedPass(int uid, int desired_pass, double now_sec) {
  auto it = side_lock_map_.find(uid);
  if (it == side_lock_map_.end()) return desired_pass;  // 엔트리 없음(비정상): 그대로
  SideLockInfo& info = it->second;
  if (!info.pass_set) {
    info.pass_sign = desired_pass;
    info.pass_set = true;
    return desired_pass;
  }
  if (now_sec < info.lock_until) return info.pass_sign;  // lock 중 유지(방향 핑퐁 방지)
  info.pass_sign = desired_pass;                          // lock 만료 → 갱신
  return desired_pass;
}

void PathShiftPlanner::cleanupSideLocks(double now_sec) {
  std::vector<int> to_erase;
  to_erase.reserve(side_lock_map_.size());
  for (const auto& kv : side_lock_map_) {
    if ((now_sec - kv.second.last_seen) > cfg.track_lost_sec) to_erase.push_back(kv.first);
  }
  for (int uid : to_erase) side_lock_map_.erase(uid);
}

void PathShiftPlanner::updateFSM(const std::vector<Candidate>& candidates, double now_sec) {
  const bool has_any = !candidates.empty();
  if (state_ == PlannerState::NORMAL) {
    if (has_any) state_ = PlannerState::AVOID;
  } else if (state_ == PlannerState::AVOID) {
    if (!has_any) {
      state_ = PlannerState::RETURN;
      return_start_time_ = now_sec;
    }
  } else if (state_ == PlannerState::RETURN) {
    if (has_any) {
      state_ = PlannerState::AVOID;
    } else if ((now_sec - return_start_time_) > cfg.t_return_hold) {
      state_ = PlannerState::NORMAL;
    }
  }
}

std::vector<double> PathShiftPlanner::resampleOffsetByS(const std::vector<double>& old_S,
                                                        const std::vector<double>& old_offset,
                                                        const std::vector<double>& new_S) const {
  std::vector<double> out;
  out.reserve(new_S.size());
  if (old_S.size() < 2 || old_S.size() != old_offset.size() || new_S.empty()) {
    out.assign(new_S.size(), 0.0);
    return out;
  }
  std::size_t j = 0;
  for (double s : new_S) {
    if (s <= old_S.front()) { out.push_back(old_offset.front()); continue; }
    if (s >= old_S.back()) { out.push_back(old_offset.back()); continue; }
    while (j + 1 < old_S.size() && old_S[j + 1] < s) j++;
    std::size_t j2 = std::min(j + 1, old_S.size() - 1);
    double s1 = old_S[j], s2 = old_S[j2];
    double o1 = old_offset[j], o2 = old_offset[j2];
    double t = (s2 > s1) ? (s - s1) / (s2 - s1) : 0.0;
    out.push_back(o1 + t * (o2 - o1));
  }
  if (out.size() != new_S.size()) out.resize(new_S.size(), 0.0);
  return out;
}

void PathShiftPlanner::buildSoftBounds(const std::vector<Candidate>& candidates,
                                       const std::vector<double>& S, std::vector<double>& lo,
                                       std::vector<double>& hi, std::vector<double>& lo_score,
                                       std::vector<double>& hi_score) const {
  const std::size_t N = S.size();
  for (const auto& c : candidates) {
    if (c.gain <= 1e-6) continue;
    if (!std::isfinite(c.cinfo.s_star)) continue;
    const double s_i = c.cinfo.s_star;
    const double d_i = c.side_value;
    const double R_i = c.d_block;
    const double s_start = s_i - cfg.pre_dist;
    const double s_end = s_i + cfg.post_dist;
    for (std::size_t k = 0; k < N; ++k) {
      const double s = S[k];
      if (s > s_end || s > cfg.s_plan_max) break;  // S 단조증가 → 조기종료
      if (s < 0.0 || s < s_start) continue;
      const double w = weightForS(s, s_start, s_end);
      if (w <= 0.0) continue;
      const double R_eff = R_i * w * c.gain;
      const double score = std::abs(s - s_i);
      if (c.pass_sign < 0) {
        const double new_hi = d_i - R_eff;
        if (new_hi < hi[k]) { hi[k] = new_hi; hi_score[k] = score; }
      } else {
        const double new_lo = d_i + R_eff;
        if (new_lo > lo[k]) { lo[k] = new_lo; lo_score[k] = score; }
      }
    }
  }
  for (std::size_t k = 0; k < N; ++k) {
    lo[k] = clampd(lo[k], -cfg.max_shift, cfg.max_shift);
    hi[k] = clampd(hi[k], -cfg.max_shift, cfg.max_shift);
  }
}

void PathShiftPlanner::buildHardBounds(const std::vector<Candidate>& candidates,
                                       const std::vector<double>& S, std::vector<double>& lo,
                                       std::vector<double>& hi, std::vector<double>& lo_score,
                                       std::vector<double>& hi_score) const {
  const std::size_t N = S.size();
  for (const auto& c : candidates) {
    if (!std::isfinite(c.cinfo.s_star)) continue;
    const double s_i = c.cinfo.s_star;
    const double d_i = c.side_value;
    const double R_i = c.d_block + std::max(0.0, cfg.hard_long_margin);
    if (R_i <= 1e-6) continue;
    for (std::size_t k = 0; k < N; ++k) {
      const double s = S[k];
      if (s > s_i + R_i || s > cfg.s_plan_max) break;  // S 단조증가 → 조기종료
      if (s < 0.0 || s < s_i - R_i) continue;
      const double ds = s - s_i;
      const double lat_req = std::sqrt(std::max(0.0, R_i * R_i - ds * ds));
      const double score = std::abs(ds);
      if (c.pass_sign < 0) {
        const double new_hi = d_i - lat_req;
        if (new_hi < hi[k]) { hi[k] = new_hi; hi_score[k] = score; }
      } else {
        const double new_lo = d_i + lat_req;
        if (new_lo > lo[k]) { lo[k] = new_lo; lo_score[k] = score; }
      }
    }
  }
  for (std::size_t k = 0; k < N; ++k) {
    lo[k] = clampd(lo[k], -cfg.max_shift, cfg.max_shift);
    hi[k] = clampd(hi[k], -cfg.max_shift, cfg.max_shift);
  }
}

void PathShiftPlanner::fixBoundsConflicts(std::vector<double>& lo, std::vector<double>& hi,
                                          const std::vector<double>& lo_score,
                                          const std::vector<double>& hi_score) const {
  const std::size_t N = lo.size();
  for (std::size_t k = 0; k < N; ++k) {
    if (k >= hi.size()) break;
    if (lo[k] <= hi[k]) continue;
    if (lo_score[k] <= hi_score[k])
      hi[k] = cfg.max_shift;
    else
      lo[k] = -cfg.max_shift;
    if (lo[k] > hi[k]) { lo[k] = -cfg.max_shift; hi[k] = cfg.max_shift; }
  }
}

void PathShiftPlanner::applyClusterPassPolicy(std::vector<Candidate>& candidates,
                                              const std::vector<double>& S,
                                              const std::vector<double>& offset_prev) const {
  if (candidates.size() < 2) return;
  if (cfg.cluster_s_eps <= 1e-6) return;

  std::sort(candidates.begin(), candidates.end(),
            [](const Candidate& a, const Candidate& b) {
              return a.cinfo.s_star < b.cinfo.s_star;
            });

  auto nearestOffsetAtS = [&](double s_query) -> double {
    if (S.empty() || offset_prev.size() != S.size()) return 0.0;
    std::size_t best_k = 0;
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t k = 0; k < S.size(); ++k) {
      double d = std::abs(S[k] - s_query);
      if (d < best) { best = d; best_k = k; }
    }
    return offset_prev[best_k];
  };

  std::size_t start = 0;
  int gid = 0;
  while (start < candidates.size()) {
    std::size_t end = start + 1;
    const double s0 = candidates[start].cinfo.s_star;
    while (end < candidates.size() && (candidates[end].cinfo.s_star - s0) <= cfg.cluster_s_eps) end++;

    for (std::size_t i = start; i < end; ++i) {
      candidates[i].cluster_id = gid;
      candidates[i].cluster_barrier = false;
      candidates[i].cluster_overrode = false;
    }
    if (end - start >= 2) {
      bool has_left = false, has_right = false, mixed_pass = false;
      const int pass0 = candidates[start].pass_sign;
      double s_sum = 0.0;
      for (std::size_t i = start; i < end; ++i) {
        s_sum += candidates[i].cinfo.s_star;
        if (candidates[i].obstacle_side > 0) has_left = true;
        if (candidates[i].obstacle_side < 0) has_right = true;
        if (candidates[i].pass_sign != pass0) mixed_pass = true;
      }
      const double s_g = s_sum / static_cast<double>(end - start);

      double lo_def = -cfg.max_shift, hi_def = cfg.max_shift;
      for (std::size_t i = start; i < end; ++i) {
        const double d_i = candidates[i].side_value;
        const double R_i = candidates[i].d_block + std::max(0.0, cfg.hard_long_margin);
        if (candidates[i].pass_sign > 0)
          lo_def = std::max(lo_def, d_i + R_i);
        else
          hi_def = std::min(hi_def, d_i - R_i);
      }
      const bool conflict_default = (lo_def > hi_def);
      const bool barrier_like = conflict_default && mixed_pass;

      if (barrier_like || (has_left && has_right && conflict_default)) {
        for (std::size_t i = start; i < end; ++i) candidates[i].cluster_barrier = true;
        const double prev_g = nearestOffsetAtS(s_g);

        double lo_L = -cfg.max_shift;
        for (std::size_t i = start; i < end; ++i) {
          const double d_i = candidates[i].side_value;
          const double R_i = candidates[i].d_block + std::max(0.0, cfg.hard_long_margin);
          lo_L = std::max(lo_L, d_i + R_i);
        }
        const double hi_L = cfg.max_shift;
        const bool feas_L = (lo_L <= hi_L);
        const double proj_L = feas_L ? clampd(prev_g, lo_L, hi_L) : prev_g;
        const double cost_L = feas_L ? std::abs(proj_L - prev_g) : std::numeric_limits<double>::infinity();

        double hi_R = cfg.max_shift;
        for (std::size_t i = start; i < end; ++i) {
          const double d_i = candidates[i].side_value;
          const double R_i = candidates[i].d_block + std::max(0.0, cfg.hard_long_margin);
          hi_R = std::min(hi_R, d_i - R_i);
        }
        const double lo_R = -cfg.max_shift;
        const bool feas_R = (lo_R <= hi_R);
        const double proj_R = feas_R ? clampd(prev_g, lo_R, hi_R) : prev_g;
        const double cost_R = feas_R ? std::abs(proj_R - prev_g) : std::numeric_limits<double>::infinity();

        const int pass_group = (cost_L <= cost_R) ? +1 : -1;
        for (std::size_t i = start; i < end; ++i) {
          const int before = candidates[i].pass_sign;
          candidates[i].pass_sign = pass_group;
          candidates[i].cluster_overrode = (before != candidates[i].pass_sign);
        }
      }
    }
    start = end;
    gid++;
  }
}

double PathShiftPlanner::weightForS(double s, double s_start, double s_end) const {
  if (s < s_start || s > s_end) return 0.0;
  const double enter_end = s_start + cfg.ramp_in_len;
  const double exit_start = s_end - cfg.ramp_out_len;
  if (s <= enter_end) {
    double u = (cfg.ramp_in_len > 1e-6) ? (s - s_start) / cfg.ramp_in_len : 1.0;
    return smoothstep(u);
  }
  if (s >= exit_start) {
    double u = (cfg.ramp_out_len > 1e-6) ? (s_end - s) / cfg.ramp_out_len : 1.0;
    return smoothstep(u);
  }
  return 1.0;
}

void PathShiftPlanner::smoothOffsets(std::vector<double>& io, int win) {
  // 이동평균(반경 win)을 prefix-sum 으로 O(N) in-place 계산. 매 iter 새 vector 할당 제거.
  const int N = static_cast<int>(io.size());
  if (N == 0 || win <= 0) return;
  prefix_buf_.assign(N + 1, 0.0);
  for (int i = 0; i < N; ++i) prefix_buf_[i + 1] = prefix_buf_[i] + io[i];
  for (int i = 0; i < N; ++i) {
    const int lo = std::max(0, i - win), hi = std::min(N - 1, i + win);
    io[i] = (prefix_buf_[hi + 1] - prefix_buf_[lo]) / static_cast<double>(hi - lo + 1);
  }
}

void PathShiftPlanner::enforceSlopeLimit(std::vector<double>& offset, const std::vector<double>& S,
                                         double max_doffset_ds) const {
  if (offset.size() < 2 || S.size() != offset.size()) return;
  if (max_doffset_ds <= 1e-6) return;
  for (std::size_t k = 1; k < offset.size(); ++k) {
    const double ds = S[k] - S[k - 1];
    if (ds <= 1e-6) continue;
    const double md = max_doffset_ds * ds;
    offset[k] = clampd(offset[k], offset[k - 1] - md, offset[k - 1] + md);
  }
  for (std::size_t kk = offset.size(); kk-- > 1;) {
    const std::size_t k = kk - 1;
    const double ds = S[k + 1] - S[k];
    if (ds <= 1e-6) continue;
    const double md = max_doffset_ds * ds;
    offset[k] = clampd(offset[k], offset[k + 1] - md, offset[k + 1] + md);
  }
}

std::vector<double> PathShiftPlanner::solveOffset1D(const std::vector<double>& init,
                                                    const std::vector<double>& S,
                                                    const std::vector<double>& lo_hard,
                                                    const std::vector<double>& hi_hard) {
  std::vector<double> offset = init;
  if (offset.size() != S.size() || offset.size() != lo_hard.size() || offset.size() != hi_hard.size())
    return offset;
  const int iters = std::max(1, cfg.solver_iters);
  for (int it = 0; it < iters; ++it) {
    if (cfg.smooth_window > 0) smoothOffsets(offset, cfg.smooth_window);  // in-place
    enforceSlopeLimit(offset, S, cfg.max_doffset_ds);
    for (std::size_t k = 0; k < offset.size(); ++k)
      offset[k] = clampd(offset[k], lo_hard[k], hi_hard[k]);
  }
  return offset;
}

std::vector<Vec2> PathShiftPlanner::applyOffset(const std::vector<Vec2>& P,
                                                const std::vector<double>& offset) {
  std::vector<Vec2> out = P;
  if (P.size() != offset.size() || P.size() < 2) return out;
  for (std::size_t i = 0; i < P.size(); ++i) {
    Vec2 t = tangentAt(i, P);  // static
    Vec2 n(-t.y, t.x);
    out[i].x = P[i].x + offset[i] * n.x;
    out[i].y = P[i].y + offset[i] * n.y;
  }
  return out;
}

PathShiftResult PathShiftPlanner::plan(const std::vector<Vec2>& P,
                                       const std::vector<Obstacle>& obs, double now_sec) {
  PathShiftResult result;
  // 시간 역행(bag 재시작/클록 점프) 감지 → 상태 리셋(side-lock 누수·FSM 정지 방지)
  if (last_now_ >= 0.0 && now_sec < last_now_ - 1e-3) reset();
  last_now_ = now_sec;

  const std::size_t N = P.size();
  std::vector<double> S = cumulativeS(P);

  // offset 연속성: 경로가 바뀌어도 이전 offset을 s축으로 재샘플링
  if (prev_S_.size() >= 2 && offset_current_.size() == prev_S_.size() && N >= 2) {
    offset_current_ = resampleOffsetByS(prev_S_, offset_current_, S);
  } else {
    offset_current_.assign(N, 0.0);
  }
  prev_S_ = S;

  const double total_len = S.empty() ? 0.0 : S.back();
  // 유효 경로 미달/비정상(NaN·Inf) → passthrough. (경로점 NaN 은 total_len 이 NaN 이 되어 걸러짐)
  if (static_cast<int>(N) < cfg.min_points || !std::isfinite(total_len) ||
      total_len < cfg.min_path_len) {
    state_ = PlannerState::NORMAL;
    offset_current_.assign(N, 0.0);
    cleanupSideLocks(now_sec);
    result.valid = false;
    result.state = state_;
    result.offset = offset_current_;
    return result;
  }

  std::vector<double> offset_prev = offset_current_;
  std::vector<Candidate> candidates = buildCandidates(obs, P, S, now_sec);
  applyClusterPassPolicy(candidates, S, offset_prev);
  updateFSM(candidates, now_sec);

  std::vector<double> lo_soft(N, -cfg.max_shift), hi_soft(N, cfg.max_shift);
  std::vector<double> lo_hard(N, -cfg.max_shift), hi_hard(N, cfg.max_shift);
  const double INF = std::numeric_limits<double>::infinity();
  std::vector<double> lo_soft_sc(N, INF), hi_soft_sc(N, INF), lo_hard_sc(N, INF), hi_hard_sc(N, INF);

  if (state_ == PlannerState::AVOID) {
    buildSoftBounds(candidates, S, lo_soft, hi_soft, lo_soft_sc, hi_soft_sc);
    buildHardBounds(candidates, S, lo_hard, hi_hard, lo_hard_sc, hi_hard_sc);
    fixBoundsConflicts(lo_soft, hi_soft, lo_soft_sc, hi_soft_sc);
    fixBoundsConflicts(lo_hard, hi_hard, lo_hard_sc, hi_hard_sc);
  }

  if (offset_prev.size() != N) offset_prev.assign(N, 0.0);

  std::vector<double> desired(N, 0.0);
  if (state_ == PlannerState::AVOID) {
    for (std::size_t k = 0; k < N; ++k) desired[k] = clampd(offset_prev[k], lo_soft[k], hi_soft[k]);
  }

  std::vector<double> offset_sol = desired;
  for (std::size_t k = 0; k < N; ++k) offset_sol[k] = clampd(offset_sol[k], lo_hard[k], hi_hard[k]);
  offset_sol = solveOffset1D(offset_sol, S, lo_hard, hi_hard);

  const double k_follow =
      (state_ == PlannerState::AVOID) ? cfg.filter_k_avoid : cfg.filter_k_return;
  std::vector<double> offset_new = offset_prev;
  for (std::size_t k = 0; k < N; ++k) {
    offset_new[k] = offset_prev[k] + k_follow * (offset_sol[k] - offset_prev[k]);
    offset_new[k] = clampd(offset_new[k], -cfg.max_shift, cfg.max_shift);
  }

  offset_current_ = offset_new;

  // blocked: 통로가 좁아 max_shift 로도 hard bound 를 만족 못 하는 활성 후보가 있는가 → 상위에 정지 신호
  bool blocked = false;
  if (state_ == PlannerState::AVOID) {
    for (const auto& c : candidates) {
      if (c.gain <= 1e-6) continue;
      const double R = c.d_block + std::max(0.0, cfg.hard_long_margin);
      if (c.pass_sign < 0) {
        if (c.side_value - R < -cfg.max_shift - 1e-6) blocked = true;
      } else {
        if (c.side_value + R > cfg.max_shift + 1e-6) blocked = true;
      }
    }
  }

  cleanupSideLocks(now_sec);

  result.valid = true;
  result.blocked = blocked;
  result.state = state_;
  result.offset = offset_current_;
  result.candidates.reserve(candidates.size());
  for (const auto& c : candidates) {
    CandidateDebug d;
    d.id = c.unique_id;
    d.pos_x = c.pos.x; d.pos_y = c.pos.y;
    d.q_x = c.cinfo.q.x; d.q_y = c.cinfo.q.y;
    d.s_star = c.cinfo.s_star; d.d_min = c.cinfo.d_min;
    d.side_value = c.side_value; d.d_block = c.d_block;
    d.obstacle_side = c.obstacle_side;
    d.pass_sign_raw = c.pass_sign_raw; d.pass_sign = c.pass_sign;
    d.cluster_id = c.cluster_id;
    d.cluster_barrier = c.cluster_barrier; d.cluster_overrode = c.cluster_overrode;
    // 판단 근거
    const double lane_room = std::max(0.0, cfg.lane_half_width - cfg.veh_half_width);
    const double right_need = std::max(0.0, c.d_block - c.side_value);
    const double left_need = std::max(0.0, c.d_block + c.side_value);
    d.right_need = right_need; d.left_need = left_need; d.lane_room = lane_room;
    if (c.cluster_barrier)
      d.reason = (c.pass_sign < 0 ? "RIGHT(cluster)" : "LEFT(cluster)");
    else if (!cfg.prefer_right)
      d.reason = (c.pass_sign < 0 ? "RIGHT(free-side)" : "LEFT(free-side)");
    else if (c.pass_sign < 0)
      d.reason = "RIGHT(prefer)";
    else
      d.reason = "LEFT(obstacle right of path)";
    result.candidates.push_back(d);
  }
  return result;
}

}  // namespace gpp
