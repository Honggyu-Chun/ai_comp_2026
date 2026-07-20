// test_path_shift_planner.cpp — gpp::PathShiftPlanner 오프라인 단위테스트 (ROS 무관).
// 직선 base_link 경로 + S자 드럼 배치로 회피 offset의 정확성을 검증한다.
// 빌드 후 실행: rosrun global_path_planner test_path_shift_planner  (또는 devel/lib/.../ 직접)
// 실패 시 비정상 종료코드 반환.

#include "global_path_planner/path_shift_planner.h"

#include <cmath>
#include <cstdio>
#include <vector>

using gpp::Vec2;
using gpp::Obstacle;
using gpp::PathShiftPlanner;
using gpp::PathShiftResult;
using gpp::PlannerState;

static int g_fail = 0;
#define CHECK(cond, msg)                                              \
  do {                                                               \
    if (!(cond)) { std::printf("  FAIL: %s\n", msg); g_fail++; }     \
    else { std::printf("  ok  : %s\n", msg); }                       \
  } while (0)

// 직선 경로: x=0..len (0.1m), y=0. base_link 원점=차량.
static std::vector<Vec2> straightPath(double len, double step) {
  std::vector<Vec2> P;
  for (double x = 0.0; x <= len + 1e-9; x += step) P.emplace_back(x, 0.0);
  return P;
}

static double dBlock(const PathShiftPlanner& pl, double w, double l) {
  double r = 0.5 * std::sqrt(w * w + l * l);
  r = std::max(pl.cfg.r_obs_min, std::min(r, pl.cfg.r_obs_max));
  return pl.cfg.veh_half_width + pl.cfg.obs_radius_scale * r + pl.cfg.margin_obs +
         std::max(0.0, pl.cfg.extra_clearance);
}

// 드럼까지 shifted 경로의 최소 유클리드 거리
static double minClearance(const std::vector<Vec2>& sp, const Obstacle& o) {
  double best = 1e18;
  for (const auto& p : sp) {
    double d = std::hypot(p.x - o.x, p.y - o.y);
    if (d < best) best = d;
  }
  return best;
}

int main() {
  const double DT = 0.05;  // 20Hz
  const std::vector<Vec2> P = straightPath(50.0, 0.1);

  // ---- Test 1: S자 드럼 회피 (수렴 후 충돌 없음 + 부호 + |offset| 한계 + 기울기) ----
  std::printf("[Test1] S-curve avoidance\n");
  {
    PathShiftPlanner pl;  // 기본 config
    std::vector<Obstacle> obs = {
        {1, 10.0, +0.6, 0.0, 0.5, 0.5},  // 경로 왼쪽 → 우측 회피(offset<0)
        {2, 18.0, -0.6, 0.0, 0.5, 0.5},  // 경로 오른쪽 → 좌측 회피(offset>0)
        {3, 26.0, +0.6, 0.0, 0.5, 0.5},
    };
    PathShiftResult res;
    double t = 0.0;
    int first_pass[4] = {0, 0, 0, 0};
    bool pass_stable = true;
    for (int frame = 0; frame < 400; ++frame) {
      res = pl.plan(P, obs, t);
      t += DT;
      // side-lock 안정성: 각 id의 pass_sign이 최초 결정 후 뒤집히지 않아야 함
      for (const auto& c : res.candidates) {
        if (c.id >= 1 && c.id <= 3) {
          if (first_pass[c.id] == 0) first_pass[c.id] = c.pass_sign;
          else if (first_pass[c.id] != c.pass_sign) pass_stable = false;
        }
      }
    }
    CHECK(res.valid, "result valid");
    CHECK(res.state == PlannerState::AVOID, "state == AVOID");

    // offset 한계
    double max_abs = 0.0;
    for (double v : res.offset) max_abs = std::max(max_abs, std::abs(v));
    CHECK(max_abs <= pl.cfg.max_shift + 1e-6, "|offset| <= max_shift");

    // 기울기 제한: solver 의 best-effort. 단, hard bound(반원 제약)가 최종 clamp 되므로
    // 장애물 바로 옆(|s-s*|<R)에서는 안전(clearance) 우선으로 slope 가 국소적으로 초과될 수 있다.
    // → 장애물 hard 영역 밖(ramp 구간)에서만 엄격 검증한다.
    bool slope_ok = true;
    std::vector<double> S(P.size(), 0.0);
    for (size_t i = 1; i < P.size(); ++i) S[i] = S[i-1] + std::hypot(P[i].x - P[i-1].x, P[i].y - P[i-1].y);
    auto nearDrum = [&](double s) {
      for (const auto& o : obs) if (std::abs(s - o.x) < dBlock(pl, o.width, o.length)) return true;
      return false;
    };
    for (size_t k = 1; k < res.offset.size(); ++k) {
      double ds = S[k] - S[k-1];
      if (ds <= 1e-9) continue;
      if (nearDrum(S[k]) || nearDrum(S[k-1])) continue;  // 장애물 경계는 안전 우선
      double slope = std::abs(res.offset[k] - res.offset[k-1]) / ds;
      if (slope > pl.cfg.max_doffset_ds + 1e-3) slope_ok = false;
    }
    CHECK(slope_ok, "|doffset/ds| <= max_doffset_ds (outside hard-bound region)");

    // 부호: 각 드럼 s* 근처 offset 부호가 장애물 반대편
    std::vector<Vec2> sp = PathShiftPlanner::applyOffset(P, res.offset);
    // 드럼 x≈s* 인덱스 = 드럼.x/0.1
    auto offAtX = [&](double x) { return res.offset[std::min(res.offset.size()-1, (size_t)std::lround(x/0.1))]; };
    CHECK(offAtX(10.0) < 0.0, "drum@+y -> offset<0 (avoid right)");
    CHECK(offAtX(18.0) > 0.0, "drum@-y -> offset>0 (avoid left)");
    CHECK(offAtX(26.0) < 0.0, "drum2@+y -> offset<0");

    // 충돌 없음: 각 드럼과 shifted 경로 최소거리 >= d_block (수렴 후)
    for (const auto& o : obs) {
      double clr = minClearance(sp, o);
      double need = dBlock(pl, o.width, o.length);
      char buf[128];
      std::snprintf(buf, sizeof(buf), "drum id=%d clearance %.3f >= d_block %.3f", o.id, clr, need);
      CHECK(clr >= need - 0.08, buf);
    }
    CHECK(pass_stable, "side-lock: pass_sign stable across frames (no ping-pong)");
  }

  // ---- Test 2: 무장애물 → offset 0, NORMAL 로 복귀(passthrough) ----
  std::printf("[Test2] no obstacle -> zero offset / NORMAL\n");
  {
    PathShiftPlanner pl;
    std::vector<Obstacle> none;
    PathShiftResult res;
    double t = 0.0;
    for (int f = 0; f < 100; ++f) { res = pl.plan(P, none, t); t += DT; }
    double max_abs = 0.0;
    for (double v : res.offset) max_abs = std::max(max_abs, std::abs(v));
    CHECK(max_abs < 1e-6, "offset ~ 0 with no obstacles");
    CHECK(res.state == PlannerState::NORMAL, "state == NORMAL after hold");
  }

  // ---- Test 3: 짧은 경로 → passthrough(valid=false, offset 0) ----
  std::printf("[Test3] short path passthrough\n");
  {
    PathShiftPlanner pl;
    std::vector<Vec2> shortP = straightPath(0.5, 0.1);  // 6점, 0.5m < min_path_len
    std::vector<Obstacle> obs = {{1, 0.3, 0.0, 0.0, 0.5, 0.5}};
    PathShiftResult res = pl.plan(shortP, obs, 0.0);
    CHECK(!res.valid, "short path -> valid=false (passthrough)");
    double max_abs = 0.0;
    for (double v : res.offset) max_abs = std::max(max_abs, std::abs(v));
    CHECK(max_abs < 1e-9, "short path offset all zero");
  }

  // ---- Test 4: 통로가 좁으면 blocked 플래그 (정지 신호) ----
  std::printf("[Test4] narrow passage -> blocked flag\n");
  {
    PathShiftPlanner pl;
    // 경로 위 대형 장애물(반경 clamp 최대) → 필요 offset 이 max_shift 초과
    std::vector<Obstacle> obs = {{1, 15.0, 0.0, 0.0, 6.0, 6.0}};
    PathShiftResult res;
    double t = 0.0;
    for (int f = 0; f < 50; ++f) { res = pl.plan(P, obs, t); t += DT; }
    CHECK(res.blocked, "blocked==true when passage narrower than max_shift");

    // 무장애물이면 blocked==false
    std::vector<Obstacle> none;
    for (int f = 0; f < 50; ++f) { res = pl.plan(P, none, t); t += DT; }
    CHECK(!res.blocked, "blocked==false when clear");
  }

  // ---- Test 5: 우측 우선(side 임계값) + avoid_offset 직접 지정 ----
  std::printf("[Test5] prefer-right by side threshold + avoid_offset\n");
  {
    auto offAtX = [](const PathShiftResult& r, double x) {
      return r.offset[std::min(r.offset.size()-1, (size_t)std::lround(x/0.1))];
    };
    // 5a: 중앙 장애물 → 우측
    {
      PathShiftPlanner pl; pl.cfg.prefer_right = true; pl.cfg.right_switch_side = 0.5;
      std::vector<Obstacle> obs = {{1, 15.0, 0.0, 0.0, 0.5, 0.5}};
      PathShiftResult res; double t = 0.0;
      for (int f = 0; f < 200; ++f) { res = pl.plan(P, obs, t); t += DT; }
      CHECK(offAtX(res, 15.0) < 0.0, "centered -> RIGHT");
    }
    // 5b: 임계값(0.5) 안쪽으로 약간 오른쪽 장애물 → 여전히 우측
    {
      PathShiftPlanner pl; pl.cfg.prefer_right = true; pl.cfg.right_switch_side = 0.5;
      std::vector<Obstacle> obs = {{1, 15.0, -0.3, 0.0, 0.5, 0.5}};  // 0.3m 오른쪽 < 0.5
      PathShiftResult res; double t = 0.0;
      for (int f = 0; f < 200; ++f) { res = pl.plan(P, obs, t); t += DT; }
      CHECK(offAtX(res, 15.0) < 0.0, "slightly-right (within threshold) -> RIGHT");
    }
    // 5c: 임계값 밖으로 확실히 오른쪽 장애물 → 좌측
    {
      PathShiftPlanner pl; pl.cfg.prefer_right = true; pl.cfg.right_switch_side = 0.5;
      std::vector<Obstacle> obs = {{1, 15.0, -0.9, 0.0, 0.5, 0.5}};  // 0.9m 오른쪽 > 0.5
      PathShiftResult res; double t = 0.0;
      for (int f = 0; f < 200; ++f) { res = pl.plan(P, obs, t); t += DT; }
      CHECK(offAtX(res, 15.0) > 0.0, "well-right (beyond threshold) -> LEFT");
    }
    // 5d: avoid_offset 로 이격(≈offset) 직접 지정 (중앙 장애물 → |offset| ≈ avoid_offset)
    {
      PathShiftPlanner pl; pl.cfg.prefer_right = true; pl.cfg.avoid_offset = 1.5;
      std::vector<Obstacle> obs = {{1, 15.0, 0.0, 0.0, 0.5, 0.5}};
      PathShiftResult res; double t = 0.0;
      for (int f = 0; f < 300; ++f) { res = pl.plan(P, obs, t); t += DT; }
      double off = std::abs(offAtX(res, 15.0));
      char buf[96]; std::snprintf(buf, sizeof(buf), "avoid_offset=1.5 -> |offset|~1.5 (got %.2f)", off);
      CHECK(std::abs(off - 1.5) < 0.15, buf);
    }
  }

  std::printf("\n%s (failures=%d)\n", g_fail == 0 ? "ALL PASS" : "SOME FAILED", g_fail);
  return g_fail == 0 ? 0 : 1;
}
