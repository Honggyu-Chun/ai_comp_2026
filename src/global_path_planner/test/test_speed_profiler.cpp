// test_speed_profiler.cpp — gpp::SpeedProfiler 오프라인 단위테스트 (ROS 무관).
// Phase1: 곡률감속 + 제동 포락선.  Phase2: ADAS 앞차추종 + 전방 정지장애물.

#include "global_path_planner/speed_profiler.h"

#include <cmath>
#include <cstdio>
#include <vector>

using gpp::Vec2;
using gpp::Obstacle;
using gpp::SpeedProfiler;
using gpp::SpeedProfileResult;

static int g_fail = 0;
#define CHECK(cond, msg)                                          \
  do {                                                            \
    if (!(cond)) { std::printf("  FAIL: %s\n", msg); g_fail++; }  \
    else { std::printf("  ok  : %s\n", msg); }                    \
  } while (0)

static std::vector<Vec2> straight(double len, double step) {
  std::vector<Vec2> P;
  for (double x = 0.0; x <= len + 1e-9; x += step) P.emplace_back(x, 0.0);
  return P;
}
// 직선 straight_len 뒤 반경 R 좌회전 아크(길이 arc_len)
static std::vector<Vec2> straightThenArc(double straight_len, double R, double arc_len, double step) {
  std::vector<Vec2> P;
  for (double x = 0.0; x <= straight_len + 1e-9; x += step) P.emplace_back(x, 0.0);
  const double cx = straight_len, cy = R;         // 좌회전 중심
  const double da = step / R;
  for (double a = -M_PI / 2.0; a <= -M_PI / 2.0 + arc_len / R; a += da)
    P.emplace_back(cx + R * std::cos(a), cy + R * std::sin(a));
  return P;
}
static double maxKph(const std::vector<double>& v) { double m=0; for(double x:v) m=std::max(m,x); return m; }
static double minKph(const std::vector<double>& v) { double m=1e9; for(double x:v) m=std::min(m,x); return m; }

int main() {
  // ---- Test 1: 직선 → 전 구간 max_speed ----
  std::printf("[Test1] straight -> max_speed\n");
  {
    SpeedProfiler pf;  // 기본 cfg (max 60, a_lat 2.2, sf 0.82 ...)
    SpeedProfileResult r = pf.plan(straight(40.0, 0.25), {}, 0.0);
    CHECK(r.valid, "valid");
    CHECK(std::abs(maxKph(r.speed_kph) - pf.cfg.max_speed_kph) < 0.5, "max == 60kph");
    CHECK(std::abs(minKph(r.speed_kph) - pf.cfg.max_speed_kph) < 0.5, "min == 60kph (no curve)");
  }

  // ---- Test 2: 곡선 → 곡률감속 + 진입 전 제동 ----
  std::printf("[Test2] curve -> lateral-accel speed + braking approach\n");
  {
    SpeedProfiler pf;
    const double R = 15.0;
    std::vector<Vec2> P = straightThenArc(30.0, R, 20.0, 0.25);
    SpeedProfileResult r = pf.plan(P, {}, 0.0);
    // 기대 곡률속도 = sqrt(a_lat_eff / (1/R)) = sqrt(a_lat_eff * R)
    const double a_lat_eff = pf.cfg.curve_speed_safety_factor * pf.cfg.lateral_accel_limit_mps2;
    const double v_curve_kph = std::sqrt(a_lat_eff * R) * 3.6;  // ≈18.7
    CHECK(std::abs(maxKph(r.speed_kph) - 60.0) < 1.0, "some point still 60 (straight start)");
    char buf[96];
    std::snprintf(buf, sizeof(buf), "min ~ curve speed %.1fkph (got %.1f)", v_curve_kph, minKph(r.speed_kph));
    CHECK(std::abs(minKph(r.speed_kph) - v_curve_kph) < 0.20 * v_curve_kph, buf);
    // 곡선 진입 전 제동 단조 비증가 확인 (직선 초반 60 → 곡선 감속)
    bool has_decrease = false;
    for (std::size_t i = 1; i < r.speed_kph.size(); ++i)
      if (r.speed_kph[i] < r.speed_kph[i-1] - 0.1) has_decrease = true;
    CHECK(has_decrease, "braking region present (speed decreases into curve)");
  }

  // ---- Test 3: min_curve 하한 (아주 급한 곡선) ----
  std::printf("[Test3] sharp curve clamped to minimum_curve_speed\n");
  {
    SpeedProfiler pf;
    std::vector<Vec2> P = straightThenArc(10.0, 2.0, 6.0, 0.2);  // R=2 매우 급
    SpeedProfileResult r = pf.plan(P, {}, 0.0);
    CHECK(minKph(r.speed_kph) >= pf.cfg.minimum_curve_speed_kph - 0.5, "min >= minimum_curve_speed");
  }

  // ---- Test 4: ADAS 앞차 추종 ----
  std::printf("[Test4] ACC following\n");
  {
    SpeedProfiler pf;
    std::vector<Vec2> P = straight(60.0, 0.25);
    // 선행차: x=40, y=0(차로 내), 속도 5m/s, 길이 4.6. 자차 8m/s.
    std::vector<Obstacle> obs = {{7, 40.0, 0.0, 5.0, 1.9, 4.6}};
    SpeedProfileResult r = pf.plan(P, obs, 8.0);
    // gap=40-2.5-2.3=35.2, desired=6+1.5*8=18, v=5+0.4*(35.2-18)=11.88 m/s=42.8kph
    CHECK(r.lead_id == 7, "lead detected (id=7)");
    char buf[96]; std::snprintf(buf, sizeof(buf), "follow ~42.8kph (got %.1f)", r.v_follow_kph);
    CHECK(std::abs(r.v_follow_kph - 42.77) < 2.0, buf);
    CHECK(minKph(r.speed_kph) < 59.0, "profile capped below 60 by ACC");

    // 옆 차로 장애물(y=5)는 선행차 아님 → 무제한(60)
    std::vector<Obstacle> side = {{8, 40.0, 5.0, 5.0, 1.9, 4.6}};
    SpeedProfileResult r2 = pf.plan(P, side, 8.0);
    CHECK(r2.lead_id == -1, "off-lane obstacle is not lead");
    CHECK(std::abs(maxKph(r2.speed_kph) - 60.0) < 0.5 && std::abs(minKph(r2.speed_kph) - 60.0) < 0.5,
          "no lead -> 60kph");
  }

  // ---- Test 5: 전방 정지 장애물 ----
  std::printf("[Test5] static obstacle ahead -> stop\n");
  {
    SpeedProfiler pf;
    std::vector<Vec2> P = straight(30.0, 0.2);
    std::vector<Obstacle> obs = {{9, 8.0, 0.0, 0.0, 0.5, 0.5}};  // 정지 장애물 x=8
    SpeedProfileResult r = pf.plan(P, obs, 5.0);
    CHECK(minKph(r.speed_kph) < 1.0, "stops (some point ~0)");
    CHECK(r.speed_kph.front() > 1.0, "braking before stop (start speed > 0)");
    // 정지점 이후는 0
    CHECK(r.speed_kph.back() < 1.0, "beyond stop point speed ~0");
  }

  std::printf("\n%s (failures=%d)\n", g_fail == 0 ? "ALL PASS" : "SOME FAILED", g_fail);
  return g_fail == 0 ? 0 : 1;
}
