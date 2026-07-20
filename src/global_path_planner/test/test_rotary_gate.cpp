// test_rotary_gate.cpp — gpp::RotaryGate 오프라인 단위테스트 (ROS 무관).

#include "global_path_planner/rotary_gate.h"

#include <cmath>
#include <cstdio>
#include <vector>

using gpp::RotaryGate;
using gpp::RotaryGateConfig;
using gpp::AgentState;
using gpp::EgoState;
using gpp::RotaryDecision;

static int g_fail = 0;
#define CHECK(cond, msg)                                          \
  do {                                                            \
    if (!(cond)) { std::printf("  FAIL: %s\n", msg); g_fail++; }  \
    else { std::printf("  ok  : %s\n", msg); }                    \
  } while (0)

// 중심(cx,cy) 반경 R 위 각도 phi 의 CCW(+1)/CW(-1) 순환차
static AgentState ringAgent(double cx, double cy, double R, double phi, double sp, int dir, int id) {
  AgentState a; a.id = id;
  a.x = cx + R * std::cos(phi); a.y = cy + R * std::sin(phi);
  if (dir > 0) { a.vx = -sp * std::sin(phi); a.vy = sp * std::cos(phi); }   // CCW 접선
  else         { a.vx = sp * std::sin(phi);  a.vy = -sp * std::cos(phi); }  // CW 접선
  return a;
}
// 정지선에 선 자차 (충돌점까지 s_stopline_to_conflict, v=0), 6시(-pi/2) 진입 가정
static EgoState egoAtStopline(const RotaryGateConfig& c) {
  EgoState e; e.v_mps = 0.0;
  const double s = c.ring_radius_m + c.s_stopline_to_conflict_m;  // 중심에서 진입 반대방향(아래) 바깥
  e.x = c.center_x + s * std::cos(c.theta_entry_rad);
  e.y = c.center_y + s * std::sin(c.theta_entry_rad);
  e.yaw_rad = 0.0;
  return e;
}

int main() {
  const double R = 15.0, cx = 0.0, cy = 0.0;

  // ---- Test 1: 무에이전트 → 연속확정 후 COMMIT ----
  std::printf("[Test1] no agents -> GO -> COMMIT\n");
  {
    RotaryGate g; g.cfg.center_x=cx; g.cfg.center_y=cy; g.cfg.ring_radius_m=R;
    EgoState e = egoAtStopline(g.cfg);
    RotaryDecision d;
    for (int i = 0; i < 10; ++i) d = g.update(e, {}, /*s*/6.0, i * 0.033);
    CHECK(d.phase == RotaryDecision::COMMIT || d.phase == RotaryDecision::INSIDE, "committed with no agents");
    CHECK(d.stop_wall_s < 0.0, "stop_wall released on commit");
  }

  // ---- Test 2: 도착창에 에이전트 → 거부(HOLD 유지, 정지벽 유지) ----
  std::printf("[Test2] agent in arrival window -> reject/HOLD\n");
  {
    RotaryGate g; g.cfg.center_x=cx; g.cfg.center_y=cy; g.cfg.ring_radius_m=R;
    EgoState e = egoAtStopline(g.cfg);
    // 진입각 바로 앞(작은 dtheta)에 CCW 차 → ETA 작아 도착창 침범
    const double sp = 5.0, omega = sp / R;
    const double phi = g.cfg.theta_entry_rad - g.cfg.dir * (omega * 3.0);  // ETA≈3s
    std::vector<AgentState> ags = { ringAgent(cx,cy,R,phi,sp,+1,1) };
    RotaryDecision d;
    for (int i = 0; i < 10; ++i) d = g.update(e, ags, 6.0, i * 0.033);
    CHECK(d.phase == RotaryDecision::HOLD, "stays HOLD (not committed)");
    CHECK(d.stop_wall_s >= 0.0, "stop wall held");
    CHECK(d.blocking_agent_id == 1, "blocking agent identified");
  }

  // ---- Test 3: 4대 90° CCW → 갭 주기적 개방, 결국 진입 ----
  std::printf("[Test3] 4 agents 90deg CCW -> gap opens, eventually COMMIT\n");
  {
    RotaryGate g; g.cfg.center_x=cx; g.cfg.center_y=cy; g.cfg.ring_radius_m=R;
    EgoState e = egoAtStopline(g.cfg);
    const double sp = 5.0, omega = sp / R;  // gap 간격 (pi/2)/omega ≈4.7s > 필요창 → 열림
    double t = 0.0; const double dt = 0.05;
    bool committed = false;
    for (int i = 0; i < 400 && !committed; ++i) {  // 20s
      std::vector<AgentState> ags;
      for (int k = 0; k < 4; ++k)
        ags.push_back(ringAgent(cx, cy, R, omega * t + k * (M_PI / 2.0), sp, +1, k));
      RotaryDecision d = g.update(e, ags, 6.0, t);
      if (d.phase == RotaryDecision::COMMIT || d.phase == RotaryDecision::INSIDE) committed = true;
      t += dt;
    }
    CHECK(committed, "found a gap and committed within 20s");
  }

  // ---- Test 4: 동일거리서 v0=0 도달시각 > v0>0 (사다리꼴 발진) ----
  std::printf("[Test4] lower v0 -> later arrival (same distance)\n");
  {
    // 같은 s=20, 막는 차 하나로 APPROACH 유지, ego 속도만 다르게.
    const double sp=5.0, om=sp/R; const double phi=-M_PI/2 - (om*0.2);
    std::vector<AgentState> blk={ ringAgent(cx,cy,R,phi,sp,+1,9) };
    RotaryGate g0; g0.cfg.center_x=cx;g0.cfg.center_y=cy;g0.cfg.ring_radius_m=R;
    EgoState e0=egoAtStopline(g0.cfg); e0.v_mps=0.0;
    RotaryDecision d0=g0.update(e0, blk, 20.0, 0.0);
    RotaryGate g5; g5.cfg=g0.cfg; EgoState e5=egoAtStopline(g5.cfg); e5.v_mps=5.0;
    RotaryDecision d5=g5.update(e5, blk, 20.0, 0.0);
    char buf[96]; std::snprintf(buf,sizeof(buf),"t_arrive v0=0 (%.2f) > v0=5 (%.2f)", d0.t_ego_arrive_s, d5.t_ego_arrive_s);
    CHECK(d0.t_ego_arrive_s > d5.t_ego_arrive_s, buf);
  }

  // ---- Test 5: COMMIT 래치 (이후 위험차에도 정지벽 미생성) ----
  std::printf("[Test5] COMMIT latch\n");
  {
    RotaryGate g; g.cfg.center_x=cx; g.cfg.center_y=cy; g.cfg.ring_radius_m=R;
    EgoState e = egoAtStopline(g.cfg);
    RotaryDecision d;
    for (int i=0;i<10;++i) d=g.update(e,{},6.0,i*0.033);  // 무에이전트 → COMMIT
    CHECK(d.phase==RotaryDecision::COMMIT||d.phase==RotaryDecision::INSIDE, "in commit/inside");
    // 위험차 주입
    const double sp=5.0, om=sp/R; const double phi=g.cfg.theta_entry_rad - g.cfg.dir*(om*0.1);
    std::vector<AgentState> danger={ ringAgent(cx,cy,R,phi,sp,+1,7) };
    bool re_stopped=false;
    for (int i=0;i<10;++i){ RotaryDecision dd=g.update(e,danger,6.0,(10+i)*0.033); if(dd.stop_wall_s>=0.0) re_stopped=true; }
    CHECK(!re_stopped, "never re-creates stop wall after COMMIT");
  }

  // ---- Test 6: 방향 데이터 역산 (CW 차는 dir=+1 에서 제외, dir=-1 에서 인식) ----
  std::printf("[Test6] direction from cross product\n");
  {
    const double sp=5.0, om=sp/R; const double phi=-M_PI/2 - (om*3.0);
    std::vector<AgentState> cw = { ringAgent(cx,cy,R,phi,sp,-1,3) };  // CW 차
    // dir=+1: CW 차 제외 → 무에이전트처럼 accept → COMMIT
    { RotaryGate g; g.cfg.center_x=cx;g.cfg.center_y=cy;g.cfg.ring_radius_m=R; g.cfg.dir=+1;
      EgoState e=egoAtStopline(g.cfg); RotaryDecision d;
      for(int i=0;i<10;++i) d=g.update(e,cw,6.0,i*0.033);
      CHECK(d.phase==RotaryDecision::COMMIT||d.phase==RotaryDecision::INSIDE, "CW agent excluded when dir=+1 -> commit"); }
    // dir=-1: CW 차 인식 → 도착창 침범 → 거부
    { RotaryGate g; g.cfg.center_x=cx;g.cfg.center_y=cy;g.cfg.ring_radius_m=R; g.cfg.dir=-1;
      g.cfg.theta_entry_rad=-M_PI/2;
      // dir=-1 이면 진입각 도달 dtheta 계산이 반대 → CW 차가 진입각으로 옴. phi 재설정.
      std::vector<AgentState> cw2 = { ringAgent(cx,cy,R, -M_PI/2 + (om*3.0), sp, -1, 3) };
      EgoState e=egoAtStopline(g.cfg); RotaryDecision d;
      for(int i=0;i<10;++i) d=g.update(e,cw2,6.0,i*0.033);
      CHECK(d.phase==RotaryDecision::HOLD, "CW agent blocks when dir=-1"); }
  }

  // ---- Test 7: 데드락 완화 (timeout 후에만 진입) ----
  std::printf("[Test7] deadlock relax\n");
  {
    RotaryGate g; g.cfg.center_x=cx;g.cfg.center_y=cy;g.cfg.ring_radius_m=R;
    g.cfg.deadlock_timeout_s = 2.0; g.cfg.relaxed_scale = 0.3; g.cfg.go_confirm_frames = 3;
    EgoState e = egoAtStopline(g.cfg);
    // 완화 전 도착창은 침범, 완화 후엔 벗어나도록 ETA 를 창 경계 근처에 배치
    // HOLD 에서 ta≈2.83, tc≈4.0. 정상창 [1.83,5.5], 완화창(×0.3) [2.53,4.45]. ETA=2.0 배치.
    const double sp=5.0, om=sp/R; const double eta_target=2.0;
    const double phi=g.cfg.theta_entry_rad - g.cfg.dir*(om*eta_target);
    std::vector<AgentState> ag = { ringAgent(cx,cy,R,phi,sp,+1,1) };
    bool committed_before=false, committed_after=false;
    for (int i=0;i<200;++i){
      double t=i*0.033;
      RotaryDecision d=g.update(e,ag,6.0,t);
      bool c=(d.phase==RotaryDecision::COMMIT||d.phase==RotaryDecision::INSIDE);
      if (t < 2.0 && c) committed_before=true;
      if (t >= 2.2 && c) committed_after=true;
    }
    CHECK(!committed_before, "not committed before deadlock timeout");
    CHECK(committed_after, "committed after relax");
  }

  // ---- Test 8: 랩어라운드 (theta_entry ~ pi) 무튐 ----
  std::printf("[Test8] wraparound near +-pi\n");
  {
    RotaryGate g; g.cfg.center_x=cx;g.cfg.center_y=cy;g.cfg.ring_radius_m=R;
    g.cfg.theta_entry_rad = M_PI - 0.01; g.cfg.theta_exit_rad = M_PI - 0.01 + M_PI/2;
    EgoState e; e.v_mps=0; const double s=R+g.cfg.s_stopline_to_conflict_m;
    e.x=cx+s*std::cos(g.cfg.theta_entry_rad); e.y=cy+s*std::sin(g.cfg.theta_entry_rad); e.yaw_rad=0;
    RotaryDecision d;
    for(int i=0;i<10;++i) d=g.update(e,{},6.0,i*0.033);
    CHECK(std::isfinite(d.speed_cap_kph)||d.speed_cap_kph<0, "no NaN across wraparound");
    CHECK(d.phase==RotaryDecision::COMMIT||d.phase==RotaryDecision::INSIDE, "commits near pi entry");
  }

  // ---- Test 9: v0>v_ring 클램프 ----
  std::printf("[Test9] v0 > v_ring arrival finite/positive\n");
  {
    RotaryGate g; g.cfg.center_x=cx;g.cfg.center_y=cy;g.cfg.ring_radius_m=R; g.cfg.v_ring_kph=20.0;
    EgoState e=egoAtStopline(g.cfg); e.v_mps=20.0;  // 72kph >> v_ring 5.56
    const double sp=5.0, om=sp/R; const double phi=g.cfg.theta_entry_rad - g.cfg.dir*(om*0.2);
    std::vector<AgentState> blk={ ringAgent(cx,cy,R,phi,sp,+1,9) };
    RotaryDecision d=g.update(e, blk, 20.0, 0.0);  // APPROACH, s=20
    char buf[96]; std::snprintf(buf,sizeof(buf),"t_arrive finite>0 (%.2f)", d.t_ego_arrive_s);
    CHECK(std::isfinite(d.t_ego_arrive_s) && d.t_ego_arrive_s > 0.0, buf);
  }

  // ---- Test 10: 자차가 물리적으로 멀면 FAR 유지 — 전방투영 s 노이즈 무관 (조기 COMMIT 방지) ----
  std::printf("[Test10] no false activation when ego physically far (s noisy)\n");
  {
    RotaryGate g; g.cfg.center_x=cx;g.cfg.center_y=cy;g.cfg.ring_radius_m=R;
    g.cfg.theta_entry_rad=-M_PI/2; g.cfg.activate_dist_m=40.0; g.cfg.go_confirm_frames=3;
    EgoState e; e.v_mps=5.0; e.yaw_rad=0.0;
    e.x=cx; e.y=cy-200.0;    // 진입점(cx,cy-R)에서 ~185m — 실제로 멂
    RotaryDecision d;
    // s 를 작게(5) 줘 노이즈로 "가까운 척" 해도, 유클리드 거리로 FAR 유지해야 함
    for(int i=0;i<10;++i) d=g.update(e,{},5.0,i*0.033);
    CHECK(d.phase==RotaryDecision::FAR, "physically-far ego stays FAR despite small noisy s");
    CHECK(d.speed_cap_kph<0.0, "no cap/commit when far");
  }

  // ---- Test 11: INSIDE 는 물리적으로 링 안일 때만 (s 크다고 INSIDE 되면 안 됨) ----
  std::printf("[Test11] INSIDE gated by physical ring position\n");
  {
    RotaryGate g; g.cfg.center_x=cx;g.cfg.center_y=cy;g.cfg.ring_radius_m=R;
    g.cfg.theta_entry_rad=-M_PI/2; g.cfg.theta_exit_rad=0.0; g.cfg.dir=1;
    g.cfg.go_confirm_frames=3;
    EgoState e=egoAtStopline(g.cfg);  // 링 밖(r=R+6)
    RotaryDecision d;
    for(int i=0;i<6;++i) d=g.update(e,{},6.0,i*0.033);
    CHECK(d.phase==RotaryDecision::COMMIT, "committed at stopline (empty)");
    // 링 밖에 머무는데 s만 크게 → INSIDE 금지
    for(int i=0;i<10;++i) d=g.update(e,{},150.0,(6+i)*0.033);
    CHECK(d.phase==RotaryDecision::COMMIT, "stays COMMIT outside ring even at s=150");
    // 링 안을 진입각→진출각으로 실제 이동 → INSIDE 거쳐 DONE
    const int N=20; bool saw_inside=false;
    for(int k=0;k<=N;++k){
      double th=g.cfg.theta_entry_rad + (g.cfg.theta_exit_rad-g.cfg.theta_entry_rad)*k/N;
      EgoState ei; ei.v_mps=3.0; ei.yaw_rad=0.0;
      ei.x=cx+R*std::cos(th); ei.y=cy+R*std::sin(th);
      d=g.update(ei,{},10.0,(20+k)*0.033);
      if(d.phase==RotaryDecision::INSIDE) saw_inside=true;
    }
    CHECK(saw_inside, "INSIDE while physically in ring");
    CHECK(d.phase==RotaryDecision::DONE, "DONE after sweeping to exit angle");
  }

  // ---- Test 12: 갭이 열려도 commit zone 밖이면 COMMIT 금지 (조기진입 방지) ----
  std::printf("[Test12] no COMMIT outside commit zone even if gap open\n");
  {
    RotaryGate g; g.cfg.center_x=cx;g.cfg.center_y=cy;g.cfg.ring_radius_m=R;
    g.cfg.theta_entry_rad=-M_PI/2; g.cfg.activate_dist_m=40.0; g.cfg.commit_dist_m=20.0;
    g.cfg.go_confirm_frames=3;
    auto egoOnEntryRay=[&](double extra){ EgoState e; e.v_mps=5.0; e.yaw_rad=0.0;
      const double r=R+extra;
      e.x=cx+r*std::cos(g.cfg.theta_entry_rad); e.y=cy+r*std::sin(g.cfg.theta_entry_rad); return e; };
    RotaryDecision d;
    EgoState farOpen=egoOnEntryRay(30.0);  // dist_to_entry=30 > commit_dist=20, 에이전트 없음(갭 열림)
    for(int i=0;i<10;++i) d=g.update(farOpen,{},30.0,i*0.033);
    CHECK(d.phase==RotaryDecision::APPROACH, "gap open but outside zone -> APPROACH (no early COMMIT)");
    EgoState nearOpen=egoOnEntryRay(15.0); // dist=15 <= commit_dist
    for(int i=0;i<6;++i) d=g.update(nearOpen,{},15.0,(10+i)*0.033);
    CHECK(d.phase==RotaryDecision::COMMIT||d.phase==RotaryDecision::INSIDE,
          "inside commit zone with open gap -> COMMIT");
  }

  std::printf("\n%s (failures=%d)\n", g_fail == 0 ? "ALL PASS" : "SOME FAILED", g_fail);
  return g_fail == 0 ? 0 : 1;
}
