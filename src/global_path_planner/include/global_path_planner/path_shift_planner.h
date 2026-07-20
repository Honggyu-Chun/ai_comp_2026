// path_shift_planner.h
// 정적 장애물 회피 — 경로 법선방향 offset(path shifting) 순수 코어 (ROS 의존 없음).
//
// 설계(decision_architecture.md §4): plan() 안에서 ros:: 금지 → 시간은 now_sec 인자로 주입.
// 좌표 규약: 입력 경로/장애물 모두 base_link (x 전방, y 좌측+, REP-103).
//   법선 n=(-t.y, t.x) = 좌측.  offset>0 = 좌측 이동, offset<0 = 우측 이동.
//   side_value=(O-Q)·n  → +: 장애물이 경로 왼쪽, -: 오른쪽.
//   pass_sign  → +1: 좌측으로 회피(offset+), -1: 우측으로 회피(offset-).
//
// 원본: ~/Downloads/global_path_planner (1)/.../avoid_algorithm.cpp 를 라이브러리로 분리 이식.

#ifndef GLOBAL_PATH_PLANNER_PATH_SHIFT_PLANNER_H
#define GLOBAL_PATH_PLANNER_PATH_SHIFT_PLANNER_H

#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include <unordered_map>

namespace gpp {

struct Vec2 {
  double x{0.0}, y{0.0};
  Vec2() = default;
  Vec2(double _x, double _y) : x(_x), y(_y) {}
  Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
  Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
  Vec2 operator*(double s) const { return {x * s, y * s}; }
};

// 회피 대상 장애물 (base_link, SI). /obstacle_info(katri_msgs/ObstacleInfo)에서 1:1 매핑.
struct Obstacle {
  int id{-1};
  double x{0.0}, y{0.0};      // base_link 위치
  double speed{0.0};          // 속도 크기 [m/s] (정적 판정용)
  double width{0.0};          // [m]
  double length{0.0};         // [m]
};

enum class PlannerState { NORMAL = 0, AVOID = 1, RETURN = 2 };

// 모든 튜닝 파라미터 (기본값 = 원본 코드 기본값). 노드가 ROS param으로 채워 넣는다.
struct PathShiftConfig {
  // 기본값 = 검증된 튜닝값(obstacle_avoid.launch 와 동일) → rosrun(launch 없이)도 안전.
  double veh_half_width = 0.95;
  double margin_obs = 0.50;

  double x_front_min = 0.5;
  double x_plan_max = 30.0;
  double s_plan_max = 30.0;

  double v_static_max = 0.5;
  double side_deadband = 0.20;
  double max_shift = 3.0;

  // 회피 방향 정책: 웬만하면 우측(offset<0). 장애물이 경로보다 right_switch_side 이상
  // "오른쪽"에 있을 때만 좌측으로 전환한다. (side_value < -right_switch_side → 좌측)
  bool   prefer_right = true;
  double right_switch_side = 0.5;   // [m] 이 값까지 오른쪽 장애물도 우측으로 피함(클수록 우측 고집)

  // 회피 시 장애물과 유지할 횡방향 이격[m] (= 대략 경로 offset 크기). 0 이면 자동 계산
  // (veh_half_width + r_obs + margin_obs + extra_clearance). >0 이면 이 값으로 직접 지정.
  // 안전을 위해 물리 최소치(veh_half_width + r_obs) 아래로는 내려가지 않는다.
  double avoid_offset = 0.0;

  // 시각화용(차선 경계선 그리기). 판단에는 사용하지 않음.
  double lane_half_width = 1.75;

  double pre_dist = 15.0;
  double post_dist = 25.0;
  double ramp_in_len = 12.0;
  double ramp_out_len = 18.0;
  double soften_margin = 0.5;

  double cluster_s_eps = 1.0;

  int solver_iters = 8;
  int smooth_window = 4;
  double max_doffset_ds = 0.6;
  double hard_long_margin = 0.0;

  double filter_k_avoid = 0.70;
  double filter_k_return = 0.25;

  double obs_radius_scale = 1.0;
  double extra_clearance = 0.10;

  double t_return_hold = 0.5;
  double side_lock_sec = 4.0;
  double track_lost_sec = 1.0;

  double r_obs_min = 0.2;
  double r_obs_max = 3.0;

  // 유효 경로 최소 조건 (미달 시 passthrough)
  int min_points = 10;
  double min_path_len = 5.0;
};

// 마커/로그용 후보 디버그 정보 (노드가 시각화).
struct CandidateDebug {
  int id = -1;
  double pos_x = 0.0, pos_y = 0.0;
  double q_x = 0.0, q_y = 0.0;
  double s_star = 0.0, d_min = 0.0, side_value = 0.0, d_block = 0.0;
  int obstacle_side = 0, pass_sign_raw = 0, pass_sign = 0, cluster_id = -1;
  bool cluster_barrier = false, cluster_overrode = false;
  // 회피 방향 판단 근거 (왜 좌/우로 갔는지)
  double right_need = 0.0;   // 우측으로 피하는 데 필요한 center 이동량[m]
  double left_need = 0.0;    // 좌측으로 피하는 데 필요한 center 이동량[m]
  double lane_room = 0.0;    // 차선 안에서 한쪽으로 이동 가능한 여유[m]
  std::string reason;        // "RIGHT(in-lane)" / "LEFT(right over lane)" / "LEFT(less shift)" 등
};

struct PathShiftResult {
  bool valid = false;                 // false = passthrough(경로 미변형 / 무효 입력)
  bool blocked = false;               // true = 통로가 좁아 max_shift 로도 충분히 못 피함(정지 필요)
  PlannerState state = PlannerState::NORMAL;
  std::vector<double> offset;         // 포인트별 횡오프셋 [m] (P와 같은 길이)
  std::vector<CandidateDebug> candidates;
};

class PathShiftPlanner {
 public:
  PathShiftConfig cfg;

  // 메인 진입점: base_link 경로 P 와 장애물 obs, 현재시각 now_sec[초] → offset(s).
  // 내부에 FSM/side-lock/offset 연속성 상태를 유지하므로 매 프레임 호출한다.
  PathShiftResult plan(const std::vector<Vec2>& P,
                       const std::vector<Obstacle>& obs,
                       double now_sec);

  // offset(s)를 경로에 법선방향으로 적용 (순수 헬퍼). 노드가 nav_msgs/Path 생성에 사용.
  static std::vector<Vec2> applyOffset(const std::vector<Vec2>& P,
                                       const std::vector<double>& offset);

  PlannerState state() const { return state_; }
  void reset();

 private:
  struct ClosestInfo {
    double d_min = std::numeric_limits<double>::infinity();
    double s_star = std::numeric_limits<double>::infinity();
    int seg_idx = -1;
    double t = 0.0;
    Vec2 q;
  };
  struct Candidate {
    int unique_id = -1;
    Vec2 pos;
    double speed = 0.0;
    double r_obs = 0.0;
    double d_block = 0.0;
    ClosestInfo cinfo;
    double side_value = 0.0;
    int obstacle_side = 0;
    int pass_sign = 0;
    double gain = 0.0;
    int pass_sign_raw = 0;
    int cluster_id = -1;
    bool cluster_barrier = false;
    bool cluster_overrode = false;
  };
  struct SideLockInfo {
    int obstacle_side = 0;
    double lock_until = 0.0;
    double last_seen = 0.0;
    double side_value0 = 0.0;
    int pass_sign = 0;        // 잠긴 회피 방향(핑퐁 방지)
    bool pass_set = false;
  };

  // 상태 (프레임 간 유지)
  PlannerState state_ = PlannerState::NORMAL;
  double return_start_time_ = 0.0;
  double last_now_ = -1.0;   // 직전 now_sec (시간 역행 감지용)
  std::vector<double> offset_current_;   // 시간필터된 현재 오프셋
  std::vector<double> prev_S_;           // 직전 프레임 누적거리 (연속성 재샘플용)
  std::unordered_map<int, SideLockInfo> side_lock_map_;

  static std::vector<double> cumulativeS(const std::vector<Vec2>& P);
  ClosestInfo computeClosestToPath(const Vec2& O, const std::vector<Vec2>& P,
                                   const std::vector<double>& S) const;
  static Vec2 tangentAt(std::size_t i, const std::vector<Vec2>& P);
  std::vector<Candidate> buildCandidates(const std::vector<Obstacle>& obs,
                                         const std::vector<Vec2>& P,
                                         const std::vector<double>& S,
                                         double now_sec);
  int getLockedObstacleSide(int uid, double side_value, double now_sec);
  int preferredPass(double side_value, double d_block, int obstacle_side) const;
  int getLockedPass(int uid, int desired_pass, double now_sec);
  void cleanupSideLocks(double now_sec);
  void updateFSM(const std::vector<Candidate>& candidates, double now_sec);
  std::vector<double> resampleOffsetByS(const std::vector<double>& old_S,
                                        const std::vector<double>& old_offset,
                                        const std::vector<double>& new_S) const;
  void buildSoftBounds(const std::vector<Candidate>& c, const std::vector<double>& S,
                       std::vector<double>& lo, std::vector<double>& hi,
                       std::vector<double>& lo_score, std::vector<double>& hi_score) const;
  void buildHardBounds(const std::vector<Candidate>& c, const std::vector<double>& S,
                       std::vector<double>& lo, std::vector<double>& hi,
                       std::vector<double>& lo_score, std::vector<double>& hi_score) const;
  void fixBoundsConflicts(std::vector<double>& lo, std::vector<double>& hi,
                          const std::vector<double>& lo_score,
                          const std::vector<double>& hi_score) const;
  void applyClusterPassPolicy(std::vector<Candidate>& candidates,
                              const std::vector<double>& S,
                              const std::vector<double>& offset_prev) const;
  double weightForS(double s, double s_start, double s_end) const;
  void smoothOffsets(std::vector<double>& io, int win);   // in-place, prefix-sum O(N)
  void enforceSlopeLimit(std::vector<double>& offset, const std::vector<double>& S,
                         double max_doffset_ds) const;
  std::vector<double> solveOffset1D(const std::vector<double>& init,
                                    const std::vector<double>& S,
                                    const std::vector<double>& lo_hard,
                                    const std::vector<double>& hi_hard);

  std::vector<double> prefix_buf_;   // smoothOffsets 재사용 스크래치
};

}  // namespace gpp

#endif  // GLOBAL_PATH_PLANNER_PATH_SHIFT_PLANNER_H
