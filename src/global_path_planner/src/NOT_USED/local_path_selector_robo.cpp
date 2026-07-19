#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
// 최근접2대 장애물 투영 + 경로 시각화 마커
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <array>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

// 로컬 경로 선택에서 사용하는 메시지
#include "roborace_msgs/Objects.h"
// 최근접2대 장애물(Objects) 기반 좌우 경로 판별 입력
#include "roborace_msgs/NearestVehicles2.h"

// 최근접2대 장애물 및 경로 선택 노드
class PathSelect
{
public:
    // ===== ROS 인터페이스 =====
    // - 구독 토픽
    //   - /local_path1      : 기본 경로 1
    //   - /local_path2      : 기본 경로 2
    //   - /current_path     : 현재 경로 선택 인덱스
    //   - /region           : 상태(go/warning/overtake)
    //   - /obstacle_info    : 최근접 2대 장애물 정보
    //   - /camera_stop      : 카메라 상태(true면 stop override)
    // - 발행 토픽
    //   - /selected_path    : 최종 선택 경로
    //   - /decision         : 제어 의사결정 문자열
    //   - debug_marker_topic: RViz 마커(디버그)
    ros::Subscriber local_path1_sub, local_path2_sub, current_path_sub, state_sub, nearest_vehicles_sub, nearest_vehicles2_sub, camera_stop_sub;
    ros::Publisher selected_path_pub, speed_control_pub, debug_marker_pub;

    // ===== 경로/상태 =====
    // 수신된 원본 경로
    nav_msgs::Path local_path1, local_path2, select_path;
    // ego 차량 현재 자세 (현재는 x,y만 활용)
    geometry_msgs::Point current_pose;
    // 선택된 경로 프레임
    std::string frame_id;
    // 상태 문자열: go / warning / slow / curve / overtake
    std::string state = "go";
    // camera_stop: object_select가 true면 카메라 비정상으로 판단한 상태
    std::string camera_stop_topic_ = "/camera_stop";
    bool camera_stop_active_ = false;

    // ===== 판별 플래그 =====
    // front_obstacle: 전방 장애물 존재(현재 차로)
    // side_obstacle:  인접 차로 장애물 존재
    bool front_obstacle = false, side_obstacle = false;

    // ===== 선택 경로 정보 =====
    // current_path: 외부에서 주는 현재 경로 인덱스(1/2)
    // selected_path_number: 최종 발행 경로(차선 변경 반영 포함)
    int current_path = 1, selected_path_number = 0;
    // warning 구간 및 post-warning 윈도우 동안 유지할 경로 인덱스
    int warning_locked_path_ = 1;
    bool warning_path_lock_active_ = false;

    // ===== 거리/포인트 =====
    // cal_distance: 전방 장애물 거리(로컬 경로 상 s 차이)
    // path_x, path_y: 전방 장애물을 대응시킬 경로상의 좌표
    double cal_distance = 0.0, INF = 99.9;
    double path_x = std::numeric_limits<double>::quiet_NaN();
    double path_y = std::numeric_limits<double>::quiet_NaN();

    // ===== 최근접2대 모듈 상수 =====
    // LANE_THRES: 차선 판별 여유거리 임계
    // LANE_THRES2: 위 임계의 제곱, 거리 제곱 비교용
    // EPS: 수치 안정성용
    // DEFAULT_CURVE_FIXED_PATH: curve 구간 고정 경로의 기본값
    static constexpr double LANE_THRES = 1.5;
    static constexpr double LANE_THRES2 = LANE_THRES * LANE_THRES;
    static constexpr double EPS = 1e-9;
    static constexpr int DEFAULT_CURVE_FIXED_PATH = 2;

    // x, y 좌표를 담는 기본 2차원 점 타입
    struct XY {
        double x = 0.0;
        double y = 0.0;
    };

    // 경로 캐시(누적거리 기반 투영 최적화용)
    // - pts: 경로 점열(원본 좌표)
    // - seg: 각 구간 벡터(pts[i+1]-pts[i])
    // - seg_len: 구간 길이
    // - seg_len2: 구간 길이의 제곱(투영 계산 가속)
    // - cumlen: 각 구간 누적거리
    // - total_len: 전체 경로 길이
    // - ego_s: 원점(0,0) 투영시의 ego 누적거리
    // - valid / ego_valid: 캐시 유효성
    struct PathCache {
        std::vector<XY> pts;
        std::vector<XY> seg;
        std::vector<double> seg_len;
        std::vector<double> seg_len2;
        std::vector<double> cumlen;
        double total_len = 0.0;
        double ego_s = 0.0;
        bool valid = false;
        bool ego_valid = false;
    };

    // 점->경로 투영 결과
    // - dist2: 최소 거리 제곱
    // - proj: 가장 가까운 투영 좌표
    // - s: 누적거리 좌표계 위치
    // - seg_idx/t: 해당 구간 인덱스 및 보간 파라미터
    struct ProjectionResult {
        bool valid = false;
        double dist2 = std::numeric_limits<double>::infinity();
        XY proj;
        double s = 0.0;
        int seg_idx = -1;
        double t = 0.0;
    };

    // 장애물 원본/해석(한 슬롯) 결과
    // - input_valid: 현재 슬롯 사용 여부
    // - ambiguous_lane: 양쪽 경로와의 경계 사이 구간 판별 여부
    // - lane_thres_ok: 양쪽 경로 간 거리 임계 내 포함 여부
    // - is_front: 현재 차로 기준 전방(front)인지 여부
    // - raw_lane_id: 거리기반 임시 차로 판정
    // - lane_id: 최종 확정 차로 판정
    // - delta_s_lane: 장애물-ego s 차이
    // - obs_s_lane / ego_s_lane: 해당 차로 기준 누적거리
    // - lane_proj: 최종 차로에 매핑된 투영 결과
    struct ObjEval {
        bool input_valid = false;
        bool ambiguous_lane = false;
        bool lane_thres_ok = false;
        bool is_front = false;
        int raw_lane_id = 0;
        int lane_id = 0;
        double delta_s_lane = std::numeric_limits<double>::infinity();
        double obs_s_lane = 0.0;
        double ego_s_lane = 0.0;
        XY raw;
        ProjectionResult proj1;
        ProjectionResult proj2;
        ProjectionResult lane_proj;
    };

    // front/side 판단과 경로 좌표, 그림자 점까지 합친 결정 결과
    // - front_obstacle: 최종 전방 장애물 존재
    // - side_obstacle: 최종 측방(타차로) 장애물 존재
    // - front_slot: 전방 판단 슬롯(0/1)
    // - front_candidate: 전방 후보 플래그
    // - shadow_valid: warning 모드 그림자 위치 계산 성공 여부
    // - shadow_points: warning 모드에서 경로 상 대응점
    struct Decision {
        bool front_obstacle = false;
        bool side_obstacle = false;
        double path_x = std::numeric_limits<double>::quiet_NaN();
        double path_y = std::numeric_limits<double>::quiet_NaN();
        double cal_distance = 99.9;
        int front_slot = -1;
        std::array<bool, 2> front_candidate{{false, false}};
        std::array<bool, 2> shadow_valid{{false, false}};
        std::array<XY, 2> shadow_points;
    };

    // /obstacle_info 입력 스냅샷
    // one/two 슬롯 각각의 존재 여부 및 좌표를 저장한다.
    struct NearestVehiclesInput {
        bool received = false;
        bool one = false;
        bool two = false;
        double one_x = 0.0;
        double one_y = 0.0;
        double two_x = 0.0;
        double two_y = 0.0;
    };

    // 대표 장애물 로그(콘솔/디버깅 표시용)
    // 조건 충족 시, 로그/디버그에서 하나만 대표로 표시.
    struct RepresentativeObstacleLog {
        bool valid = false;
        int slot = -1; // 0:one, 1:two
        int lane_id = 0;
        bool lane_thres_ok = false;
        bool is_front = false;
        double delta_s_lane = std::numeric_limits<double>::infinity();
        double raw_x = 0.0;
        double raw_y = 0.0;
        double proj_x = 0.0;
        double proj_y = 0.0;
        double lateral_dist = std::numeric_limits<double>::infinity();
    };

    // 슬롯별(원시)로그
    // one/two 각각의 역할(front/side/none) 상태를 추적.
    struct ObjectSlotLog {
        bool exists = false;
        bool valid = false;
        int lane_id = 0;
        bool lane_thres_ok = false;
        bool is_front = false;
        double x = 0.0;
        double y = 0.0;
        std::string role = "none"; // front / side / none
    };

    // 병합 경계 곡선 캐시
    // 병합 구간/병행 구간 모두에서 경계 시각화를 위한 네 개 경계선.
    struct SplitBoundaryCurves {
        std::vector<XY> lane1_outer;
        std::vector<XY> lane1_inner;
        std::vector<XY> lane2_inner;
        std::vector<XY> lane2_outer;
        bool valid = false;
    };

    // ===== 내부 상태 =====
    // 경로 캐시
    PathCache path1_cache_;
    PathCache path2_cache_;
    // 병합 경계 캐시
    SplitBoundaryCurves split_boundary_cache_;
    // 최근 장애물 입력 스냅샷
    NearestVehiclesInput nearest_input_;
    // 슬롯별 최근 차로 판정 히스테리시스(미확정 구간 유지용)
    std::array<int, 2> lane_hold_{{0, 0}}; // 0:없음, 1:path1, 2:path2
    // launch 파라미터: ambiguous인 경우 항상 현재 차로로 고정
    bool force_ambiguous_to_current_lane_ = false;
    int curve_fixed_path_ = DEFAULT_CURVE_FIXED_PATH;
    // 디버그용 최근 처리 로그
    RepresentativeObstacleLog rep_obstacle_log_;
    bool side_obstacle_log_ = false;
    std::array<ObjectSlotLog, 2> object_slot_logs_;
    // 업데이트 플래그: 새 입력이 들어왔는지 여부
    bool projection_dirty_ = true;
    // 최근 계산 스냅샷(디버그 마커 재발행용)
    std::array<ObjEval, 2> last_obj_evals_;
    Decision last_decision_;
    int last_debug_current_lane_ = 1;
    bool debug_snapshot_valid_ = false;

    // 디버그 마커 제어 상태
    bool enable_debug_markers_ = true;
    bool debug_marker_prev_enabled_ = false;
    std::string debug_marker_topic_ = "/local_path_selector/debug_markers";
    double debug_marker_hz_ = 10.0;
    std::string debug_marker_frame_id_ = "base_link";
    ros::Time last_debug_marker_pub_time_;

    bool isWarningRegionState(const std::string& region) const
    {
        return region == "warning" || region == "Warning";
    }

    bool isCurveRegionState(const std::string& region) const
    {
        return region == "curve" || region == "Curve";
    }

    bool isWarningLikeRegionState(const std::string& region) const
    {
        return isWarningRegionState(region) || isCurveRegionState(region);
    }

    int normalizeLaneId(int lane_id) const
    {
        return (lane_id == 2) ? 2 : 1;
    }

    int getEffectiveCurrentPath() const
    {
        // [curve 고정 경로] curve 구간은 path1/path2가 겹친 단일 차선으로 간주하므로
        // 내부 current_lane 계산과 selected_path 선택 모두 curve_fixed_path_ 기준으로 통일한다.
        if (isCurveRegionState(state)) {
            return curve_fixed_path_;
        }
        return warning_path_lock_active_ ? warning_locked_path_ : current_path;
    }

    int getCurveFixedPath() const
    {
        return curve_fixed_path_;
    }

    /**
     * warning 구간에서 사용할 경로 잠금 상태를 갱신한다.
     * - active == true  : warning 진입 시점의 경로(locked_path)를 저장하고, 이후 내부 판단/current_lane 계산과
     *                     selected_path 선택이 모두 이 경로를 기준으로 동작하게 만든다.
     * - active == false : warning 종료 또는 차선 변경 시작 시 잠금을 해제하고, 다시 외부 /current_path 값을 사용한다.
     * - locked_path 값은 1/2만 허용되며, 그 외 값은 안전하게 1로 정규화한다.
     * - 잠금 상태나 잠금 경로가 바뀌면 markProjectionDirty()를 호출해 장애물 투영/차로 판단을 즉시 다시 계산한다.
     */
    void setWarningPathLock(bool active, int locked_path)
    {
        const int normalized_path = normalizeLaneId(locked_path);
        const bool changed = (warning_path_lock_active_ != active) ||
                             (active && warning_locked_path_ != normalized_path);
        warning_path_lock_active_ = active;
        if (active) {
            warning_locked_path_ = normalized_path;
        }
        if (changed) {
            markProjectionDirty();
        }
    }

    bool hasLaneFrontObstacleInRange(int lane_id, double min_raw_x, double max_raw_x) const
    {
        for (const auto& eval : last_obj_evals_) {
            if (!eval.input_valid || !eval.lane_thres_ok || !eval.lane_proj.valid) {
                continue;
            }
            if (eval.lane_id != lane_id) {
                continue;
            }
            if (eval.raw.x <= min_raw_x || eval.raw.x >= max_raw_x) {
                continue;
            }
            return true;
        }
        return false;
    }

    bool isLaneOccupiedInSnapshot(int lane_id) const
    {
        for (const auto& eval : last_obj_evals_) {
            if (!eval.input_valid || !eval.lane_thres_ok || !eval.lane_proj.valid) {
                continue;
            }
            if (eval.lane_id != lane_id) {
                continue;
            }
            if (eval.raw.x <= 0.0) {
                continue;
            }
            return true;
        }
        return false;
    }

    /**
     * 경로 메시지를 내부 캐시로 변환한다.
     * - seg, seg_len, cumlen을 구성해 향후 투영 계산을 O(1) 근사로 처리
     * - 차량(원점) 기반 ego_s와 유효성(ego_valid)을 갱신한다.
     */
    void buildPathCache(const nav_msgs::Path& path_msg, PathCache& cache)
    {
        cache.pts.clear();
        cache.seg.clear();
        cache.seg_len.clear();
        cache.seg_len2.clear();
        cache.cumlen.clear();
        cache.total_len = 0.0;
        cache.ego_s = 0.0;
        cache.valid = false;
        cache.ego_valid = false;

        // 경로 점이 2개 미만이면 구간 기반 계산이 불가하므로 캐시 무효 상태로 유지
        if (path_msg.poses.size() < 2) {
            return;
        }

        cache.pts.reserve(path_msg.poses.size());
        for (const auto& pose_stamped : path_msg.poses) {
            XY pt;
            pt.x = pose_stamped.pose.position.x;
            pt.y = pose_stamped.pose.position.y;
            cache.pts.push_back(pt);
        }

        const size_t seg_count = cache.pts.size() - 1;
        cache.seg.resize(seg_count);
        cache.seg_len.resize(seg_count);
        cache.seg_len2.resize(seg_count);
        cache.cumlen.resize(seg_count + 1, 0.0);
        double best_ego_d2 = std::numeric_limits<double>::infinity();

        // 세그먼트 벡터/길이/누적거리 및 ego 기준 s를 한 번에 계산한다.
        for (size_t i = 0; i < seg_count; ++i) {
            const XY& a = cache.pts[i];
            const XY& b = cache.pts[i + 1];
            XY v;
            v.x = b.x - a.x;
            v.y = b.y - a.y;
            cache.seg[i] = v;

            const double len2 = v.x * v.x + v.y * v.y;
            const double len = std::sqrt(len2);
            cache.seg_len2[i] = len2;
            cache.seg_len[i] = len;
            cache.cumlen[i + 1] = cache.cumlen[i] + len;

            if (len2 > EPS) {
                double t = (-(a.x * v.x + a.y * v.y)) / len2;
                t = std::max(0.0, std::min(1.0, t));
                const double proj_x = a.x + t * v.x;
                const double proj_y = a.y + t * v.y;
                const double d2 = proj_x * proj_x + proj_y * proj_y;
                if (d2 < best_ego_d2) {
                    best_ego_d2 = d2;
                    cache.ego_valid = true;
                    cache.ego_s = cache.cumlen[i] + t * len;
                }
            }
        }

        cache.total_len = cache.cumlen.back();
        cache.valid = cache.total_len > EPS;
        if (!cache.valid) {
            cache.ego_valid = false;
            cache.ego_s = 0.0;
        }
    }

    /**
     * 장애물의 차로 판별이 애매한 경우(ambiguous)에 대한 최종 차로 선택.
     * - force_ambiguous_to_current_lane 우선
     * - warning 모드에서는 current_lane 고정
     * - 히스테리시스(lane_hold_) 유지 시 이전 판정 유지
     */
    int resolveLaneForAmbiguous(size_t slot_idx,
                                bool ambiguous_lane,
                                int raw_lane_id,
                                int current_lane,
                                bool warning_mode)
    {
        if (!ambiguous_lane) {
            return raw_lane_id;
        }

        // 1) 파라미터 강제 모드: ambiguous일 때 항상 현재 차로 유지
        if (force_ambiguous_to_current_lane_) {
            return current_lane;
        }

        // 2) warning 상태에서는 갑작스런 전환보다 현재 차로 우선 고정
        if (warning_mode) {
            return current_lane;
        }

        // 3) 히스테리시스: 직전 판정이 남아 있으면 유지해 번들링 제거
        if (slot_idx < lane_hold_.size()) {
            const int hold_lane = lane_hold_[slot_idx];
            if (hold_lane == 1 || hold_lane == 2) {
                return hold_lane;
            }
        }

        return current_lane;
    }

    /**
     * 점을 경로 세그먼트들에 투영해, 최단거리 점을 찾는다.
     */
    ProjectionResult projectPointToPath(const XY& p, const PathCache& cache) const
    {
        ProjectionResult best;
        if (!cache.valid || cache.seg.empty()) {
            return best;
        }

        // 모든 세그먼트에 대해 직교투영거리 최소점을 찾아 가장 가까운 경로 점으로 사용
        for (size_t i = 0; i < cache.seg.size(); ++i) {
            if (cache.seg_len2[i] <= EPS) {
                continue;
            }

            const XY& a = cache.pts[i];
            const XY& v = cache.seg[i];
            const double apx = p.x - a.x;
            const double apy = p.y - a.y;
            double t = (apx * v.x + apy * v.y) / cache.seg_len2[i];
            t = std::max(0.0, std::min(1.0, t));

            XY proj;
            proj.x = a.x + t * v.x;
            proj.y = a.y + t * v.y;

            const double dx = p.x - proj.x;
            const double dy = p.y - proj.y;
            const double d2 = dx * dx + dy * dy;

            if (d2 < best.dist2) {
                best.valid = true;
                best.dist2 = d2;
                best.proj = proj;
                best.s = cache.cumlen[i] + t * cache.seg_len[i];
                best.seg_idx = static_cast<int>(i);
                best.t = t;
            }
        }

        return best;
    }

    /**
     * 경로 누적거리 s에서 선형 보간으로 좌표를 샘플링한다.
     */
    XY samplePathAtS(const PathCache& cache, double target_s) const
    {
        if (!cache.valid || cache.pts.empty()) {
            return XY{};
        }

        if (target_s <= 0.0) {
            return cache.pts.front();
        }
        if (target_s >= cache.total_len) {
            return cache.pts.back();
        }

        // target_s의 구간 위치를 upper_bound로 찾은 뒤, 선형 보간으로 내부 점 산출
        auto upper = std::upper_bound(cache.cumlen.begin(), cache.cumlen.end(), target_s);
        size_t seg_idx = static_cast<size_t>(std::distance(cache.cumlen.begin(), upper));
        if (seg_idx == 0) {
            seg_idx = 1;
        }
        seg_idx -= 1;
        if (seg_idx >= cache.seg.size()) {
            seg_idx = cache.seg.size() - 1;
        }

        const double seg_s = target_s - cache.cumlen[seg_idx];
        double t = 0.0;
        if (cache.seg_len[seg_idx] > EPS) {
            t = seg_s / cache.seg_len[seg_idx];
        }
        t = std::max(0.0, std::min(1.0, t));

        XY sampled;
        sampled.x = cache.pts[seg_idx].x + t * cache.seg[seg_idx].x;
        sampled.y = cache.pts[seg_idx].y + t * cache.seg[seg_idx].y;
        return sampled;
    }

    /**
     * 경로에서 누적거리 s 위치의 접선 단위벡터를 구한다.
     * 병합 구간 경계 생성에서 보조로 사용한다.
     */
    XY sampleDirAtS(const PathCache& cache, double target_s) const
    {
        if (!cache.valid || cache.seg.empty()) {
            return XY{1.0, 0.0};
        }
        // 경계 생성에서 사용되는 접선 방향 단위벡터 계산
        auto upper = std::upper_bound(cache.cumlen.begin(), cache.cumlen.end(), target_s);
        size_t seg_idx = static_cast<size_t>(std::distance(cache.cumlen.begin(), upper));
        if (seg_idx == 0) {
            seg_idx = 1;
        }
        seg_idx -= 1;
        if (seg_idx >= cache.seg.size()) {
            seg_idx = cache.seg.size() - 1;
        }
        
        double len = cache.seg_len[seg_idx];
        if (len > EPS) {
            return XY{cache.seg[seg_idx].x / len, cache.seg[seg_idx].y / len};
        }
        return XY{1.0, 0.0};
    }
 
    /**
     * 장애물 슬롯(one/two)을 좌/우 경로로 각각 투영해 차로/전방성/거리 특성까지 계산한다.
     */

    ObjEval evalObjectOnPaths(const XY& obj_xy,
                              const PathCache& cache1,
                              const PathCache& cache2,
                              double ego_s1,
                              double ego_s2,
                              int current_lane,
                              bool warning_mode,
                              size_t slot_idx)
    {
        ObjEval eval;
        eval.input_valid = true;
        eval.raw = obj_xy;

        // 각 경로에 대한 정사영(거리 최소점)을 계산한다.
        eval.proj1 = projectPointToPath(obj_xy, cache1);
        eval.proj2 = projectPointToPath(obj_xy, cache2);

        // 거리 기반으로 1차 raw 차로를 정한다(어느쪽도 못 쓰면 조기 종료).
        int nearest_lane_id = 0;
        if (eval.proj1.valid && eval.proj2.valid) {
            nearest_lane_id = (eval.proj1.dist2 <= eval.proj2.dist2) ? 1 : 2;
            eval.raw_lane_id = nearest_lane_id;
        } else if (eval.proj1.valid) {
            nearest_lane_id = 1;
            eval.raw_lane_id = 1;
        } else if (eval.proj2.valid) {
            nearest_lane_id = 2;
            eval.raw_lane_id = 2;
        } else {
            return eval;
        }

        if (eval.proj1.valid && eval.proj2.valid) {
            // 두 경로 투영 간의 분리 벡터를 계산해 t축 기준(차선 바깥~내부~차선) band를 만든다.
            const double sep_x = eval.proj2.proj.x - eval.proj1.proj.x;
            const double sep_y = eval.proj2.proj.y - eval.proj1.proj.y;
            const double sep2 = sep_x * sep_x + sep_y * sep_y;

            if (sep2 > EPS) {
                // 병합이 아닌 구간: 두 경로 사이에서 ambiguity 영역을 만들고
                // 각 차선의 허용 band로 원시 판정(raw_lane_id)을 보정한다.
                const double sep = std::sqrt(sep2);
                const double inv_sep = 1.0 / sep;
                const double ux = sep_x * inv_sep;
                const double uy = sep_y * inv_sep;

                const double px = obj_xy.x - eval.proj1.proj.x;
                const double py = obj_xy.y - eval.proj1.proj.y;
                const double t = px * ux + py * uy;

                const double half_sep = 0.5 * sep;
                const double ambig_half = std::min(0.25, half_sep * 0.9);

                const double lane1_outer = -LANE_THRES;
                const double lane1_inner = half_sep - ambig_half;
                const double lane2_inner = half_sep + ambig_half;
                const double lane2_outer = sep + LANE_THRES;

                const bool in_lane1_band = (t >= lane1_outer) && (t <= lane1_inner);
                const bool in_lane2_band = (t >= lane2_inner) && (t <= lane2_outer);
                const bool in_ambig_band = (t > lane1_inner) && (t < lane2_inner);
                const bool within_outer_bound = (t >= lane1_outer) && (t <= lane2_outer);

                if (in_lane1_band && !in_lane2_band) {
                    eval.raw_lane_id = 1;
                } else if (in_lane2_band && !in_lane1_band) {
                    eval.raw_lane_id = 2;
                } else {
                    eval.raw_lane_id = nearest_lane_id;
                }

                eval.ambiguous_lane = in_ambig_band;
                eval.lane_thres_ok = within_outer_bound;
            } else {
                // 병합(Merged) 구간: 차로 분리가 불명확하므로 현재 차로를 우선 사용
                eval.ambiguous_lane = false; 
                const double min_d2 = std::min(eval.proj1.dist2, eval.proj2.dist2);
                eval.lane_thres_ok = min_d2 < LANE_THRES2;
                
                eval.raw_lane_id = current_lane;
            }
        } else {
            const double only_d2 = eval.proj1.valid ? eval.proj1.dist2 : eval.proj2.dist2;
            // 한쪽 경로만 유효하면 t기반 band 계산이 불가하므로 거리 임계만 유지한다.
            eval.ambiguous_lane = false;
            eval.lane_thres_ok = only_d2 < LANE_THRES2;
        }

        // ambiguous 정책 + warning 규칙을 반영해 최종 차로를 확정한다.
        eval.lane_id = resolveLaneForAmbiguous(slot_idx,
                                               eval.ambiguous_lane,
                                               eval.raw_lane_id,
                                               current_lane,
                                               warning_mode);

        // 최종 차로가 보정되었더라도 투영이 유효한 쪽을 기준으로 s/ego_s를 채운다.
        if (eval.lane_id == 1 && eval.proj1.valid) {
            eval.lane_proj = eval.proj1;
            eval.obs_s_lane = eval.proj1.s;
            eval.ego_s_lane = ego_s1;
        } else if (eval.lane_id == 2 && eval.proj2.valid) {
            eval.lane_proj = eval.proj2;
            eval.obs_s_lane = eval.proj2.s;
            eval.ego_s_lane = ego_s2;
        } else if (eval.raw_lane_id == 1 && eval.proj1.valid) {
            eval.lane_id = 1;
            eval.lane_proj = eval.proj1;
            eval.obs_s_lane = eval.proj1.s;
            eval.ego_s_lane = ego_s1;
        } else if (eval.raw_lane_id == 2 && eval.proj2.valid) {
            eval.lane_id = 2;
            eval.lane_proj = eval.proj2;
            eval.obs_s_lane = eval.proj2.s;
            eval.ego_s_lane = ego_s2;
        }

        if (slot_idx < lane_hold_.size() && (eval.lane_id == 1 || eval.lane_id == 2)) {
            lane_hold_[slot_idx] = eval.lane_id;
        }

        // 현재 차로 기준 전방 여부 판별용 값:
        // obs_s_lane - ego_s_lane > 0 이면 전방(front)
        eval.delta_s_lane = eval.obs_s_lane - eval.ego_s_lane;
        eval.is_front = eval.lane_thres_ok && (eval.delta_s_lane > 0.0);
        return eval;
    }

    /**
     * warning/go/overtake 상태별로 front/side 판단 규칙을 적용해 최종 결정을 만든다.
     */
    Decision decideFrontSide(const std::string& cur_state,
                             int current_lane,
                             const std::array<ObjEval, 2>& obj_evals,
                             const PathCache& cache_my,
                             double ego_s_my,
                             bool build_all_warning_shadows) const
    {
        Decision decision;
        decision.cal_distance = INF;

        const bool warning_mode = isWarningRegionState(cur_state);
        const bool curve_mode = isCurveRegionState(cur_state);
        const bool warning_like_mode = warning_mode || curve_mode;
        const bool go_like_mode = (cur_state == "go" ||
                                   cur_state == "slow");
        const bool overtake_mode = (cur_state == "overtake");

        // [변경] 대표 장애물 선택과 최종 제동 거리값은 모든 상태에서 raw.x를 사용한다.
        double nearest_raw_x = std::numeric_limits<double>::infinity();

        // [유지] 알 수 없는 기타 상태 대비용 fallback.
        double nearest_delta_s = std::numeric_limits<double>::infinity();

        // [변경] 이번 패치에서는 warning shadow를 s-copy로 만들지 않고,
        //        현재 차로 직접 투영점만 사용한다.
        (void)ego_s_my;
        (void)build_all_warning_shadows;

        for (size_t i = 0; i < obj_evals.size(); ++i) {
            const ObjEval& eval = obj_evals[i];

            if (!eval.input_valid) {
                continue;
            }

            // =========================================================
            // [변경] warning:
            // - 잡힌 장애물은 모두 전방 위험물처럼 취급
            // - lane_id/current_lane, eval.is_front로 후보를 제한하지 않음
            // - 후보 선택 기준과 최종 거리값은 raw.x
            // =========================================================
            if (warning_like_mode) {
                if (eval.raw.x <= 0.0) {
                    continue;
                }

                decision.front_candidate[i] = true;

                // [유지] side_obstacle 플래그는 기존 해석/호환을 위해 lane_id 기반으로 유지한다.
                if (eval.lane_thres_ok && eval.lane_id != current_lane) {
                    decision.side_obstacle = true;
                }

                // [curve warning 동일투영] curve 구간은 path1/path2가 겹치는 1차선으로 간주하므로
                // 차량 차선 소속보다 현재 추종 기준(path1) 위 위험물로 직접 투영하는 것이 중요하다.
                const ProjectionResult my_proj = projectPointToPath(eval.raw, cache_my);
                if (!my_proj.valid) {
                    continue;
                }

                decision.shadow_points[i] = my_proj.proj;
                decision.shadow_valid[i] = true;

                // [변경] warning의 대표 장애물 선택 기준은 raw.x로 통일한다.
                if (eval.raw.x < nearest_raw_x) {
                    nearest_raw_x = eval.raw.x;
                    decision.front_slot = static_cast<int>(i);
                }
                continue;
            }

            // =========================================================
            // [변경] go/slow/curve/overtake:
            // - 현재 차로 후보만 유지
            // - eval.is_front/delta_s_lane은 제어 후보 선정에 사용하지 않음
            // - 후보 비교와 최종 거리값은 raw.x로 통일
            // =========================================================
            if (go_like_mode || overtake_mode) {
                // [유지] 측면 장애물 플래그는 기존 호환을 위해 lane_id 기반으로 유지한다.
                if (eval.lane_thres_ok && eval.lane_id != current_lane) {
                    decision.side_obstacle = true;
                }

                // [변경] 현재 차로 + lane_thres_ok 인 후보만 제동 대상으로 사용한다.
                if (!(eval.lane_thres_ok && eval.lane_id == current_lane)) {
                    continue;
                }

                if (eval.raw.x <= 0.0) {
                    continue;
                }

                decision.front_candidate[i] = true;

                // [변경] go/slow/curve/overtake의 대표 장애물 선택 기준도 raw.x로 통일한다.
                if (eval.raw.x < nearest_raw_x) {
                    nearest_raw_x = eval.raw.x;
                    decision.front_slot = static_cast<int>(i);
                }
                continue;
            }

            // =========================================================
            // [유지] 알 수 없는 기타 상태는 기존 delta_s_lane 로직을 유지한다.
            // =========================================================
            if (!eval.is_front) {
                continue;
            }

            decision.front_candidate[i] = true;

            if (eval.lane_id != current_lane) {
                decision.side_obstacle = true;
            }

            if (eval.lane_id == current_lane && eval.delta_s_lane < nearest_delta_s) {
                nearest_delta_s = eval.delta_s_lane;
                decision.front_slot = static_cast<int>(i);
            }
        }

        if (decision.front_slot < 0) {
            return decision;
        }

        const ObjEval& front_eval = obj_evals[decision.front_slot];
        decision.front_obstacle = true;

        // =========================================================
        // [변경] warning 최종 결과:
        // - estop_plan에 들어가는 값은 raw.x
        // - 대응 경로점은 현재 차로 직접 투영점
        // =========================================================
        if (warning_like_mode) {
            decision.cal_distance = front_eval.raw.x;

            if (decision.shadow_valid[decision.front_slot]) {
                decision.path_x = decision.shadow_points[decision.front_slot].x;
                decision.path_y = decision.shadow_points[decision.front_slot].y;
            } else {
                decision.front_obstacle = false;
                decision.cal_distance = INF;
            }
            return decision;
        }

        // =========================================================
        // [변경] go/slow/curve/overtake 최종 결과:
        // - estop_plan에 들어가는 값은 raw.x
        // - 대응 경로점은 기존 현재 차로 투영점 유지
        // =========================================================
        if (go_like_mode || overtake_mode) {
            decision.cal_distance = front_eval.raw.x;

            // [유지] 현재 차로 투영점은 기존 방식 유지
            if (current_lane == 1 && front_eval.proj1.valid) {
                decision.path_x = front_eval.proj1.proj.x;
                decision.path_y = front_eval.proj1.proj.y;
            } else if (current_lane == 2 && front_eval.proj2.valid) {
                decision.path_x = front_eval.proj2.proj.x;
                decision.path_y = front_eval.proj2.proj.y;
            } else {
                decision.front_obstacle = false;
                decision.cal_distance = INF;
            }
            return decision;
        }

        // =========================================================
        // [유지] 알 수 없는 기타 상태 fallback
        // =========================================================
        decision.cal_distance = front_eval.delta_s_lane;

        if (current_lane == 1 && front_eval.proj1.valid) {
            decision.path_x = front_eval.proj1.proj.x;
            decision.path_y = front_eval.proj1.proj.y;
        } else if (current_lane == 2 && front_eval.proj2.valid) {
            decision.path_x = front_eval.proj2.proj.x;
            decision.path_y = front_eval.proj2.proj.y;
        } else {
            decision.front_obstacle = false;
            decision.cal_distance = INF;
        }

        return decision;
    }

    // ======== 디버그 마커 유틸 ========
    /**
     * 원형 마커(구형)을 생성/삭제해서 점 단위 장애물/좌표를 표시한다.
     */
    void pushSphereMarker(visualization_msgs::MarkerArray& marker_array,
                          int id,
                          const std::string& ns,
                          const ros::Time& stamp,
                          bool show,
                          const XY& p,
                          double scale,
                          float r,
                          float g,
                          float b,
                          float a,
                          const ros::Duration& lifetime) const
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = debug_marker_frame_id_;
        marker.header.stamp = stamp;
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = show ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;
        marker.lifetime = lifetime;

        if (show) {
            marker.pose.orientation.w = 1.0;
            marker.pose.position.x = p.x;
            marker.pose.position.y = p.y;
            marker.pose.position.z = 0.0;
            marker.scale.x = scale;
            marker.scale.y = scale;
            marker.scale.z = scale;
            marker.color.r = r;
            marker.color.g = g;
            marker.color.b = b;
            marker.color.a = a;
        }

        marker_array.markers.push_back(marker);
    }

    /**
     * 라인 스트립 마커 생성/삭제 (경로/경계선 표시).
     */
    void pushLineStripMarker(visualization_msgs::MarkerArray& marker_array,
                             int id,
                             const std::string& ns,
                             const ros::Time& stamp,
                             bool show,
                             const std::vector<XY>& pts,
                             double width,
                             float r,
                             float g,
                             float b,
                             float a,
                             const ros::Duration& lifetime) const
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = debug_marker_frame_id_;
        marker.header.stamp = stamp;
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = show ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;
        marker.lifetime = lifetime;

        if (show) {
            marker.scale.x = width;
            marker.color.r = r;
            marker.color.g = g;
            marker.color.b = b;
            marker.color.a = a;
            marker.points.reserve(pts.size());
            for (const auto& xy : pts) {
                geometry_msgs::Point p;
                p.x = xy.x;
                p.y = xy.y;
                p.z = 0.0;
                marker.points.push_back(p);
            }
        }

        marker_array.markers.push_back(marker);
    }

    /**
     * 라인 리스트 마커 생성/삭제 (점-점 연결선 표시).
     */
    void pushLineMarker(visualization_msgs::MarkerArray& marker_array,
                        int id,
                        const std::string& ns,
                        const ros::Time& stamp,
                        bool show,
                        const XY& from,
                        const XY& to,
                        double width,
                        float r,
                        float g,
                        float b,
                        float a,
                        const ros::Duration& lifetime) const
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = debug_marker_frame_id_;
        marker.header.stamp = stamp;
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = show ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;
        marker.lifetime = lifetime;

        if (show) {
            marker.scale.x = width;
            marker.color.r = r;
            marker.color.g = g;
            marker.color.b = b;
            marker.color.a = a;

            geometry_msgs::Point p0;
            p0.x = from.x;
            p0.y = from.y;
            p0.z = 0.0;
            geometry_msgs::Point p1;
            p1.x = to.x;
            p1.y = to.y;
            p1.z = 0.0;

            marker.points.push_back(p0);
            marker.points.push_back(p1);
        }

        marker_array.markers.push_back(marker);
    }

    /**
     * 두 경로의 상대 위치에서 split boundary를 구성한다.
     * sep<=EPS는 병합구간으로 간주한다.
     */
    SplitBoundaryCurves buildSplitBoundaryCurves(const PathCache& cache1, const PathCache& cache2) const
    {
        SplitBoundaryCurves curves;
        if (!cache1.valid || !cache2.valid || cache1.pts.size() < 2 || cache2.pts.size() < 2) {
            return curves;
        }

        const size_t base_samples = std::min(cache1.pts.size(), cache2.pts.size());
        const size_t sample_count = std::max<size_t>(24, std::min<size_t>(140, base_samples));
        curves.lane1_outer.reserve(sample_count);
        curves.lane1_inner.reserve(sample_count);
        curves.lane2_inner.reserve(sample_count);
        curves.lane2_outer.reserve(sample_count);

        for (size_t i = 0; i < sample_count; ++i) {
            const double alpha = (sample_count > 1) ?
                                 static_cast<double>(i) / static_cast<double>(sample_count - 1) :
                                 0.0;
            const XY p1 = samplePathAtS(cache1, alpha * cache1.total_len);
            const XY p2 = samplePathAtS(cache2, alpha * cache2.total_len);

            const double vx = p2.x - p1.x;
            const double vy = p2.y - p1.y;
            const double sep2 = vx * vx + vy * vy;

            if (sep2 <= EPS) {
                // 병합 구간: 진행 방향 법선 기반으로 좌/우 LANE_THRES를 양방향 경계로 생성
                XY dir = sampleDirAtS(cache1, alpha * cache1.total_len);
                double ux = dir.y;
                double uy = -dir.x;

                XY lane1_outer{p1.x - LANE_THRES * ux, p1.y - LANE_THRES * uy};
                XY lane1_inner{p1.x, p1.y}; // 내측선은 중앙(경로 위)으로 수렴시킴
                XY lane2_inner{p1.x, p1.y}; // 내측선은 중앙(경로 위)으로 수렴시킴
                XY lane2_outer{p1.x + LANE_THRES * ux, p1.y + LANE_THRES * uy};

                curves.lane1_outer.push_back(lane1_outer);
                curves.lane1_inner.push_back(lane1_inner);
                curves.lane2_inner.push_back(lane2_inner);
                curves.lane2_outer.push_back(lane2_outer);
                continue;
            }

            const double sep = std::sqrt(sep2);
            const double inv_sep = 1.0 / sep;
            const double ux = vx * inv_sep;
            const double uy = vy * inv_sep;

            const double half_sep = 0.5 * sep;
            const double ambig_half = std::min(0.25, half_sep * 0.9);

            const double lane1_inner_t = half_sep - ambig_half;
            const double lane2_inner_t = half_sep + ambig_half;

            XY lane1_outer{p1.x - LANE_THRES * ux, p1.y - LANE_THRES * uy};
            XY lane1_inner{p1.x + lane1_inner_t * ux, p1.y + lane1_inner_t * uy};
            XY lane2_inner{p1.x + lane2_inner_t * ux, p1.y + lane2_inner_t * uy};
            XY lane2_outer{p2.x + LANE_THRES * ux, p2.y + LANE_THRES * uy};

            curves.lane1_outer.push_back(lane1_outer);
            curves.lane1_inner.push_back(lane1_inner);
            curves.lane2_inner.push_back(lane2_inner);
            curves.lane2_outer.push_back(lane2_outer);
        }

        curves.valid = curves.lane1_outer.size() >= 2 &&
                       curves.lane1_inner.size() >= 2 &&
                       curves.lane2_inner.size() >= 2 &&
                       curves.lane2_outer.size() >= 2;
        return curves;
    }

    /**
     * split boundary 캐시 갱신.
     */
    void rebuildSplitBoundaryCache()
    {
        split_boundary_cache_ = buildSplitBoundaryCurves(path1_cache_, path2_cache_);
    }

    /**
     * 새 데이터 수신 후 투영/판단 재실행 필요 플래그.
     */
    void markProjectionDirty()
    {
        projection_dirty_ = true;
    }

    /**
     * 최근 계산한 장애물 평가/결정값을 저장.
     */
    void storeDebugSnapshot(const std::array<ObjEval, 2>& obj_evals,
                            const Decision& decision,
                            int current_lane)
    {
        last_obj_evals_ = obj_evals;
        last_decision_ = decision;
        last_debug_current_lane_ = current_lane;
        debug_snapshot_valid_ = true;
    }

    /**
     * dirty 상태면 update 계산, 아니면 스냅샷 기반 마커 재발행.
     */
    void processProjectionIfNeeded()
    {
        if (projection_dirty_) {
            updateObstacleProjectionDecision();
            projection_dirty_ = false;
            return;
        }

        if (debug_snapshot_valid_ && (enable_debug_markers_ || debug_marker_prev_enabled_)) {
            publishDebugMarkers(last_obj_evals_, last_decision_, last_debug_current_lane_);
        }
    }

    /**
     * 경로/장애물 상태를 RViz 마커로 시각화.
     * enabled가 false면 이전 마커 정리 후 비활성.
     */
    void publishDebugMarkers(const std::array<ObjEval, 2>& obj_evals,
                             const Decision& decision,
                             int current_lane)
    {
        if (!enable_debug_markers_) {
            if (debug_marker_prev_enabled_) {
                visualization_msgs::MarkerArray delete_array;
                visualization_msgs::Marker marker;
                marker.header.frame_id = debug_marker_frame_id_;
                marker.header.stamp = ros::Time::now();
                marker.action = visualization_msgs::Marker::DELETEALL;
                delete_array.markers.push_back(marker);
                debug_marker_pub.publish(delete_array);
            }
            debug_marker_prev_enabled_ = false;
            return;
        }

        debug_marker_prev_enabled_ = true;

        if (debug_marker_hz_ <= 0.0) {
            debug_marker_hz_ = 10.0;
        }

        const ros::Time now = ros::Time::now();
        if (!last_debug_marker_pub_time_.isZero()) {
            const double dt = (now - last_debug_marker_pub_time_).toSec();
            if (dt < (1.0 / debug_marker_hz_)) {
                return;
            }
        }
        last_debug_marker_pub_time_ = now;

        visualization_msgs::MarkerArray marker_array;
        const ros::Duration lifetime(2.0 / debug_marker_hz_);

        const SplitBoundaryCurves& split_curves = split_boundary_cache_;

        const bool p1_valid = path1_cache_.valid && path1_cache_.pts.size() >= 2;
        const bool p2_valid = path2_cache_.valid && path2_cache_.pts.size() >= 2;
        const bool split_valid = split_curves.valid;
        const float p1_gain = (current_lane == 1) ? 1.0f : 0.55f;
        const float p2_gain = (current_lane == 2) ? 1.0f : 0.55f;

        pushLineStripMarker(marker_array, 500, "lane_center", now, p1_valid, path1_cache_.pts, 0.08,
                            0.15f * p1_gain, 0.90f * p1_gain, 1.00f * p1_gain, 0.90f, lifetime);
        pushLineStripMarker(marker_array, 501, "lane_boundary", now, split_valid, split_curves.lane1_outer, 0.06,
                            0.00f, 0.70f * p1_gain, 1.00f * p1_gain, 0.85f, lifetime);
        pushLineStripMarker(marker_array, 502, "lane_boundary", now, split_valid, split_curves.lane1_inner, 0.05,
                            0.00f, 1.00f * p1_gain, 1.00f * p1_gain, 0.90f, lifetime);

        pushLineStripMarker(marker_array, 510, "lane_center", now, p2_valid, path2_cache_.pts, 0.08,
                            1.00f * p2_gain, 0.60f * p2_gain, 0.10f * p2_gain, 0.90f, lifetime);
        pushLineStripMarker(marker_array, 511, "lane_boundary", now, split_valid, split_curves.lane2_inner, 0.05,
                            1.00f, 0.85f * p2_gain, 0.20f, 0.90f, lifetime);
        pushLineStripMarker(marker_array, 512, "lane_boundary", now, split_valid, split_curves.lane2_outer, 0.06,
                            1.00f * p2_gain, 0.45f * p2_gain, 0.00f, 0.85f, lifetime);

        for (size_t i = 0; i < obj_evals.size(); ++i) {
            const bool show = obj_evals[i].input_valid;
            pushSphereMarker(marker_array,
                             static_cast<int>(100 + i),
                             "veh_raw",
                             now,
                             show,
                             obj_evals[i].raw,
                             0.35,
                             1.0f,
                             0.85f,
                             0.0f,
                             0.9f,
                             lifetime);

            const bool show_proj = obj_evals[i].input_valid && obj_evals[i].lane_proj.valid;
            const bool in_lane = obj_evals[i].lane_thres_ok;
            pushLineMarker(marker_array,
                           static_cast<int>(220 + i),
                           "veh_to_proj",
                           now,
                           show_proj,
                           obj_evals[i].raw,
                           obj_evals[i].lane_proj.proj,
                           0.05,
                           in_lane ? 0.0f : 1.0f,
                           in_lane ? 1.0f : 0.15f,
                           in_lane ? 0.15f : 0.15f,
                           0.95f,
                           lifetime);
        }

        for (size_t i = 0; i < obj_evals.size(); ++i) {
            const bool show = obj_evals[i].input_valid && obj_evals[i].lane_proj.valid;
            pushSphereMarker(marker_array,
                             static_cast<int>(200 + i),
                             "veh_lane_proj",
                             now,
                             show,
                             obj_evals[i].lane_proj.proj,
                             0.25,
                             0.2f,
                             0.4f,
                             1.0f,
                             0.9f,
                             lifetime);
        }

        const bool show_front_raw = decision.front_obstacle &&
                                    decision.front_slot >= 0 &&
                                    obj_evals[decision.front_slot].input_valid;
        XY front_raw;
        if (show_front_raw) {
            front_raw = obj_evals[decision.front_slot].raw;
        }
        pushSphereMarker(marker_array,
                         300,
                         "front_selected",
                         now,
                         show_front_raw,
                         front_raw,
                         0.50,
                         1.0f,
                         0.1f,
                         0.1f,
                         0.95f,
                         lifetime);

        const bool show_front_path = decision.front_obstacle &&
                                     std::isfinite(decision.path_x) &&
                                     std::isfinite(decision.path_y);
        XY front_path;
        if (show_front_path) {
            front_path.x = decision.path_x;
            front_path.y = decision.path_y;
        }
        pushSphereMarker(marker_array,
                         301,
                         "front_selected",
                         now,
                         show_front_path,
                         front_path,
                         0.45,
                         0.0f,
                         1.0f,
                         0.2f,
                         0.95f,
                         lifetime);

        const bool warning_like_mode = isWarningLikeRegionState(state);
        for (size_t i = 0; i < obj_evals.size(); ++i) {
            // [변경] warning에서는 잡힌 장애물을 모두 전방 후보처럼 취급하므로
            //        shadow 표시는 front_candidate 기준으로 바꾼다.
            const bool show_shadow = warning_like_mode &&
                                     decision.front_candidate[i] &&
                                     obj_evals[i].lane_id != current_lane &&
                                     decision.shadow_valid[i];

            pushSphereMarker(marker_array,
                             static_cast<int>(400 + i),
                             "warning_shadow",
                             now,
                             show_shadow,
                             decision.shadow_points[i],
                             0.35,
                             1.0f,
                             0.0f,
                             1.0f,
                             0.95f,
                             lifetime);

            pushLineMarker(marker_array,
                           static_cast<int>(410 + i),
                           "warning_shadow_line",
                           now,
                           show_shadow,
                           obj_evals[i].raw,
                           decision.shadow_points[i],
                           0.08,
                           0.1f,
                           1.0f,
                           1.0f,
                           0.95f,
                           lifetime);
        }

        debug_marker_pub.publish(marker_array);
    }

    /**
     * path1/path2 유효성, 경로 변경, 장애물 슬롯값을 반영해 최종 판단을 갱신한다.
     * - 경로 상태/slot 역할(front/side/none) 로그 생성
     */
    void updateObstacleProjectionDecision()
    {
        // 1) 상태 초기화: 매번 최신 계산으로 덮어쓴다.
        front_obstacle = false;
        side_obstacle = false;
        path_x = std::numeric_limits<double>::quiet_NaN();
        path_y = std::numeric_limits<double>::quiet_NaN();
        cal_distance = INF;
        rep_obstacle_log_ = RepresentativeObstacleLog{};
        side_obstacle_log_ = false;
        object_slot_logs_ = std::array<ObjectSlotLog, 2>{ObjectSlotLog{}, ObjectSlotLog{}};
        object_slot_logs_[0].exists = nearest_input_.one;
        object_slot_logs_[0].x = nearest_input_.one_x;
        object_slot_logs_[0].y = nearest_input_.one_y;
        object_slot_logs_[1].exists = nearest_input_.two;
        object_slot_logs_[1].x = nearest_input_.two_x;
        object_slot_logs_[1].y = nearest_input_.two_y;

        std::array<ObjEval, 2> obj_evals;

        const int current_lane = (getEffectiveCurrentPath() == 2) ? 2 : 1;
        const PathCache& my_cache = (current_lane == 1) ? path1_cache_ : path2_cache_;
        const bool warning_like_mode = isWarningLikeRegionState(state);

        // 필수 입력(양 경로 유효, ego 투영 유효, 장애물 메시지 수신)이 하나라도 없으면 계산 생략
        if (!path1_cache_.valid || !path2_cache_.valid || !my_cache.valid ||
            !path1_cache_.ego_valid || !path2_cache_.ego_valid || !nearest_input_.received) {
            Decision empty_decision;
            empty_decision.cal_distance = INF;
            storeDebugSnapshot(obj_evals, empty_decision, current_lane);
            if (enable_debug_markers_ || debug_marker_prev_enabled_) {
                publishDebugMarkers(obj_evals, empty_decision, current_lane);
            }
            return;
        }

        // one/two 소멸 시 히스테리시스 이력도 함께 초기화
        if (!nearest_input_.one) {
            lane_hold_[0] = 0;
        }
        if (!nearest_input_.two) {
            lane_hold_[1] = 0;
        }

        // 슬롯이 둘 다 없으면 front/side는 모두 false 처리
        if (!nearest_input_.one && !nearest_input_.two) {
            Decision empty_decision;
            empty_decision.cal_distance = INF;
            storeDebugSnapshot(obj_evals, empty_decision, current_lane);
            if (enable_debug_markers_ || debug_marker_prev_enabled_) {
                publishDebugMarkers(obj_evals, empty_decision, current_lane);
            }
            return;
        }

        // 슬롯별로 차로 투영/전방성/ambiguous 판정을 수행
        if (nearest_input_.one) {
            XY p{nearest_input_.one_x, nearest_input_.one_y};
            obj_evals[0] = evalObjectOnPaths(p,
                                             path1_cache_,
                                             path2_cache_,
                                             path1_cache_.ego_s,
                                             path2_cache_.ego_s,
                                             current_lane,
                                             warning_like_mode,
                                             0);
        }

        if (nearest_input_.two) {
            XY p{nearest_input_.two_x, nearest_input_.two_y};
            obj_evals[1] = evalObjectOnPaths(p,
                                             path1_cache_,
                                             path2_cache_,
                                             path1_cache_.ego_s,
                                             path2_cache_.ego_s,
                                             current_lane,
                                             warning_like_mode,
                                             1);
        }

        const double ego_s_my = (current_lane == 1) ? path1_cache_.ego_s : path2_cache_.ego_s;
        // 상태별 규칙으로 front/side 결정 + warning shadow 계산을 수행한다.
        const Decision decision = decideFrontSide(state, current_lane, obj_evals, my_cache, ego_s_my, enable_debug_markers_);

        // 먼저 전방 슬롯을 우선 사용하고, 없다면 front 조건을 만족하는 후보에서 보조로 대체한다.
        int rep_slot = -1;
        if (decision.front_slot >= 0 &&
            decision.front_slot < static_cast<int>(obj_evals.size()) &&
            obj_evals[decision.front_slot].input_valid &&
            obj_evals[decision.front_slot].lane_proj.valid) {
            rep_slot = decision.front_slot;
        }

        double best_thres_d2 = std::numeric_limits<double>::infinity();
        int best_thres_slot = -1;
        double best_any_d2 = std::numeric_limits<double>::infinity();
        int best_any_slot = -1;

        for (size_t i = 0; i < obj_evals.size(); ++i) {
            const ObjEval& eval = obj_evals[i];
            if (!eval.input_valid || !eval.lane_proj.valid) {
                continue;
            }

            // 각 슬롯을 로그 출력/디버그에서 보여주기 위해 "front/side/none"로 분류
            object_slot_logs_[i].valid = true;
            object_slot_logs_[i].lane_id = eval.lane_id;
            object_slot_logs_[i].lane_thres_ok = eval.lane_thres_ok;
            object_slot_logs_[i].is_front = eval.is_front;
            if (eval.lane_thres_ok && eval.lane_id == current_lane && eval.is_front) {
                object_slot_logs_[i].role = "front";
            } else if (eval.lane_thres_ok && eval.lane_id != current_lane) {
                object_slot_logs_[i].role = "side";
                side_obstacle_log_ = true;
            } else {
                object_slot_logs_[i].role = "none";
            }

            if (eval.lane_thres_ok && eval.lane_proj.dist2 < best_thres_d2) {
                best_thres_d2 = eval.lane_proj.dist2;
                best_thres_slot = static_cast<int>(i);
            }

            if (eval.lane_proj.dist2 < best_any_d2) {
                best_any_d2 = eval.lane_proj.dist2;
                best_any_slot = static_cast<int>(i);
            }
        }

        // representative 장애물은 로그 가독성을 위해
        // 1순위: front_slot, 2순위: 임계내 거리최소, 3순위: 전체 거리최소
        if (rep_slot < 0) {
            if (best_thres_slot >= 0) {
                rep_slot = best_thres_slot;
            } else if (best_any_slot >= 0) {
                rep_slot = best_any_slot;
            }
        }

        if (rep_slot >= 0) {
            const ObjEval& rep_eval = obj_evals[rep_slot];
            rep_obstacle_log_.valid = true;
            rep_obstacle_log_.slot = rep_slot;
            rep_obstacle_log_.lane_id = rep_eval.lane_id;
            rep_obstacle_log_.lane_thres_ok = rep_eval.lane_thres_ok;
            rep_obstacle_log_.is_front = rep_eval.is_front;
            rep_obstacle_log_.delta_s_lane = rep_eval.delta_s_lane;
            rep_obstacle_log_.raw_x = rep_eval.raw.x;
            rep_obstacle_log_.raw_y = rep_eval.raw.y;
            rep_obstacle_log_.proj_x = rep_eval.lane_proj.proj.x;
            rep_obstacle_log_.proj_y = rep_eval.lane_proj.proj.y;
            rep_obstacle_log_.lateral_dist = std::sqrt(std::max(0.0, rep_eval.lane_proj.dist2));
        }

        front_obstacle = decision.front_obstacle;
        side_obstacle = decision.side_obstacle;
        path_x = decision.path_x;
        path_y = decision.path_y;
        cal_distance = decision.cal_distance;

        // 마지막 계산값을 스냅샷에 저장해, projection이 dirty하지 않을 때에도 마커 갱신을 가능하게 한다.
        storeDebugSnapshot(obj_evals, decision, current_lane);
        if (enable_debug_markers_ || debug_marker_prev_enabled_) {
            publishDebugMarkers(obj_evals, decision, current_lane);
        }
    }

    /**
     * 생성자: ROS 구독/발행 및 노드 파라미터를 초기화한다.
     * - 구독: local_path1, local_path2, current_path, region, obstacle_info, nearest_vehicles2
     * - 발행: selected_path, decision
     * - 디버그 마커 관련 파라미터를 private namespace에서 읽음
     */
    PathSelect(ros::NodeHandle& nh)
    {
        // 입력 토픽
        local_path1_sub = nh.subscribe("/local_path1", 10, &PathSelect::LocalPath1CallBack, this);
        local_path2_sub = nh.subscribe("/local_path2", 10, &PathSelect::LocalPath2CallBack, this);
        current_path_sub = nh.subscribe("/current_path", 10, &PathSelect::CurrentPathCallBack, this);
        state_sub = nh.subscribe("/region", 10, &PathSelect::StateCallBack, this);

        // 장애물 토픽
        nearest_vehicles_sub = nh.subscribe("/obstacle_info", 10,
                                            &PathSelect::ObstacleInfoCallBack, this);
        nearest_vehicles2_sub = nh.subscribe("/nearest_vehicles2", 10,
                                             &PathSelect::NearestVehicles2CallBack, this);

        ros::NodeHandle pnh("~");
        pnh.param("camera_stop_topic", camera_stop_topic_, std::string("/camera_stop"));

        camera_stop_sub = nh.subscribe(camera_stop_topic_, 10,
                                       &PathSelect::CameraStopCallBack, this);

        // 출력 토픽
        selected_path_pub = nh.advertise<nav_msgs::Path>("/selected_path", 1);
        speed_control_pub = nh.advertise<std_msgs::String>("/decision", 1);

        // 디버그 마커 파라미터
        pnh.param("enable_debug_markers", enable_debug_markers_, true);
        pnh.param("debug_marker_topic", debug_marker_topic_, std::string("/local_path_selector/debug_markers"));
        pnh.param("debug_marker_hz", debug_marker_hz_, 10.0);
        pnh.param("force_ambiguous_to_current_lane", force_ambiguous_to_current_lane_, false);
        int curve_fixed_path_param = DEFAULT_CURVE_FIXED_PATH;
        pnh.param("curve_fixed_path", curve_fixed_path_param, DEFAULT_CURVE_FIXED_PATH);
        curve_fixed_path_ = normalizeLaneId(curve_fixed_path_param);
        if (curve_fixed_path_param != 1 && curve_fixed_path_param != 2) {
            ROS_WARN_STREAM("Invalid ~curve_fixed_path=" << curve_fixed_path_param
                            << ", fallback to " << curve_fixed_path_);
        }
        debug_marker_frame_id_ = "base_link";
        debug_marker_pub = nh.advertise<visualization_msgs::MarkerArray>(debug_marker_topic_, 1);

        frame_id = "base_link";
    }

    /**
     * /local_path1 콜백: 경로 캐시/병합 경계/투영 플래그 갱신
     */
    void LocalPath1CallBack(const nav_msgs::Path::ConstPtr &msg)
    {
        local_path1 = *msg;
        buildPathCache(local_path1, path1_cache_);
        rebuildSplitBoundaryCache();
        markProjectionDirty();
    }

    /**
     * /local_path2 콜백: 경로 캐시/병합 경계/투영 플래그 갱신
     */
    void LocalPath2CallBack(const nav_msgs::Path::ConstPtr &msg)
    {
        local_path2 = *msg;
        buildPathCache(local_path2, path2_cache_);
        rebuildSplitBoundaryCache();
        markProjectionDirty();
    }

    /**
     * /current_path 콜백: 현재 선택 경로 인덱스 갱신
     * 변경 시 dirty 처리하여 경로 기준 거리 보정 재실행
     */
    void CurrentPathCallBack(const std_msgs::Int32::ConstPtr &msg)
    {
        if (current_path != msg->data) {
            markProjectionDirty();
        }
        current_path = msg->data;
    }

    /**
     * /obstacle_info 콜백: one/two 슬롯 정보 수신 및 변경 시만 재평가 예약
     */
    void ObstacleInfoCallBack(const roborace_msgs::Objects::ConstPtr &msg)
    {
        // roborace_msgs/Object(one/two/one_x/one_y/two_x/two_y) 스키마 기준으로
        // 최근접 2대 입력(one/two 슬롯)으로 변환한다.
        bool one = false;
        bool two = false;
        double one_x = 0.0;
        double one_y = 0.0;
        double two_x = 0.0;
        double two_y = 0.0;

        if (!msg->objects.empty()) {
            const auto& packed = msg->objects[0];
            one = packed.one;
            two = packed.two;
            one_x = packed.one_x;
            one_y = packed.one_y;
            two_x = packed.two_x;
            two_y = packed.two_y;
        }

        // 변경량 체크(좌표 변화 포함)로 불필요한 재연산을 줄인다.
        const bool changed = (!nearest_input_.received) ||
                             (nearest_input_.one != one) ||
                             (nearest_input_.two != two) ||
                             (std::fabs(nearest_input_.one_x - one_x) > 1e-6) ||
                             (std::fabs(nearest_input_.one_y - one_y) > 1e-6) ||
                             (std::fabs(nearest_input_.two_x - two_x) > 1e-6) ||
                             (std::fabs(nearest_input_.two_y - two_y) > 1e-6);

        nearest_input_.received = true;
        nearest_input_.one = one;
        nearest_input_.two = two;
        nearest_input_.one_x = one_x;
        nearest_input_.one_y = one_y;
        nearest_input_.two_x = two_x;
        nearest_input_.two_y = two_y;

        if (changed) {
            markProjectionDirty();
        }
    }

    /**
     * /nearest_vehicles2 콜백: 대체 인지 포맷(one/two+xy)을 동일 내부 입력으로 변환
     */
    void NearestVehicles2CallBack(const roborace_msgs::NearestVehicles2::ConstPtr &msg)
    {
        const bool one = msg->one;
        const bool two = msg->two;
        const double one_x = msg->one_x;
        const double one_y = msg->one_y;
        const double two_x = msg->two_x;
        const double two_y = msg->two_y;

        const bool changed = (!nearest_input_.received) ||
                             (nearest_input_.one != one) ||
                             (nearest_input_.two != two) ||
                             (std::fabs(nearest_input_.one_x - one_x) > 1e-6) ||
                             (std::fabs(nearest_input_.one_y - one_y) > 1e-6) ||
                             (std::fabs(nearest_input_.two_x - two_x) > 1e-6) ||
                             (std::fabs(nearest_input_.two_y - two_y) > 1e-6);

        nearest_input_.received = true;
        nearest_input_.one = one;
        nearest_input_.two = two;
        nearest_input_.one_x = one_x;
        nearest_input_.one_y = one_y;
        nearest_input_.two_x = two_x;
        nearest_input_.two_y = two_y;

        if (changed) {
            markProjectionDirty();
        }
    }

    /**
     * /camera_stop 콜백: true면 최종 /decision을 stop으로 override한다.
     */
    void CameraStopCallBack(const std_msgs::Bool::ConstPtr &msg)
    {
        if (camera_stop_active_ == msg->data) {
            return;
        }

        camera_stop_active_ = msg->data;

        if (camera_stop_active_) {
            ROS_WARN_STREAM("camera_stop override enabled"
                            << " topic=" << camera_stop_topic_
                            << " data=true");
        } else {
            ROS_INFO_STREAM("camera_stop override cleared"
                            << " topic=" << camera_stop_topic_
                            << " data=false");
        }
    }

    /**
     * /region 콜백: 상태(go/warning/overtake) 갱신
     * 상태 변경도 판단 정책에 영향하므로 dirty 처리
     */
    void StateCallBack(const std_msgs::String::ConstPtr &msg)
    {
        if (state != msg->data) {
            markProjectionDirty();
        }
        state = msg->data;
    }

    /**
     * 선택된 경로를 헤더를 채워 퍼블리시
     */
    void publishSelectedPath(nav_msgs::Path& path)
    {
        path.header.frame_id = frame_id;
        path.header.stamp = ros::Time::now();
        selected_path_pub.publish(path);
    }


    /**
     * 경로의 일부를 1차 연속성을 유지하는 cubic 곡선으로 보간 후 반환
     * (차선 변경 시 전환 구간 부드럽게 생성)
     */
    std::vector<geometry_msgs::PoseStamped> cubicInterpolatePath(const nav_msgs::Path &path, int num_points)
    {
        std::vector<geometry_msgs::PoseStamped> interpolated_path;
        if (path.poses.empty())
        {
            ROS_WARN("Empty path for interpolation");
            return interpolated_path;
        }

        int end_idx = 0;
        for (size_t i = 0; i < path.poses.size(); ++i)
        {
            double dx = path.poses[i].pose.position.x - current_pose.x;
            if (dx >= 5.0)
            {
                end_idx = i;
                break;
            }
        }

        if (end_idx == 0)
        {
            ROS_WARN("Path is too short for interpolation from current position");
            return interpolated_path;
        }

        geometry_msgs::PoseStamped start_pose;
        start_pose.pose.position.x = 0;
        start_pose.pose.position.y = 0;
        geometry_msgs::PoseStamped end_pose = path.poses[end_idx];

        double x0 = start_pose.pose.position.x;
        double y0 = start_pose.pose.position.y;
        double x1 = end_pose.pose.position.x;
        double y1 = end_pose.pose.position.y;
        double dx = x1 - x0;
        double dy = y1 - y0;

        if (dx <= 0)
        {
            x1 = x0 - dx;
            y1 = y0 - dy;
            dx = x1 - x0;
            dy = y1 - y0;
        }

        double start_slope = 0.0;
        double end_slope = 0.0;

        if (end_idx + 1 < path.poses.size())
        {
            double next_x = path.poses[end_idx + 1].pose.position.x;
            double next_y = path.poses[end_idx + 1].pose.position.y;
            end_slope = (next_y - y1) / (next_x - x1);
        }

        double c0 = y0;
        double c1 = start_slope;
        double c2 = (3 * (y1 - y0) / (dx * dx)) - ((end_slope + 2 * start_slope) / dx);
        double c3 = (2 * (y0 - y1) / (dx * dx * dx)) + ((end_slope + start_slope) / (dx * dx));

        for (int i = 0; i < num_points; ++i)
        {
            double t = static_cast<double>(i) / (num_points - 1);
            double x = x0 + t * dx;
            double y = c0 + c1 * (x - x0) + c2 * pow((x - x0), 2) + c3 * pow((x - x0), 3);
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = frame_id;
            pose.header.stamp = ros::Time::now();
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            interpolated_path.push_back(pose);
        }

        for (size_t i = end_idx; i < path.poses.size(); ++i)
        {
            geometry_msgs::PoseStamped pose = path.poses[i];
            pose.header.frame_id = frame_id;
            interpolated_path.push_back(pose);
        }

        return interpolated_path;
    }

    /**
     * 전방 장애물 거리값 계산 (front_obstacle가 아니면 INF)
     */
    double calculation_ob_to_global(void)
    {
        if (!front_obstacle)
        {
            cal_distance = INF;
        }
        return cal_distance;
    }

    /**
     * 거리+상태 기반 저수준 결정 문자열 생성
     * - em: 정지
     * - start_change: 추월(차선변경) 시작 (overtake 구간에서만 허용)
     * - slowdown: 감속
     * - fast: 기본 주행
     */
    std::string estop_plan(void) 
    {
        // [curve 거리기반 유지] curve 구간도 고정 경로 추종만 강제할 뿐
        // 속도 명령은 기존 장애물 거리 기준(em/slowdown/fast)을 그대로 사용한다.

        // 전방 장애물 거리(미존재 시 INF) 기반 정책
        double distance = calculation_ob_to_global();

        std::string decision;

        if (state == "overtake") {
            // overtake: <8 stop, 8~14 change, 14~16 slowdown, >16 fast
            if (distance < 8.0) {
                decision = "em";
            } else if (distance < 14.0) {
                decision = "start_change";
            } else if (distance <= 16.0) {
                decision = "slowdown";
            } else {
                decision = "fast";
            }
        } else {
            // go/warning/slow/curve: <8 stop, <15 slowdown, else fast
            if (distance < 8.0) {
                decision = "em";
            } else if (distance < 15.0) {
                decision = "slowdown";
            } else {
                decision = "fast";
            }
        }

        return decision;
    }

    /**
     * 현재 current_path에 맞는 select_path와 selected_path_number를 갱신
     */
    void current_path_changer(void)
    {
        const int effective_path = getEffectiveCurrentPath();
        if (effective_path == 1) 
        {
            select_path = local_path1;
            selected_path_number = 1;
        }
        else 
        {
            select_path = local_path2;
            selected_path_number = 2;
        }
    }
};

constexpr int PathSelect::DEFAULT_CURVE_FIXED_PATH;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_path_selector");
    ros::NodeHandle nh;

    // ===== 실행 인스턴스 =====
    // 노드 전체 상태를 들고 있는 PathSelect 클래스 단일 객체를 생성한다.
    PathSelect path_selector(nh);
    // ===== 최종 제어 문자열/판단 상태 =====
    std_msgs::String speed_control_msg; 
    std::string estop_decision = "fast";

    // ===== 차선 변경 요청 진행 플래그 =====
    // overtake에서 start_change 조건 충족 시 set true
    bool check_lc = false;
    ros::Time lane_change_start_time;
    // 차선 변경 완료 직후 fast_lc를 잠시 유지하기 위한 후행 상태
    bool post_lane_change_fast_lc = false;
    ros::Time post_lane_change_start_time;

    // ===== 차선 변경 시작 전 차선 캐시 =====
    // 차선 변경 완료(current_path 변경) 감지용
    int save_current_path = path_selector.current_path;

    ros::NodeHandle pnh("~");
    double log_hz = 2.0;
    double max_lane_change_hold_sec = 10;
    double post_lane_change_fast_lc_sec = 1.0;
    pnh.param("log_hz", log_hz, 2.0);
    pnh.param("max_lane_change_hold_sec", max_lane_change_hold_sec, 15.0);
    pnh.param("post_lane_change_fast_lc_sec", post_lane_change_fast_lc_sec, 1.0);
    if (log_hz <= 0.0) {
        log_hz = 2.0;
    }
    ros::Time last_log_time;
    bool was_in_warning = false;
    bool was_in_curve = false;
    bool warning_path_hold_active = false;
    bool curve_path_transition_active = false;
    int warning_locked_path = path_selector.current_path;

    auto isWarningState = [&]() {
        return path_selector.isWarningRegionState(path_selector.state);
    };

    auto isCurveState = [&]() {
        return path_selector.isCurveRegionState(path_selector.state);
    };

    auto normalizedPath = [](int path) {
        return (path == 2) ? 2 : 1;
    };

    // 루프 주기(20Hz). 토픽 spin + 계산 + publish + 로그 출력 주기를 포함
    ros::Rate rate(20);
    speed_control_msg.data = "fast"; 

    auto shouldForceSlowByRegion = [&]() {
        // slow 구간은 최소 slow_down 강제
        // warning/curve는 별도 정책으로 처리한다.
        return path_selector.state == "slow";
    };

    // 측면(side) 장애물 전용 처리: side_obstacle가 true면 거리 상태값만 반영
    auto setSpeedForObstacle = [&](const std::string& decision) {
        if (decision == "em") {
            speed_control_msg.data = "stop";
        } else if (shouldForceSlowByRegion()) {
            speed_control_msg.data = "slow_down";
        } else if (decision == "slowdown") {
            speed_control_msg.data = "slow_down";
        } else if (decision == "start_change") {
            speed_control_msg.data = "slow_down";
        } else {
            speed_control_msg.data = "fast";
        }
    };

    // go/warning/slow/overtake 공통 기본 정책:
    // slow 지역에서는 em이 아닌 경우 slow_down을 강제한다.
    auto setSpeedByState = [&](const std::string& decision) {
        if (decision == "em") {
            speed_control_msg.data = "stop";
        } else if (shouldForceSlowByRegion()) {
            speed_control_msg.data = "slow_down";
        } else if (decision == "slowdown") {
            speed_control_msg.data = "slow_down";
        } else {
            speed_control_msg.data = "fast";
        }
    };

    auto isOppositeLaneEmptyFromPath = [&](int source_path) {
        const int normalized_source_path = normalizedPath(source_path);
        const int target_path = (normalized_source_path == 1) ? 2 : 1;
        return !path_selector.isLaneOccupiedInSnapshot(target_path);
    };

    auto selectDirectCurvePath = [&]() {
        if (path_selector.getCurveFixedPath() == 2) {
            path_selector.select_path = path_selector.local_path2;
        } else {
            path_selector.select_path = path_selector.local_path1;
        }
        path_selector.selected_path_number = path_selector.getCurveFixedPath();
    };

    auto isCurvePathSettled = [&]() {
        constexpr double kCurveSettleMaxAbsY = 0.3;
        const nav_msgs::Path& target_curve_path =
            (path_selector.getCurveFixedPath() == 2) ? path_selector.local_path2 : path_selector.local_path1;

        if (target_curve_path.poses.empty()) {
            return true;
        }

        double best_dist2 = std::numeric_limits<double>::infinity();
        double nearest_y = target_curve_path.poses.front().pose.position.y;

        // [curve settle 최근접점 기준] curve 고정 경로 안착 여부는 전방 임의 지점이 아니라
        // 현재 차량 위치에서 가장 가까운 목표 경로점이 중심선에 얼마나 붙었는지로 판단한다.
        for (const auto& pose : target_curve_path.poses) {
            const double dx = pose.pose.position.x - path_selector.current_pose.x;
            const double dy = pose.pose.position.y - path_selector.current_pose.y;
            const double dist2 = dx * dx + dy * dy;

            if (dist2 < best_dist2) {
                best_dist2 = dist2;
                nearest_y = pose.pose.position.y;
            }
        }

        return std::fabs(nearest_y) <= kCurveSettleMaxAbsY;
    };

    auto updateCurvePathTransition = [&]() {
        // [curve 고정 경로 복귀보간] curve 진입 전 비고정 경로를 추종 중이었다면
        // 차량 자세와 로컬 경로가 계속 변해도 curve_fixed_path 로 부드럽게 붙도록 cubic 을 매 루프 다시 계산한다.
        const nav_msgs::Path& target_curve_path =
            (path_selector.getCurveFixedPath() == 2) ? path_selector.local_path2 : path_selector.local_path1;
        std::vector<geometry_msgs::PoseStamped> interpolated =
            path_selector.cubicInterpolatePath(target_curve_path, 10);

        if (interpolated.empty()) {
            curve_path_transition_active = false;
            selectDirectCurvePath();
            return;
        }

        path_selector.select_path = target_curve_path;
        path_selector.select_path.poses = interpolated;
        path_selector.selected_path_number = path_selector.getCurveFixedPath();

        // [curve 고정 경로 직출력 복귀] 목표 경로 전방 기준점이 차량 중심에 충분히 가까워지면
        // 더 이상 완충 보간이 필요 없으므로 raw target curve path 로 복귀한다.
        if (isCurvePathSettled()) {
            curve_path_transition_active = false;
        }
    };

    // overtake에서 start_change 신호가 나오면 수행되는 차선변경 요청
    auto requestLaneChangeFromPath = [&](int source_path) {
        const int normalized_source_path = normalizedPath(source_path);
        speed_control_msg.data = "change";
        // 차선 변경 시작 전 기준 차선을 저장한다.
        save_current_path = normalized_source_path;
        check_lc = true;
        lane_change_start_time = ros::Time::now();
        if (warning_path_hold_active) {
            path_selector.setWarningPathLock(false, warning_locked_path);
            warning_path_hold_active = false;
        }

        // 차선 변경 시작 시점에도 즉시 보간 경로를 한 번 생성한다.
        if (normalized_source_path == 1) {
            path_selector.select_path.poses = path_selector.cubicInterpolatePath(path_selector.local_path2, 10);
            path_selector.selected_path_number = 2;
        } else {
            path_selector.select_path.poses = path_selector.cubicInterpolatePath(path_selector.local_path1, 10);
            path_selector.selected_path_number = 1;
        }
    };

    while (ros::ok())
    {
        ros::spinOnce();

        const bool in_warning = isWarningState();
        const bool in_curve = isCurveState();
        if (in_warning && !was_in_warning && !check_lc) {
            warning_locked_path = normalizedPath(path_selector.current_path);
            path_selector.setWarningPathLock(true, warning_locked_path);
            warning_path_hold_active = true;
        } else if (!in_warning && was_in_warning) {
            if (warning_path_hold_active) {
                path_selector.setWarningPathLock(false, warning_locked_path);
                warning_path_hold_active = false;
            }
        }

        if (in_curve && !was_in_curve) {
            // [curve lane change 무효화] curve 는 path1/path2 가 겹친 단일 차선으로 간주하므로
            // 기존 lane-change 상태보다 고정 경로 복귀를 우선하고 curve 전용 복귀 보간으로 흡수한다.
            curve_path_transition_active =
                (path_selector.selected_path_number != 0 &&
                 path_selector.selected_path_number != path_selector.getCurveFixedPath()) ||
                check_lc ||
                (path_selector.selected_path_number == 0 &&
                 path_selector.current_path != path_selector.getCurveFixedPath());
            check_lc = false;
            lane_change_start_time = ros::Time();
            post_lane_change_fast_lc = false;
            post_lane_change_start_time = ros::Time();
        } else if (!in_curve && was_in_curve) {
            // [curve override 해제] curve 가 끝나면 고정 경로 강제와 복귀 보간 상태를 모두 정리하고
            // 다음 루프부터 /current_path 기반 일반 경로 선택으로 되돌린다.
            curve_path_transition_active = false;
        }

        // [1] 최신 토픽 기반 투영 결과 반영
        path_selector.processProjectionIfNeeded();

        was_in_warning = in_warning;
        was_in_curve = in_curve;

        // [1-1] 차선 변경 요청이 오래 유지되면 안전하게 해제 (고정 방지)
        // 또는 차선 변경 시작 전 차선과 현재 차선이 달라지면 차선 변경 완료로 본다.
        if (check_lc && !lane_change_start_time.isZero()) {
            const double lc_age = (ros::Time::now() - lane_change_start_time).toSec();
            if (lc_age > max_lane_change_hold_sec) {
                check_lc = false;
            }
            else if (path_selector.current_path != save_current_path) {
                check_lc = false;
                // 차선 변경이 실제로 완료되면 잠시 fast_lc 상태를 유지한다.
                post_lane_change_fast_lc = true;
                post_lane_change_start_time = ros::Time::now();
            }
        }

        // [1-2] 차선 변경 직후 fast_lc 유지 시간 관리
        if (post_lane_change_fast_lc && !post_lane_change_start_time.isZero()) {
            const double post_lc_age = (ros::Time::now() - post_lane_change_start_time).toSec();
            if (post_lc_age > post_lane_change_fast_lc_sec) {
                post_lane_change_fast_lc = false;
            }
        }

        // [2] 매 루프 거리판단 갱신 (change 고정 방지)
        estop_decision = path_selector.estop_plan();

        // 차선 변경 진행 중에는 거리 기반 stop(em) 판정을 건너뛴다.
        // 차선 변경 중에 전방 거리 때문에 멈추지 않도록 기존 동작을 유지한다.
        if (check_lc && estop_decision == "em") {
            estop_decision = "fast";
        }

        // 차선 변경 진행 중에는 최신 로컬 경로 기준으로 보간 경로를 계속 갱신한다.
        // 그래야 헤딩/현재 위치 변화가 반영된 경로를 따라간다.
        if (check_lc) {
            if (save_current_path == 1) {
                path_selector.select_path.poses = path_selector.cubicInterpolatePath(path_selector.local_path2, 3.5);
                path_selector.selected_path_number = 2;
            } else if (save_current_path == 2) {
                path_selector.select_path.poses = path_selector.cubicInterpolatePath(path_selector.local_path1, 3.5);
                path_selector.selected_path_number = 1;
            }
        }

        // [3] 정책 우선순위: curve > 측면(side) > go/warning > overtake
        if (in_curve)
        {
            setSpeedByState(estop_decision);

            if (curve_path_transition_active) {
                updateCurvePathTransition();
            } else {
                // [curve 고정 경로] curve 구간에서는 제어기가 최종적으로 curve_fixed_path 만 추종해야 하므로
                // 전환 보간이 끝난 뒤에는 해당 로컬 경로를 그대로 publish 한다.
                selectDirectCurvePath();
            }
        }
        else if(path_selector.side_obstacle == true)
        {
            setSpeedForObstacle(estop_decision);

            if (!check_lc) {
                path_selector.current_path_changer();
            }
        }
        else
        {
            const bool has_go_like_state = (path_selector.state == "go" ||
                                            path_selector.state == "warning" ||
                                            path_selector.state == "Warning" ||
                                            path_selector.state == "slow");
            if(has_go_like_state)
            {
                setSpeedByState(estop_decision);

                if (!check_lc) {
                    path_selector.current_path_changer();
                }
            }
            else if(path_selector.state == "overtake")
            {
                if(estop_decision != "start_change")
                {
                    setSpeedByState(estop_decision);

                    if (!check_lc) {
                        path_selector.current_path_changer();
                    }
                }
                else if(estop_decision == "start_change") 
                {
                    const int reference_path = path_selector.getEffectiveCurrentPath();
                    if (isOppositeLaneEmptyFromPath(reference_path)) {
                        requestLaneChangeFromPath(reference_path);
                    } else {
                        setSpeedByState("slowdown");
                        if (!check_lc) {
                            path_selector.current_path_changer();
                        }
                    }
                }
            }
            else
            {
                // 알 수 없는 상태 문자열은 보수적으로 일반 감속 정책 사용
                setSpeedByState(estop_decision);
                if (!check_lc) {
                    path_selector.current_path_changer();
                }
            }
        }

        // [3-1] 최종 속도 우선순위 보정
        // stop/em > change > fast_lc > 기존 기본 속도
        if (estop_decision == "em") {
            speed_control_msg.data = "stop";
        } else if (check_lc) {
            speed_control_msg.data = "change";
        } else if (post_lane_change_fast_lc) {
            speed_control_msg.data = "fast_lc";
        }

        // [3-2] 외부 카메라 이상 신호가 들어오면 최종 속도 명령을 stop으로 강제한다.
        if (path_selector.camera_stop_active_) {
            speed_control_msg.data = "stop";
        }

        // [4] 결과 퍼블리시
        path_selector.speed_control_pub.publish(speed_control_msg);
        path_selector.select_path.header.frame_id = path_selector.frame_id;
        path_selector.select_path.header.stamp = ros::Time::now();
        path_selector.publishSelectedPath(path_selector.select_path);

        // [5] 텍스트 로그(설정 주기 log_hz)
        const ros::Time now = ros::Time::now();
        if (last_log_time.isZero() || (now - last_log_time).toSec() >= (1.0 / log_hz)) {
            last_log_time = now;

            std::cout << std::boolalpha << std::fixed << std::setprecision(2);
            std::cout << "Current Path : " << path_selector.current_path << " | Selected Path : " << path_selector.selected_path_number << std::endl;
            std::cout << "/decision : " << speed_control_msg.data << " | /region : " << path_selector.state
                      << " | check_lc : " << std::boolalpha << check_lc
                      << " | camera_stop_active : " << path_selector.camera_stop_active_
                      << " | curve_path_transition_active : " << curve_path_transition_active
                      << " | post_lane_change_fast_lc : " << post_lane_change_fast_lc
                      << " | warning_path_hold_active : " << warning_path_hold_active
                      << " | warning_locked_path : "
                      << (warning_path_hold_active ? std::to_string(warning_locked_path) : std::string("-"))
                      << std::endl;

            const bool obstacle_exists = path_selector.object_slot_logs_[0].exists || path_selector.object_slot_logs_[1].exists;
            std::cout << "obstacle_exists : " << obstacle_exists
                      << " | front_obstacle : " << path_selector.front_obstacle << std::endl;

            const auto& one_log = path_selector.object_slot_logs_[0];
            const auto& two_log = path_selector.object_slot_logs_[1];
            std::cout << "one : exists=" << one_log.exists
                      << " | role=" << (one_log.exists ? one_log.role : "none")
                      << " | pos=(" << one_log.x << ", " << one_log.y << ")"
                      << std::endl;
            std::cout << "two : exists=" << two_log.exists
                      << " | role=" << (two_log.exists ? two_log.role : "none")
                      << " | pos=(" << two_log.x << ", " << two_log.y << ")"
                      << std::endl;

            if (path_selector.rep_obstacle_log_.valid) {
                const char* slot_name = (path_selector.rep_obstacle_log_.slot == 0) ? "one" :
                                        (path_selector.rep_obstacle_log_.slot == 1) ? "two" : "none";
                std::cout << "representative_obstacle : " << slot_name
                          << " @ (" << path_selector.rep_obstacle_log_.raw_x
                          << ", " << path_selector.rep_obstacle_log_.raw_y << ")"
                          << std::endl;
            } else {
                std::cout << "representative_obstacle : none" << std::endl;
            }

            std::cout << "estop_decision : " << estop_decision << std::endl;
            std::cout << "=========================================== " << std::endl;
        }

        rate.sleep();
    }

    return 0;
}
