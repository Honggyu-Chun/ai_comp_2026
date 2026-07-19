# 판단부(Decision) 아키텍처 설계서

- 프로젝트: ai_comp_2026 (2026 국토부 KATRI 대학생 AI/SW 모빌리티 경진대회)
- 대상: MORAI SIM K-City (`R-KR_PG_K-City_2025`, UTM52N)
- 대회용 시뮬레이터: 25.S4.MolitComp03 / 차량: 2023 Hyundai Ioniq5 (폭 1.892m, 길이 4.635m)
- 작성 상태: **v0.2** — 규정집 v1.0 정렬, 인지/제어 경계·2026 우회전 도로교통법 확인 반영
  (v0.1: 인터페이스 미확정 초안. 본 문서가 대체)

---

## 0. v0.2에서 확정된 것

규정집 v1.0 + 조사로 아래가 확정되어 v0.1 초안을 실제 미션에 정렬했다.

- **속도 판정은 Ego Vehicle Status 기준, 60kph 초과 즉시 15초** → 60 상한 하드클램프가 최우선.
- **랜덤 미션은 "장애물 회피 / 끼어들기" 2종뿐** (규정 3-2) → 터널 처리 단순화.
- **2026 우회전 도로교통법** 확정 (아래 §1).
- **제어 노드 부재** 확인 → 판단부는 `/selected_path`+`/decision`까지만, CtrlCmd는 제어팀.
- **자차 pose 입력은 `/gps_utm_odom`** (ll2utm.py 출력).

사용자 확정 결정:
1. 정적 장애물 → **Path Shifting** (Frenet Lattice 미채택)
2. 동적 장애물 → **TTC 정지/감속 전용** (횡회피 안 함)
3. 입력 소스 미정 → **어댑터로 추상화**해서 흡수
4. 출력 경계 → **경로 + 목표속도 + 정지요구** (판단부는 CtrlCmd 안 만든다)
5. **ADAS ACC(앞차 추종) 상시 동작** — 선행차 속도에 맞춰 following하는 종방향 기능이 전 구간 항상 켜짐 (FollowingModule)

> 첨부 예정이던 미션 지도 이미지·예제 시나리오 파일은 아직 미수령. 본 설계는 규정집 전문 + 구두 경로 설명 기반이며, 파일 수령 시 RegionResolver의 구역 좌표/시퀀스만 갱신하면 된다.

---

## 1. 설계 원칙

### 1.1 모든 미션을 3가지 출력으로 환원한다

미션이 여러 종이라고 노드를 종마다 만들지 않는다. 판단부의 결론은 결국 셋 중 하나다.

| 출력 | 의미 |
|---|---|
| 경로 | 어느 쪽으로 갈 것인가 (횡방향) |
| 목표 속도 | 얼마나 빠르게 갈 것인가 (종방향) |
| 정지 요구 | 지금 멈춰야 하는가 (거부권) |

### 1.2 중재는 "가장 보수적인 값"으로 한다

우선순위 표를 만들지 않는다.
- 속도: 모든 모듈 상한 중 **최솟값**
- 정지: **하나라도** 요구하면 정지
- 경로: 우선순위 최상위 하나만 채택

법규 준수와 안전 양쪽에 자동 부합하고, 우선순위 표 관리라는 버그 온상을 없앤다. 코드도 짧다.

### 1.3 외부 메시지 타입은 어댑터에서만 다룬다

인지·제어 인터페이스가 확정되지 않았고 반드시 바뀐다. 모듈 코드는 **내부 공통 타입만** 안다.

### 1.4 규정집 확정 제약

| 항목 | 규정 | 설계 함의 |
|---|---|---|
| 속도 | 전구간 60kph, **고주로 진입~톨게이트만 해제**. 초과 **즉시** 15초, 3초 지속마다 +15초. **Ego Vehicle Status로 판정** | 60 상한 **하드클램프 최우선**. 오버슈트 마진 위해 목표 58kph 등 튜닝 |
| 교차로 신호 | **앞바퀴가 정지선 넘는 순간의 신호**로 판정. 초록에 주행. 미준수 15초 | 정지 위치 = 정지선 정밀도 중요 → 정지선 거리 소스 필요 |
| 차로 준수 | 차선 접촉 3초당 5초 (바퀴 1개 기준) | Path Shifting `max_shift`는 **차선폭 이내**. 넘으면 정지 |
| 장애물 회피 | 크기 고려 충돌 시 15초/회 | 충돌 마진에 객체 bounding box 반영 |
| 끼어들기 | 회전교차로/합류에서 NPC 충돌 유의 | 회전교차로·고주로 합류 = 동일 gap-acceptance 문제 |
| GPS 음영(터널) | GPS **blackout**. 랜덤미션 **장애물/끼어들기 2종뿐** | 터널에선 신호등·우회전 안 나옴 → Static+Dynamic+Gap만 켜면 충분 |
| 제한시간 | 15분 | 교착(회전교차로 무한대기) 방지 필수 |

**2026 우회전 도로교통법:**
- 전방신호 **적색** → 정지선 **완전정지**(바퀴 정지), 이후 횡단보도 보행자 없으면 진행. 서행 슬금 통과는 위반.
- 전방신호 **녹색** → 우회전 직후 횡단보도 보행자 건너거나 건너려 하면 정지, 없으면 일시정지 없이 서행 통과 가능.
- **우회전 전용 신호등**이 있으면 **녹색 화살표에만** 진행.

---

## 2. I/O 경계

**입력** (판단부가 구독):
- `/gps_utm_odom` — `nav_msgs/Odometry` (**자차 pose 주 입력**). 체인: `/gps`(`morai_msgs/GPSMessage`) → `ll2utm.py`(UTM52N) → `/gps_utm_odom`. (융합 출력 `/odometry/filtered`도 있으나 판단부는 `/gps_utm_odom` 사용)
- `/morai/ego_topic` — `morai_msgs/EgoVehicleStatus` (속도/heading 보조, 규정 속도판정과 동일 소스)
- 전역경로 `nav_msgs/Path` (차선별 1~N개)
- 장애물 토픽 — **미정** (`katri_msgs/Objects` 또는 MORAI `ObjectStatusList`)
- 신호등 토픽 — **미정** (인지팀, 판단부 수신 가정)

**출력** (판단부가 발행):
- `/selected_path` — `nav_msgs/Path`
- `/decision` — `katri_msgs/Decision` **(신규 정의 필요)**

**경계 주의:** `/selected_path`+`/decision` → `/ctrl_cmd`(`morai_msgs/CtrlCmd`, `longlCmdType=1` accel/brake) 변환하는 **순수추종+종방향 제어 노드가 현재 워크스페이스에 없다** → **제어팀 스코프**. 판단부는 CtrlCmd를 생성하지 않는다.
(참고: `control_udp_pub.py`가 `/ctrl_cmd`를 UDP 중계. 그 브리지가 `msg.steering`을 읽는데 repo `CtrlCmd.msg`엔 `front_steer`만 있어 필드명 불일치 — 제어팀 확인사항, 판단부 무관.)

---

## 3. 전체 구조

```
[외부입력] ─► [어댑터] ─► 내부공통타입
   /gps_utm_odom ► EgoAdapter  ──► EgoState (pose_valid 포함)
   장애물(미정)─► ObjectAdapter ──► vector<Obstacle>
   신호등(미정)─► TrafficAdapter──► TrafficLight
   전역경로 ────► PathAdapter   ──► ReferencePath
                                        │
        ┌───────────────────────────────┘
        ▼
   1) RegionResolver   위치 → MissionRegion (터널이면 gps_shadow=true)
        ▼
   2) BehaviorModules  구역 활성 모듈만 병렬 evaluate() → BehaviorOutput[]
        ▼
   3) Arbiter          속도=min, 정지=OR, 경로=우선순위 최상위
        ▼
   4) PathGenerator    기준경로 + 횡오프셋/차선선택 → 최종경로
        ▼
   /selected_path (Path) + /decision (목표속도·정지·reason)
```

min/OR 중재는 우선순위표 없이 법규+안전에 자동 부합, GPS 음영에서 "전 모듈 ON"이 랜덤미션을 별도 로직 없이 흡수.

---

## 4. 내부 공통 타입 (SI 단위, 어댑터에서만 외부 메시지 취급)

```cpp
struct EgoState {
  double x_m, y_m;       // ENU 위치
  double yaw_rad;        // ENU 기준, 반시계 +
  double speed_mps;
  bool   pose_valid;     // GPS 음영구간에서 false
};

struct Obstacle {
  int    id;
  double x_m, y_m;
  double yaw_rad;
  double speed_mps;
  double width_m, length_m;
  bool   is_pedestrian;
};

struct TrafficLight {
  bool   valid;
  bool   is_red, is_green, is_yellow;
  bool   go_left;        // 좌회전 화살표
  bool   arrow_right;    // 우회전 전용 녹색 화살표
  double stopline_dist_m;  // 모르면 음수
};

struct ReferencePath {
  std::vector<double> x_m, y_m, s_m;
  int    nearest_index;
  double lateral_error_m;
};

struct MissionRegion {
  enum Kind { NORMAL, TRAFFIC_LIGHT, STATIC_OBS, DYNAMIC_OBS,
              INTERSECTION, ROUNDABOUT, HIGHWAY, TOLLGATE } kind;
  bool gps_shadow;   // 터널
  int  subphase;     // 고주로 차선변경 시퀀스(좌1/좌2/우1/우2) 등
};

struct BehaviorOutput {
  double speed_limit_mps = 1e9;   // 제한 없으면 큰 값
  bool   request_stop    = false; // 하나라도 true면 최종 정지
  double lateral_offset_m = 0.0;  // 왼쪽 +, 오른쪽 -
  int    target_path_index = -1;  // -1이면 관심 없음
  std::string reason;             // 로그·디버깅
};

class BehaviorModule {
 public:
  virtual ~BehaviorModule() = default;
  virtual std::string name() const = 0;
  virtual bool isActive(const MissionRegion& region) const = 0;
  virtual BehaviorOutput evaluate(const EgoState& ego,
                                  const std::vector<Obstacle>& obstacles,
                                  const TrafficLight& light,
                                  const ReferencePath& path) = 0;
};
```

**규칙: `evaluate()` 안에서 `ros::` 금지** → 시뮬 없이 단위 테스트 가능(핵심).
정적/동적 장애물 타입을 나누지 않는다 — 속도로 모듈이 알아서 판단(경계 문제·오분류 버그 회피).

---

## 5. 모듈 명세

| # | 모듈 | 활성 구역 | 출력 | 방식(확정) |
|---|---|---|---|---|
| 1 | **SpeedLimitModule** | 항상 | `speed_limit_mps` | 60kph 하드상한 + 곡률기반 감속. 고주로 구간만 상한 해제. **최우선 안전선** |
| 1b | **FollowingModule (ADAS ACC)** | **항상** | `speed_limit_mps` | **앞차 추종(상시).** 자차 차로 전방 선행차와 **정속 time-gap(CTG)** 유지: gap≥목표헤드웨이면 `v_limit=min(v_lead, v_desired)`, gap<목표면 비례 감속. 선행차 없으면 무제한. DynamicObstacle(비상정지)와는 min으로 자동 합성 — 역할 분리 |
| 2 | **TrafficLightModule** | 신호등 구역 | `request_stop`, `speed_limit_mps` | **red→정지(0), green→해제.** 제동거리(v²/2a) 미달 시 통과. 정지선거리 비례 감속 → 정지선 앞 0 |
| 3 | **StaticObstacleModule** | 정적장애물 구역 (+음영) | `lateral_offset_m`, `speed_limit_mps` | **Path Shifting.** 경로 s/d 투영 → 반대쪽 완만히 밀었다 복귀. `max_shift`=차선폭 이내, 못 피하면 정지 |
| 4 | **DynamicObstacleModule** | 동적장애물 구역 (+음영) | `request_stop`, `speed_limit_mps` | **TTC 정지/감속 전용.** 횡거리 게이팅(인도 위 보행자 무시) + 히스테리시스(경계 떨림 방지). 횡회피 안 함 |
| 5 | **IntersectionModule** | 교차로 구역 | `request_stop` | **우회전**: 적색→완전정지 후 보행자없음 진행 / 녹색→보행자 있으면 정지 / 전용신호→녹색화살표만. **좌회전**: 좌회전 화살표 대기(신호 비트만 다름) |
| 6 | **GapAcceptanceModule** | 회전교차로 + 고주로 합류/차선변경 | `request_stop`, `speed_limit_mps`, `target_path_index` | 진입대상 차로 차량 TTC → gap 있으면 진입. **교착방지**: 대기 길어지면 임계값 완화(최소안전값 하한). 고주로는 좌2우2 시퀀스를 subphase로 순차 |

**GPS 음영(터널) 처리:** 별도 모듈 없음. `gps_shadow=true`면 region gate를 우회해 **Static+Dynamic+Gap 상시 활성**(랜덤미션이 이 2종뿐). 신호등 모듈도 켜두되 유효신호 없으면 `valid=false`로 자동 무동작 → 코드 안 늘어남. 측위 blackout은 판단부 소관 아님(측위팀 추측항법). 판단부는 `pose_valid=false` 시 **속도만 낮춘다**.

**회전교차로 진출:** "진입 후 바로 우측 진출로"는 **전역경로가 이미 진출로로 그려져 있으면** 경로 문제가 아니다. GapAcceptance는 **진입 타이밍(정지/출발)만** 담당, 진출 경로는 PathGenerator의 전역경로 추종이 처리 → 별도 진출 로직 불필요.

---

## 6. 중재 규칙 (Arbiter)

```
target_speed = min(모든 모듈 speed_limit)     // 60 상한이 항상 포함되어 안전
stop_required = OR(모든 모듈 request_stop)      // 하나라도 정지면 정지
path = 우선순위 최상위 모듈의 offset/target_path만 채택 (더하지 않음)
```
경로 우선순위: DynamicObstacle > StaticObstacle > GapAcceptance(차선변경) > Intersection.
(속도·정지는 우선순위 불필요 — min/OR로 해결)

`/decision.reason`에 **어느 모듈이 왜** 이 값을 냈는지 한 줄 기록 → "왜 멈췄지?" 로그 추적(15분 제한 하 디버깅 필수).

---

## 7. 구현 순서 (규정 미션 순서 정렬 + 기존 코드 재사용)

**기존 3개 노드 중 2개는 코어 재사용, 1개만 교체** — 가장 적은 작업량:

| 기존 노드 | 새 역할 | 작업 |
|---|---|---|
| `region_state_publisher.cpp` | **RegionResolver** | 출력을 behavior 문자열 → `MissionRegion`으로 축소. 컬러맵 PNG 룩업·covariance 기반 pose 품질 로직 **재사용** |
| `odom_path_publisher.cpp` | **PathAdapter** | nearest_index·base_link 변환·로컬경로 추출 로직 **거의 그대로** ReferencePath 생산 |
| `local_path_selector.cpp` | **삭제** → BehaviorModules+Arbiter+PathGenerator | 뒤엉킨 FSM 폐기, 모듈 구조로 재작성 |

| 단계 | 내용 | 검증 |
|---|---|---|
| 0 | `katri_msgs/Decision.msg` 정의 + `CMakeLists.txt` `add_message_files` 등록 | 빌드/echo |
| 1 | 어댑터 + Arbiter + 빈 모듈 1개 (골격) | 경로 passthrough 확인 |
| 2 | SpeedLimitModule (60 클램프 + 곡률) | 60kph 상한·급커브 이탈 없음 |
| 2b | FollowingModule (ADAS ACC, 상시) | 앞차에 헤드웨이 유지·앞차 속도 추종, 앞차 없으면 자유주행 |
| 3 | TrafficLightModule | 정지선 앞 정차, red→0 green→주행 |
| 4 | StaticObstacleModule (Path Shifting) | 장애물 회피·차선 이내 |
| 5 | DynamicObstacleModule (TTC) | 보행자 정지, 인도 보행자 무시, 떨림 없음 |
| 6 | IntersectionModule | 우회전 완전정지, 좌회전 화살표 대기 |
| 7 | GapAcceptanceModule (회전교차로) | 충돌없이 진입·우측 진출 |
| 8 | GapAcceptanceModule (고주로 좌2우2) | 시퀀스대로 차선변경, 톨게이트 |
| 9 | GPS 음영 전모듈 활성 | 터널 랜덤(장애물/끼어들기) 대응 |

**1단계가 가장 중요**(골격 오류 시 전 단계 재작업).

---

## 8. 어댑터 추상화 (소스 미정 대응)

`ObjectAdapter`/`TrafficAdapter`를 **인터페이스로만** 두고 소스별 구현을 교체:
- 장애물: `katri_msgs/Objects`(차선번호+거리) 구현 / MORAI `ObjectStatusList`(ENU+size) 구현 — 둘 다 `vector<Obstacle>`로 변환.
- 신호등: `GetTrafficLightStatus.trafficLightStatus` 비트(1=R,4=Y,16=G,32=G-left)→`TrafficLight` 매핑.
- 정지선 거리: 토픽에 없으면 **판단부 지도 정지선 좌표 테이블**을 fallback(규정: 앞바퀴 정지선 판정 → 정밀 위치 필요).

모듈 코드는 내부 타입만 안다 → 인터페이스 확정 전에도 설계·테스트 진행 가능.

---

## 9. Calibration knobs (실물 튜닝 여지)

| 파라미터 | 초기값 | 튜닝 근거 |
|---|---|---|
| `target_speed_margin` | 58kph | Ego Status 판정 오버슈트 방어 |
| `full_stop_speed / hold_s` | 0.05 m/s / 1.5s | 우회전 완전정지 심판 기준(규정 미공개) |
| `ttc_stop_s / ttc_slow_s / lateral_clear_m / resume_hold_s` | 2.0/4.0/1.5/1.0 | 보행자 실측 |
| `acc_headway_s / acc_min_gap_m / v_desired` | 1.5s / 6.0m / 58kph | ADAS 앞차 추종 헤드웨이·최소간격 |
| `lateral_margin / max_shift` | 0.5 / 차선폭 | 차폭 1.892m + 객체 크기 |
| `gap_ttc_s / gap_ttc_min_s / relax_per_s` | 5.0/3.0/0.2 | 회전교차로 교착 vs 안전(15분 제한) |
| `highway_limit` | 미정 | 규정집 고주로 상한 확인 필요 |

---

## 10. 검증

- **단위 테스트**: 각 `evaluate()`는 순수 함수 → 시뮬 없이 입력 fixture로 assert. 최소 각 모듈 1개 self-check.
- **통합**: 예제 시나리오(수령 후) rosbag 재생 → `/selected_path`·`/decision` rviz 확인.
- **미션별 주행**: §7 단계별 검증 항목대로.

---

## 11. 열린 항목 (구현 전 확정 필요)

1. **장애물 인지 메시지 스펙** — `katri_msgs/Object`는 폭/길이/ENU/ID가 없어 Path Shifting의 s/d 투영·충돌마진 계산 불가. 인지팀 필드확장 or MORAI `ObjectStatusList` 임시채택 결정 필요. **1단계 진입 전 최우선.**
2. 신호등 정지선 거리 포함 여부 (인지 협의) — 없으면 지도 테이블.
3. 신호등 직진/좌회전 구분 실측 (MORAI).
4. 고주로 구간 속도 상한 규정 확인.
5. 우회전 완전정지 심판 기준(시간).
6. `/decision` 최종 필드 (제어팀 협의).
7. 미션 지도·예제 시나리오 파일 수령 → RegionResolver 좌표/시퀀스 확정.
