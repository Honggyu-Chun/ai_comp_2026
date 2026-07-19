# global_path_planner 작업 기록

## 2026-06-26 작업 요약

### 빌드 및 기존 코드 정리

- `catkin build` 실패 원인을 확인하고, 사용하지 않는 AI 예제 소스 경로를 `CMakeLists.txt`에서 실제 위치에 맞게 정리했다.
- `control/src/mpc_control.cpp`의 메시지 필드명을 현재 `katri_msgs` 정의에 맞게 수정해 전체 workspace 빌드가 가능하도록 했다.

### `ll2utm.py` 및 launch 설정

- `src/global_path_planner/scripts/ll2utm.py`의 ROS parameter들을 launch에서 조정할 수 있도록 `gpp_local_final.launch`에 arg/param을 추가했다.
- `rosrun` 단독 실행과 launch 실행의 UTM 결과 차이를 확인하고, launch 기본 offset을 `0.0 / 0.0`으로 맞췄다.
- `ll2utm`은 위도/경도를 수학적으로 UTM으로 변환한 뒤 offset을 적용하는 구조임을 확인했다.

### `global_path_publisher.py`

- 경로 파일명과 publish topic 이름을 ROS parameter로 받을 수 있도록 확인 및 보완했다.
- 파일은 변동되지 않는 전제에 맞춰 최초 1회 로드 후 메모리에 캐싱하도록 최적화했다.
- 시작 시 최초 1회 publish와 latching은 유지했다.
- 반복 publish는 하지 않고, 요청 시 republish할 때는 `Path.header.stamp`만 갱신하도록 정리했다.
- 관련 parameter를 `gpp_local_final.launch`에서 설정 가능하도록 반영했다.

### `path_maker_sm.py`

- 실행 시 저장 파일명과 로깅 상태를 terminal log로 출력하도록 추가했다.
- `closed_loop` parameter를 추가하고, 시작점 반경 안에 있는 가장 먼 마지막 index를 기준으로 폐곡선 smoothing이 가능하도록 했다.
- raw 경로와 `_sm` smoothing 경로 모두 기존 저장 위치를 유지하도록 했다.
- keyboard interrupt 종료 시 경로 plot을 띄우던 흐름은 유지하되, GUI shutdown hang을 피하기 위해 plot 이미지를 저장하는 방식으로 변경했다.
- plot 저장 위치는 `path_data/plot` 폴더로 설정했다.
- 경로 plot 선 굵기를 줄여 긴 경로에서도 형태가 잘 보이도록 수정했다.

### `path_maker.launch`

- `src/global_path_planner/launch/path_maker.launch`를 추가했다.
- launch에서 `ll2utm.py`와 `path_maker_sm.py`를 함께 실행하도록 구성했다.
- 파일명, `closed_loop`, `closed_loop_radius`를 launch arg로 설정 가능하게 했다.

### launch 주석 정리

- `gpp_local_final.launch`에 parameter 설정부와 node 실행부를 한국어 한 문장 주석으로 분리했다.
- `map_server`가 설치되어 있지 않은 환경에서 launch error가 발생할 수 있음을 확인했다.

## `odom_path_publisher.cpp` 최적화 작업

### 1차 기능 고도화 완료

- `src/global_path_planner/src/odom_path_publisher_robo.cpp`는 참고용으로만 확인하고 수정하지 않았다.
- `src/global_path_planner/src/odom_path_publisher.cpp`에 window search를 적용했다.
- 최초 실행과 텔레포트 감지 시 전체 탐색으로 복귀하도록 했다.
- 차량 진행 방향과 경로 진행 방향의 dot product 기반 hard filter를 추가했다.
- `Cost = distance_weight * Distance^2 + angle_weight * AngleDiff^2` 비용 함수를 적용했다.
- 두 전역 경로 중 `/current_path` 선택 시 비용 기반 hysteresis를 적용해 잦은 전환을 줄였다.
- global path는 `nav_msgs::PathConstPtr` 기반으로 보관해 callback 이후 불필요한 deep copy를 줄였다.
- 관련 parameter를 `gpp_local_final.launch`에서 조정 가능하게 했다.

### 2차 동치 최적화 완료

- `cos(vehicle_yaw)`, `sin(vehicle_yaw)`를 odometry callback 시점에 한 번 계산해 `vehicle_heading_x/y`로 캐싱하도록 변경했다.
- `back_steps`, `front_steps`, `teleport_ego_dist_thresh_sq`, `max_valid_nearest_dist_sq`를 parameter 초기화 시점에 사전 계산하도록 변경했다.
- candidate 평가에서 전방 위치 dot product filter를 tangent 관련 계산보다 먼저 수행하도록 변경했다.
- global path callback 시 각 waypoint의 tangent vector와 tangent yaw를 미리 계산해 캐싱하도록 변경했다.
- 거리 threshold 비교는 `sqrt(distance_sq)` 대신 `distance_sq`와 threshold 제곱값을 비교하도록 변경했다.
- local path publish message는 멤버 버퍼를 재사용하고, 매 loop마다 `poses.clear()` 후 다시 채우도록 변경했다.

### 동치성 기준

- 후보 통과 조건은 기존과 같은 `forward_dot`, `path_heading_dot`, `angle_diff`, `distance_sq` 값을 사용한다.
- 비용 함수는 기존과 같은 `distance_weight * distance_sq + angle_weight * angle_diff * angle_diff` 형태를 유지한다.
- tangent cache는 기존 `computePathTangent()`와 같은 index 선택 및 수식을 callback 시점에 미리 계산한 값이다.
- publish되는 local path의 frame, stamp, pose position, orientation 값은 기존과 동일하게 유지한다.

### 검증 결과

- `catkin build global_path_planner` 통과를 확인했다.
- `catkin build` 전체 workspace 빌드 통과를 확인했다.
- `xmllint --noout`으로 `gpp_local_final.launch`, `path_maker.launch` XML 문법 통과를 확인했다.
- 난수 기반 5000개 path/pose case에서 기존 후보 평가식과 최적화된 후보 평가식의 best index, cost, distance, angle diff가 동일함을 확인했다.
- 같은 난수 검증에서 기존 local path 좌표 변환식과 최적화된 좌표 변환식의 출력 개수 및 좌표값이 동일함을 확인했다.

## `region_state_publisher.cpp` 분석

### 현재 역할

- `/gps_utm_odom`에서 차량 UTM odometry를 받는다.
- `/region_map/static_map` 서비스를 통해 region map metadata를 가져온다.
- `map_data/region_<arg>.png` 이미지를 읽고, 현재 차량 위치에 대응되는 픽셀의 grayscale 값을 mission state로 변환한다.
- 변환된 state를 `/state` 토픽으로 publish한다.

### 주요 병목 후보

- 매 loop마다 map origin quaternion을 RPY로 변환하고 `cos/sin`을 다시 계산한다.
- 매 loop마다 `Eigen::Matrix4f`와 `Eigen::Vector4f`를 만들어 단순 translation만 수행한다.
- 매 loop마다 image/map scale factor `sx`, `sy`를 다시 계산한다.
- 같은 loop 안에서 state message를 최대 두 번 publish할 수 있다.
- `cout` debug 출력이 10Hz 루프에서 반복되어 runtime jitter와 terminal noise를 만든다.
- `std::stringstream`을 매 publish마다 생성하지만 단순 string 대입이면 충분하다.

### 기능 리스크 후보

- `poseCallback()`에서 queue를 만들고 바로 pop하지만 실제로는 `currentPose = msg`와 동치이다.
- `region_image.empty()`일 때 별도 error log 없이 state publish가 유지될 수 있다.
- map service 실패 시 10Hz로 계속 service call과 실패 로그가 반복된다.
- `gps_state`, `gps_quat_state`, `traffic_light_state`는 계산되지만 최종 state 결정에는 현재 거의 사용되지 않는다.
- `state_string_result()`는 `slow_down_for_traffic_light0~3`을 검사하지만 `state_table`에는 `slow_down_for_traffic_light3`만 들어 있다.
- 파일 시작에 UTF-8 BOM이 있고, 사용하지 않는 include가 많다.

### 동치 최적화 방향

- map metadata를 받은 직후 origin, yaw, `cos/sin`, inverse resolution, image scale factor를 캐싱한다.
- Eigen 행렬 곱셈을 제거하고 `current_x/current_y`를 직접 map 좌표 변환에 사용한다.
- pixel index 계산식과 grayscale-to-state 변환식은 그대로 유지한다.
- publish message는 `msg.data = final_state`로 직접 대입한다.
- debug 출력은 `ROS_INFO_ONCE`, `ROS_WARN_THROTTLE` 등으로 바꿔 반복 출력을 줄인다.
- 중복 publish를 정리하되, 기존 토픽에 마지막 state가 10Hz로 publish되는 동작은 유지한다.

### 동치 최적화 적용 완료

- 기존 전역 변수 중심 구조를 `RegionStatePublisher` class로 정리했다.
- public ROS interface는 유지했다: node name `region_state_publisher`, `/state`, `/gps_utm_odom`, `/gphdt_quat_pub`, `/traffic_light_state`, `/region_map/static_map`.
- `poseCallback()`의 임시 queue를 제거하고 `current_pose_ = msg`로 직접 저장하도록 변경했다.
- map service 성공 시 `MapMetaData`만 보관하고, origin, yaw `cos/sin`, `1/resolution`, image scale factor를 캐싱하도록 변경했다.
- loop마다 생성하던 Eigen matrix/vector 계산을 제거하고, 기존과 같은 결과인 `currentPose.position`을 직접 사용하도록 변경했다.
- grayscale-to-state mapping과 `state_table` 내용은 기존과 동일하게 유지했다.
- `/state`는 loop당 한 번 publish하도록 정리했으며, 최종 state를 10Hz로 publish하는 동작은 유지했다.
- 반복 `cout` debug 출력과 `stringstream` 생성을 제거하고 ROS throttled log 및 직접 string 대입으로 변경했다.
- 사용하지 않는 PCL, visualization, point cloud, laser scan, Eigen include를 제거했고 파일 BOM도 제거했다.

### 검증 결과

- `catkin build global_path_planner` 통과를 확인했다.
- `catkin build` 전체 workspace 빌드 통과를 확인했다.
- 난수 기반 50000개 pose/map/image-size case에서 기존 좌표 변환식과 최적화 좌표 변환식의 `ix`, `iy`가 동일함을 확인했다.
- 0~255 전체 grayscale 값에 대해 기존 state update 로직과 최적화 state update 로직의 `state_string`, `final_state`가 동일함을 확인했다.
- `gpp_local_final.launch`에서는 기존과 동일하게 `region_state_publisher` node가 주석 처리된 상태이며 launch 파일은 변경하지 않았다.

## region 정책 조사

### 코드 기준 정책

- `region_state_publisher`는 `map_data/region_<arg>.png`의 현재 위치 pixel 평균 grayscale 값을 사용한다.
- `point > 240`이면 항상 `go`로 판정한다.
- `point <= 240`이면 `table_index = point / 20` 정수 나눗셈으로 `state_table`을 조회한다.
- 현재 `state_table`은 `go`, `traffic_light_1`, `traffic_light_2`, `traffic_light_3`, `traffic_light_4`, `static_obs`, `gps_fail`, `end`, `slow_down_for_traffic_light3`, `boost`, `delivery`, `slow_downpm`, `stop` 순서이다.
- `/gps_utm_odom` covariance와 `/gphdt_quat_pub`으로 `gps_state`, `gps_quat_state`를 계산하지만, 현재 최종 `/state` 결정에는 반영하지 않는다.
- `/traffic_light_state`도 구독하지만, 현재 최종 `/state` override에는 사용하지 않는다.

### `region_final.png` 실제 적용 상태

- `go`: 96.3623%
- `gps_fail`: 2.6534%
- `static_obs`: 0.6145%
- `traffic_light_2`: 0.1247%
- `traffic_light_1`: 0.0919%
- `end`: 0.0649%
- `traffic_light_4`: 0.0299%
- `traffic_light_3`: 0.0219%
- `slow_downpm`: 0.0144%
- `slow_down_for_traffic_light3`: 0.0128%
- `delivery`: 0.0060%
- `boost`: 0.0031%
- `stop`은 이미지 내 실제 pixel로 존재하지 않는다.

### 영향 범위

- `local_path_selector`는 `/state == gps_fail`이면 `lane_path`를 우선 추종하고, `/state == static_obs`일 때 정적 장애물 회피 판단을 수행한다.
- `traffic_light` 노드는 `/state`가 `traffic_light_1~4`일 때 ROI와 신호 판단에 사용한다.
- `hybrid_tracking` 제어기는 `traffic_light_1~4`, `gps_fail`, `end`, `static_obs` 등에 따라 제어 정책을 바꾼다.
- `object_select`는 `/state != static_obs`일 때 차량 계열 객체만 publish하는 필터 정책을 사용한다.

## `local_path_selector.cpp` 분석

### 현재 역할

- `/local_path1`, `/local_path2`, `/lane_path`, `/current_path`, `/state`, `/sign_state`, `/obstacle_path_info`를 구독한다.
- 최종 주행 경로를 `/selected_path`로 publish한다.
- 동적 장애물 속도 지시를 `/moving_obs`로 publish한다.
- `gps_fail` 상태에서는 `/lane_path`를 우선 publish한다.
- `static_obs` 상태에서는 차선별 전방 장애물을 확인하고, 현재 차선에 장애물이 있으면 반대 차선으로 spline lane change를 명령한다.
- `static_obs`가 아니면 기본적으로 1번 경로를 추종하고, 동적 장애물 거리 기반으로 `/moving_obs = go/slow_down/stop`을 publish한다.

### 주요 정책

- `/current_path`는 `odom_path_publisher`가 선택한 현재 경로 번호로 보고, 차선 변경 완료 확인에 사용한다.
- `hold_until_match_`가 켜지면 `/current_path`가 `commanded_path_`와 같아질 때까지 다른 정책을 무시하고 목표 path만 계속 publish한다.
- `static_obs` 진입 시 현재 path를 유지 의도로 잡고, 이탈 시 무조건 1번 경로로 복귀하려고 한다.
- `static_obs` 중 1번 차선에 장애물이 있으면 2번으로 변경하고, 2번 차선에 장애물이 있으면 1번으로 변경한다.
- 동적 장애물은 현재 path와 같은 `path_number`이면서 전방인 객체 중 가장 가까운 거리로 판단한다.
- 동적 장애물 거리는 `stop_dist=9m`, `slow_down_dist=16m` 기본값을 사용한다.
- 정적 장애물 회피 거리는 `obs_dist_threshold=20m` 기본값을 사용한다.

### 구조 및 최적화 후보

- 파일 앞쪽에 과거 구현이 390줄가량 주석으로 남아 있어 실제 컴파일 코드 파악을 방해한다.
- path callback에서 `nav_msgs::Path` 전체를 매번 deep copy한다.
- `publishPathByNumber()`와 `publishLanePath()`도 매 loop마다 path 전체를 복사한 뒤 header만 수정한다.
- `command_change_to()` 직후 main loop에서 같은 target path를 한 번 더 publish하는 중복 publish가 있다.
- `getFrontObjectDistance()`와 main loop debug 출력이 `std::cout`으로 20Hz 반복 출력되어 runtime jitter와 log noise를 만든다.
- `obstacle_check_static()`과 `getFrontObjectDistance()`가 같은 `/obstacle_path_info` 배열을 별도로 순회한다.
- `cubicInterpolatePath()`에서 `std::pow`를 반복 사용하므로 단순 곱셈으로 동치 최적화 가능하다.
- `current_pose_`는 실제 odom으로 갱신되지 않으며 기본값 `(0,0)`이라, local path가 base_link 기준이면 의도와 맞지만 이름은 혼동을 만든다.

### 기능 리스크 후보

- `state` 초기값이 빈 문자열이어서 `/state` 수신 전에는 `static_obs`가 아닌 상태로 처리되고 1번 경로와 `/moving_obs`가 publish될 수 있다.
- `obj_position == "front"`이면 `distance <= 0`이어도 전방 판정은 통과하지만, 실제 nearest 갱신과 threshold 판정은 `distance > 0`이 필요하다.
- `static_obs`에서 현재 차선과 반대 차선 양쪽에 장애물이 있어도 현재 차선 장애물만 보고 반대 차선으로 변경할 수 있다.
- `static_obs` 종료 시 무조건 1번으로 복귀하는 정책은 경로 상황에 따라 튐을 만들 수 있다.
- `/sign_state`는 구독하지만 현재 정책에 사용되지 않는다.
- `traffic_light_*`, `end`, `boost`, `delivery`, `slow_downpm`, `stop` 같은 region state는 이 노드에서는 별도 경로 정책 없이 기본 1번 경로로 처리된다.
