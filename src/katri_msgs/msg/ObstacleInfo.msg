# 자차 base_link 상대좌표로 변환된 장애물 하나. 모든 값 SI 단위.
# 원본: morai_msgs/ObjectStatus (/Object_topic), 변환: obstacle_info_publisher
int32   id
int32   type          # 0: 보행자, 1: NPC 차량, 2: 정적 장애물 (morai ObjectStatus.type)
bool    is_pedestrian
float64 x_m           # base_link 전방(+x) [m]
float64 y_m           # base_link 좌측(+y) [m]  (REP-103)
float64 yaw_rad       # 자차 기준 상대 heading, 반시계 + [rad]
float64 speed_mps     # 속도 크기 [m/s]
float64 width_m
float64 length_m
float64 distance_m    # sqrt(x_m^2 + y_m^2), 편의값
