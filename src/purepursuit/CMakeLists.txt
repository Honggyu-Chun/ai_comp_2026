cmake_minimum_required(VERSION 2.8.3)
project(purepursuit)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  morai_msgs
  roscpp
  std_msgs
  nav_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tracking_robo
  CATKIN_DEPENDS morai_msgs roscpp std_msgs nav_msgs tf
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## 실행 파일 추가 (REAL) - 중복 해결
add_executable(purepursuit_robo_real src/purepursuit_robo_real.cpp)
target_link_libraries(purepursuit_robo_real
  ${catkin_LIBRARIES}
)

add_executable(hybrid_robo_real src/hybrid_robo_real.cpp)
target_link_libraries(hybrid_robo_real
  ${catkin_LIBRARIES}
)

add_executable(hybrid_real_curv src/hybrid_real_curv.cpp)
target_link_libraries(hybrid_real_curv
  ${catkin_LIBRARIES}
)

add_executable(hybrid_curv_real src/hybrid_curv_real.cpp)
target_link_libraries(hybrid_curv_real
  ${catkin_LIBRARIES}
)
