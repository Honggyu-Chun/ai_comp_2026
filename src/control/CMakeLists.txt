cmake_minimum_required(VERSION 2.8.3)
project(control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  morai_msgs
  roscpp
  std_msgs
  nav_msgs
  tf
  ackermann_msgs
  osqp_vendor
)

find_library(OSQP_LIBRARY
  NAMES osqp
  PATHS /opt/ros/noetic/lib
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tracking_robo
  CATKIN_DEPENDS morai_msgs roscpp std_msgs nav_msgs tf ackermann_msgs osqp_vendor
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mpc_control src/mpc_control.cpp)
target_link_libraries(mpc_control
  ${catkin_LIBRARIES}
  ${OSQP_LIBRARY}
)

add_executable(tracking_error_reporter src/tracking_error_reporter.cpp)
target_link_libraries(tracking_error_reporter
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/acceleration_test.py
  scripts/braking_distance_test.py
  scripts/mpc_lap_tuner.py
  scripts/purepursuit.py
  scripts/purepursuit_ego.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
