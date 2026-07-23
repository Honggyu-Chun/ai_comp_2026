cmake_minimum_required(VERSION 3.0.2)
project(competition_vehicle_interface)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  morai_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    morai_msgs
    rospy
    sensor_msgs
    std_msgs
)

catkin_install_python(PROGRAMS
  scripts/competition_morai_udp_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
