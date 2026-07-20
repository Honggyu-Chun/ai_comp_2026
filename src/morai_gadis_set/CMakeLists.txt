cmake_minimum_required(VERSION 3.0.2)
project(morai_gadis_set)

find_package(catkin REQUIRED COMPONENTS
  morai_msgs
  rospy
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/morai_udp_bridge.py
  scripts/control_udp_pub.py
  scripts/object_info_udp_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
