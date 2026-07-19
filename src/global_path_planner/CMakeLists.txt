cmake_minimum_required(VERSION 3.0.2)
project(global_path_planner)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
  geometry_msgs
  tf
  image_geometry
  pcl_ros
  tf2
  katri_msgs
  morai_msgs
)
find_package(OpenCV REQUIRED)

# find_package(Boost REQUIRED COMPONENTS system)

# catkin_python_setup()

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   nav_msgs#   sensor_msgs#   std_msgs
# )

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES global_path_planner
  CATKIN_DEPENDS nav_msgs pcl_ros roscpp rospy sensor_msgs std_msgs tf2
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/global_path_planner.cpp
# )

add_executable(odom_path_publisher src/odom_path_publisher.cpp)
add_dependencies(odom_path_publisher ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom_path_publisher ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(local_path_selector src/local_path_selector.cpp)
add_dependencies(local_path_selector ${catkin_EXPORTED_TARGETS})
target_link_libraries(local_path_selector ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(ai_local_path_selector src/NOT_USED/ai_local_path_selector.cpp)
add_dependencies(ai_local_path_selector ${catkin_EXPORTED_TARGETS})
target_link_libraries(ai_local_path_selector ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(region_state_publisher src/region_state_publisher.cpp)
add_dependencies(region_state_publisher ${catkin_EXPORTED_TARGETS})
target_link_libraries(region_state_publisher ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(obstacle_info_publisher src/obstacle_info_publisher.cpp)
add_dependencies(obstacle_info_publisher ${catkin_EXPORTED_TARGETS})
target_link_libraries(obstacle_info_publisher ${catkin_LIBRARIES})

add_executable(acc_node src/acc_node.cpp)
add_dependencies(acc_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(acc_node ${catkin_LIBRARIES})

add_executable(ai_region_state_publisher src/NOT_USED/ai_region_state_publisher.cpp)
add_dependencies(ai_region_state_publisher ${catkin_EXPORTED_TARGETS})
target_link_libraries(ai_region_state_publisher ${catkin_LIBRARIES} ${OpenCV_LIBS})

# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_executable(${PROJECT_NAME}_node src/global_path_planner_node.cpp)
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
catkin_install_python(PROGRAMS
  scripts/global_path_publisher.py
  scripts/gps_imu_fusion.py
  scripts/ll2utm.py
  scripts/path_maker_sm.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_global_path_planner.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
