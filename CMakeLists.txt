cmake_minimum_required(VERSION 2.8.3)
project(lsd_slam_to_pcl)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -ggdb -gdwarf-3")

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  pcl_ros
  pluginlib
  roscpp
  std_msgs
)

add_message_files(
  FILES
  keyframeMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES lsd_slam_to_pcl_nodelet
   CATKIN_DEPENDS message_generation message_runtime pcl_ros pluginlib roscpp std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(lsd_slam_to_pcl_nodelet
  src/lsd_slam_to_pcl.cpp
  src/lsd_slam_to_pcl_nodelet.cpp
)

add_dependencies(lsd_slam_to_pcl_nodelet lsd_slam_to_pcl_gencpp)

target_link_libraries(lsd_slam_to_pcl_nodelet
  ${catkin_LIBRARIES}
)

install(TARGETS lsd_slam_to_pcl_nodelet
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY plugins
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)