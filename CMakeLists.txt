cmake_minimum_required(VERSION 3.0.2)
project(merging_cloud)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
  message_filters
  geometry_msgs
  nav_msgs
  tf
  message_generation
  std_msgs
)

add_message_files(
  FILES
  pose.msg
  key_m.msg
  plane.msg
  planes.msg

)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  nav_msgs
) #custom message에 추가할 메시지형태 선언

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs nav_msgs
)

find_package(GTSAM REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(merging_cloud_node src/merging_cloud.cpp)
target_link_libraries(merging_cloud_node ${catkin_LIBRARIES})