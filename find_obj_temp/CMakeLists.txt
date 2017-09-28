cmake_minimum_required(VERSION 2.8.3)
project(find_obj_temp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  tf
  pcl_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES find_obj_temp
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(find_obj_temp src/find_object.cpp src/tinyxml2.cpp)

target_link_libraries(find_obj_temp ${catkin_LIBRARIES})