cmake_minimum_required(VERSION 2.8.3)
project(armadillo_hl_interface)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  roscpp
  rospy
  std_msgs
  move_base_msgs
  actionlib
  tf
  costmap_2d
  base_local_planner

  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_action_files(
  DIRECTORY action
  FILES SimpleDriver.action Torso.action SpeechToText.action TextToSpeech.action
)

add_message_files(
)

add_service_files(
  FILES
  TextToSpeech.srv
)

generate_messages(
  DEPENDENCIES std_msgs geometry_msgs actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS actionlib_msgs
)

add_executable(testing
  src/fsm.cpp
  src/head_interface.cpp
  src/arm_interface.cpp
  src/object_handler.cpp
  src/driver_interface.cpp
  src/torso_interface.cpp
  src/speech_interface.cpp
  src/test.cpp)

target_link_libraries(testing ${catkin_LIBRARIES})
