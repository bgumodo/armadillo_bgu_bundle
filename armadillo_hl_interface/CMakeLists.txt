cmake_minimum_required(VERSION 2.8.3)
project(armadillo_hl_interface)
add_compile_options(-std=c++11)

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
add_executable(speech_pub 
src/speech_pub.cpp
src/speech_interface.cpp)
target_link_libraries(speech_pub ${catkin_LIBRARIES})

add_executable(drive_and_pick_speech src/drive_and_pick_speech.cpp)
target_link_libraries(drive_and_pick_speech ${catkin_LIBRARIES})

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

  add_executable(robot_testing
  src/fsm.cpp
  src/head_interface.cpp
  src/arm_interface.cpp
  src/object_handler.cpp
  src/driver_interface.cpp
  src/torso_interface.cpp
  src/speech_interface.cpp
  src/robot_test.cpp)
  

target_link_libraries(robot_testing ${catkin_LIBRARIES})

add_dependencies( speech_pub ${PROJECT_NAME}_generate_messages_cpp )
add_dependencies( drive_and_pick_speech ${PROJECT_NAME}_generate_messages_cpp )
add_dependencies(testing ${PROJECT_NAME}_generate_messages_cpp )
add_dependencies(robot_testing ${PROJECT_NAME}_generate_messages_cpp )
