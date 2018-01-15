#
cmake_minimum_required(VERSION 2.8.3)
project(object_identification)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        geometry_msgs
        image_transport
        cv_bridge
        pcl_conversions
        pcl_ros
        tf
        tf_conversions
        trajectory_msgs
        gazebo_ros
        message_generation
        )
find_package(gazebo REQUIRED)

## Generate messages in the 'msg' folder
add_service_files(
  FILES
  find_obj.srv
  switch_topic.srv
  
)
add_message_files(
  FILES
  find_results.msg
)

## Generate services in the 'srv' folder


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
   std_msgs
   geometry_msgs
 )


#
catkin_package(
        CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs dynamic_reconfigure
        gazebo_ros
        message_generation
        message_runtime
        DEPENDS gazebo
)

# include_directories(include)
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
        ${GAZEBO_LIBRARY_DIRS}
        ${catkin_LIBRARY_DIRS}
)

#
# Yolo detector dependancies added below.
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS} catkin_ws/src/armadillo_bgu_bundle/object_identification/include)
#
#
add_executable(find_objects src/find_objects.cpp)
add_dependencies(find_objects ${PROJECT_NAME}_gencfg)
add_dependencies(find_objects ${PROJECT_NAME}_gencpp)
#
# message("----------------------------------------")
# message(${object_identification_SOURCE_DIR}/src/libobj_pos.a) # tested for relative path
# message("----------------------------------------")
#
target_link_libraries(find_objects ${catkin_LIBRARIES} ${OpenCV_LIBS} -pthread ${object_identification_SOURCE_DIR}/src/libobj_pos.a)
# (ends).

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        FILES_MATCHING PATTERN "*.launch"
        )
#
#
