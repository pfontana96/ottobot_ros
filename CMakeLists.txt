cmake_minimum_required(VERSION 3.0.2)
project(ottobot_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  cv_bridge
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_filters
  tf2
  tf2_ros
  tf2_eigen
  pcl_ros
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

# Import OpenCV library
find_package(OpenCV 4 REQUIRED)
if(OpenCV_FOUND)
  # include_directories(${OpenCV_INCLUDE_DIRS})
else()
  message(FATAL_ERROR "OpenCV > 4 not found.")
endif(OpenCV_FOUND)

catkin_package(
  INCLUDE_DIRS include
# LIBRARIES dvo
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
  DEPENDS OpenCV
)

add_subdirectory(extern/visual-odometry)

# dvo_node
add_executable(node_camera_pose_controller src/node_camera_pose_controller.cpp)

if(DVO_CHRONO)
  message(STATUS "Building with DVO_CHRONO")
  target_compile_definitions(vo PRIVATE DVO_CHRONO)
endif(DVO_CHRONO)

target_include_directories(node_camera_pose_controller PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${DVO_INCLUDE_DIR}
  # ${PCL_INCUDE_DIRS}
  include
)

target_link_libraries(node_camera_pose_controller 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  # ${PCL_LIBRARIES} 
  # cudart
  vo
)

add_dependencies(node_camera_pose_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})