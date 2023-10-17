cmake_minimum_required(VERSION 3.0.2)
project(car_planner)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  tf
  mapping
  traj_utils
  message_generation
  cv_bridge
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES car_planner
 CATKIN_DEPENDS roscpp rospy std_msgs traj_utils mapping
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_executable(car_planner_node
  src/planner_node.cpp 
  src/replan_fsm.cpp
  src/planner_manager.cpp
  )
target_link_libraries(car_planner_node 
  ${catkin_LIBRARIES}
  )