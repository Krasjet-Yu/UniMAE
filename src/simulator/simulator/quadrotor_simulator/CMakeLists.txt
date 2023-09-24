cmake_minimum_required(VERSION 2.8.3)
project(quadrotor_simulator)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  std_msgs
  visualization_msgs
  tf
  pose_utils
  message_generation
)
find_package(Eigen3 REQUIRED)
find_package(Armadillo REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})

set(PLAN_UTILS_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include/plan_utils)
# set(GEO_UITLS_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include/geo_utils2d)

catkin_package(
#  INCLUDE_DIRS include ${PLAN_UTILS_INCLUDE_DIRS} 
#  ${GEO_UTILS_INCLUDE_DIRs}
 CATKIN_DEPENDS roscpp std_msgs
)

include_directories( include 
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PLAN_UTILS_INCLUDE_DIRS}
  # ${GEO_UTILS_INCLUDE_DIRs}
)

add_executable (quadrotor_simulator_node src/quadrotor_sim.cpp )
target_link_libraries(quadrotor_simulator_node
  ${catkin_LIBRARIES}
  ${ARMADILLO_LIBRARIES}
  pose_utils)
