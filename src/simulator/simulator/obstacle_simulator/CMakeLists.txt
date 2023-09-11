
cmake_minimum_required(VERSION 2.8.3)
project(obstacle_simulator)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  visualization_msgs
  tf
  pcl_conversions
  pose_utils
  Eigen3 REQUIRED
)


catkin_package(
 INCLUDE_DIRS 
 LIBRARIES 
 DEPENDS OpenCV Eigen Boost

#  DEPENDS system_lib
)

find_package(PCL REQUIRED)
find_package(Armadillo REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})


include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
)

add_executable(obstacle_simulator_node src/obstacle_sim.cpp)

target_link_libraries(obstacle_simulator_node
   ${catkin_LIBRARIES}
   ${ARMADILLO_LIBRARIES}
   ${PCL_LIBRARIES}
   pose_utils
)

add_executable(ply2pcd
  src/ply2pcd.cpp
)

target_link_libraries(ply2pcd
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)