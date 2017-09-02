cmake_minimum_required(VERSION 2.8.3)
project(grid_map_2d_mapper)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  grid_map_ros
  message_filters
  nodelet
  roscpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  laser_geometry
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES grid_map_2d_mapper
  CATKIN_DEPENDS roscpp message_filters nodelet sensor_msgs tf2 tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(grid_map_2d_mapper src/grid_map_2d_mapper_nodelet.cpp)
target_link_libraries(pointcloud_to_laserscan ${catkin_LIBRARIES})

add_executable(grid_map_2d_mapper_node src/grid_map_2d_mapper_node.cpp)
target_link_libraries(grid_map_2d_mapper_node grid_map_2d_mapper ${catkin_LIBRARIES})
