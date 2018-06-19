cmake_minimum_required(VERSION 2.8.3)
project(grid_map_2d_mapper)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

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
  nav_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/GridMap2DMapper.cfg
  #...
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES grid_map_2d_mapper
  CATKIN_DEPENDS roscpp message_filters nodelet sensor_msgs tf2 tf2_ros nav_msgs dynamic_reconfigure
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(grid_map_2d_mapper src/grid_map_2d_mapper_nodelet.cpp)
target_link_libraries(grid_map_2d_mapper ${catkin_LIBRARIES})
add_dependencies(grid_map_2d_mapper ${PROJECT_NAME}_gencfg)

add_executable(grid_map_2d_mapper_node src/grid_map_2d_mapper_node.cpp)
target_link_libraries(grid_map_2d_mapper_node grid_map_2d_mapper ${catkin_LIBRARIES})
