cmake_minimum_required(VERSION 3.0.2)
project(second_project)

## Compile as C++14
add_compile_options(-std=c++14)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    tf            # legacy TF
    message_filters
    sensor_msgs
    nav_msgs
    geometry_msgs
    gmapping      # slam_gmapping node
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    tf
    message_filters
    sensor_msgs
    nav_msgs
    geometry_msgs
    gmapping
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

########
## Nodes
########

## 1) odom_to_tf_node
add_executable(odom_to_tf_node
  src/odom_to_tf_node.cpp
)
target_link_libraries(odom_to_tf_node
  ${catkin_LIBRARIES}
)

## 2) laser_merger_node
add_executable(laser_merger_node
  src/laser_merger_node.cpp
)
target_link_libraries(laser_merger_node
  ${catkin_LIBRARIES}
)
