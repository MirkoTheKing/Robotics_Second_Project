cmake_minimum_required(VERSION 3.0.2)
project(second_project)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  nav_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(odom_tf_broadcaster src/odom_tf_broadcaster.cpp)
target_link_libraries(odom_tf_broadcaster ${catkin_LIBRARIES})
add_dependencies(odom_tf_broadcaster ${catkin_EXPORTED_TARGETS})
