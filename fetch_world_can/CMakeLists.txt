cmake_minimum_required(VERSION 3.0.2)
project(fetch_can)

find_package(catkin REQUIRED COMPONENTS
  fetch_gazebo
  gazebo_ros
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  control_msgs
  roscpp
  tf2
  tf2_ros
  tf2_geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(start_position "src/start_position.cpp")
target_link_libraries(start_position ${catkin_LIBRARIES})

add_executable(can_grab "src/can_grab.cpp")
target_link_libraries(can_grab ${catkin_LIBRARIES})

