cmake_minimum_required(VERSION 3.0.2)
project(euc_cluster)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  std_msgs
  sensor_msgs
)

find_package(PCL 1.8 REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES euc_cluster
#  CATKIN_DEPENDS pcl_conversions ros_pcl roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 src
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME}_cluster src/findeuc.cpp)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})




add_executable(${PROJECT_NAME} src/main.cpp)

target_link_libraries(${PROJECT_NAME}_cluster ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PROJECT_NAME}_cluster ${PCL_LIBRARIES}) 