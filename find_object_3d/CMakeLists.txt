cmake_minimum_required(VERSION 3.0.2)
project(find_object_3d)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS src
#  LIBRARIES transform_pcl
  CATKIN_DEPENDS pcl_conversions roscpp sensor_msgs geometry_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

find_package(PCL 1.13.1 REQUIRED)
find_package(PCL REQUIRED apps)
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})


find_package(HDF5 REQUIRED)
find_package(FLANN REQUIRED)

include_directories(SYSTEM
 ${HDF5_INCLUDE_DIR}
)
find_package(VTK REQUIRED)

add_library(${PROJECT_NAME}_cluster src/findeuc.cpp)
target_link_libraries(${PROJECT_NAME}_cluster ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_find_object_3d src/ClusterSearch.cpp)
target_link_libraries(${PROJECT_NAME}_find_object_3d ${Boost_LIBRARIES} FLANN::FLANN ${HDF5_LIBRARIES} ${VTK_LIBRARIES})

add_executable(${PROJECT_NAME} src/main.cpp )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${PROJECT_NAME}_cluster ${PROJECT_NAME}_find_object_3d)