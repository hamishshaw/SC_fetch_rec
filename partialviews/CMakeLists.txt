cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
project(PCL_partial_views)

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

add_library(${PROJECT_NAME}_ClusterSearch src/ClusterSearch.cpp)
 
add_executable(partialViews "src/main.cpp")
add_executable(train "src/train.cpp")
 
target_link_libraries (partialViews ${PCL_LIBRARIES} ${VTK_LIBRARIES})
target_link_libraries(train ${PCL_LIBRARIES} ${Boost_LIBRARIES} FLANN::FLANN ${HDF5_LIBRARIES} ${VTK_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_ClusterSearch ${PCL_LIBRARIES} ${Boost_LIBRARIES} FLANN::FLANN ${HDF5_LIBRARIES} ${VTK_LIBRARIES})

add_executable(nearest_neighbors "src/test.cpp")
target_link_libraries(nearest_neighbors ${PCL_LIBRARIES} ${Boost_LIBRARIES} FLANN::FLANN ${HDF5_LIBRARIES} ${PROJECT_NAME}_ClusterSearch)