cmake_minimum_required(VERSION 2.6)
project (octree_change_detection)

SET(GCC_COVERAGE_COMPILE_FLAGS "-Wall -Wextra -g")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fopenmp ${GCC_COVERAGE_COMPILE_FLAGS}")


find_package(rosbag REQUIRED)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${rosbag_INCLUDE_DIRS})
add_executable(octree_change_detection octree_change_detection.cpp peak_detector.cpp)
target_link_libraries(octree_change_detection ${rosbag_LIBRARIES})
target_link_libraries(octree_change_detection ${PCL_LIBRARIES})
