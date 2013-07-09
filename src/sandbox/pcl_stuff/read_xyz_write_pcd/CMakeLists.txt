cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

SET (CMAKE_CXX_COMPILER "/Users/lihui815/cloudlab/build/usr/local/bin/g++")

project(read_xyz_write_pcd)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (read_xyz_write_pcd read_xyz_write_pcd.cpp)
target_link_libraries (read_xyz_write_pcd ${PCL_LIBRARIES})
