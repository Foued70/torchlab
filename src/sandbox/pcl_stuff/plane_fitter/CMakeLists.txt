cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

SET (CMAKE_CXX_COMPILER "/Users/lihui815/cloudlab/build/usr/local/bin/g++")

project(plane_fitter)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (plane_fitter plane_fitter.cpp)
target_link_libraries (plane_fitter ${PCL_LIBRARIES})
