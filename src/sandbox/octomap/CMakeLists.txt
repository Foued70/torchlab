cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(make_octomap)

find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

add_executable (make_octomap make_octomap.cpp)

install(TARGETS make_octomap RUNTIME DESTINATION bin)
