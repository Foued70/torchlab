CMAKE_MINIMUM_REQUIRED(VERSION 2.6 FATAL_ERROR)
CMAKE_POLICY(VERSION 2.6)
INCLUDE(../common.cmake REQUIRED)

find_package(Boost 1.40.0 QUIET COMPONENTS serialization graph REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

SET(package pointcloud)
SET(src pointcloud.cpp)

ADD_FFI_LIB(${package} ${src})

TARGET_LINK_LIBRARIES(${package} ${Boost_LIBRARY_DIRS} luaT TH)