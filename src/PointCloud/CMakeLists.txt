CMAKE_MINIMUM_REQUIRED(VERSION 2.6 FATAL_ERROR)
CMAKE_POLICY(VERSION 2.6)
INCLUDE(../common.cmake REQUIRED)

SET(package pointcloud)
SET(src pointcloud.c)

ADD_FFI_LIB(${package} ${src})

TARGET_LINK_LIBRARIES(${package})