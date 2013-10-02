CMAKE_MINIMUM_REQUIRED(VERSION 2.6 FATAL_ERROR)
CMAKE_POLICY(VERSION 2.6)
include(../common.cmake REQUIRED)

SET(src pointcloud.c)

ADD_FFI_LIB(pointcloud "${src}")