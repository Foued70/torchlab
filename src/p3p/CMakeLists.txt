CMAKE_MINIMUM_REQUIRED(VERSION 2.6 FATAL_ERROR)
CMAKE_POLICY(VERSION 2.6)
include(../common.cmake REQUIRED)

SET(src p3p.c)

ADD_TORCH_PACKAGE(p3p "${src}" "" "Floored p3p Functions")

