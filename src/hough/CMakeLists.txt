CMAKE_MINIMUM_REQUIRED(VERSION 2.6 FATAL_ERROR)
CMAKE_POLICY(VERSION 2.6)

INCLUDE(../common.cmake REQUIRED)

SET(luasrc init.lua)
SET(src hough.c)

ADD_TORCH_PACKAGE(hough "${src}" "${luasrc}" "Floored Hough Transform")

TARGET_LINK_LIBRARIES(hough luaT TH)
