CMAKE_MINIMUM_REQUIRED(VERSION 2.6 FATAL_ERROR)
CMAKE_POLICY(VERSION 2.6)

INCLUDE(../../../common.cmake REQUIRED)

LINK_DIRECTORIES(${install_lib})

SET(package pvm)

SET(src point_vom.cpp)


ADD_FFI_LIB(${package} ${src})

TARGET_LINK_LIBRARIES(${package} luaT TH)