CMAKE_MINIMUM_REQUIRED(VERSION 2.6 FATAL_ERROR)
CMAKE_POLICY(VERSION 2.6)

INCLUDE(../../../common.cmake REQUIRED)

LINK_DIRECTORIES(${install_lib})

SET(package plane_tools)

SET(src plane_tools.cpp)


ADD_FFI_LIB(${package} ${src})

TARGET_LINK_LIBRARIES(${package} luaT TH)