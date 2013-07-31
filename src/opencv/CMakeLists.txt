CMAKE_MINIMUM_REQUIRED(VERSION 2.6 FATAL_ERROR)
CMAKE_POLICY(VERSION 2.6)

INCLUDE(../common.cmake REQUIRED)

FIND_PACKAGE( OpenCV REQUIRED )

LINK_DIRECTORIES(${install_lib})

SET(package copencv)

SET(src copencv.cpp)

ADD_FFI_LIB(${package} ${src})

TARGET_LINK_LIBRARIES(${package} ${OpenCV_LIBS} luaT TH)
