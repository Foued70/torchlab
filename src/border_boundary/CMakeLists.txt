cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
CMAKE_POLICY(VERSION 2.6)
INCLUDE(../common.cmake REQUIRED)
FIND_PACKAGE( OpenMP REQUIRED)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lgomp")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")
SET(package border_boundary)

SET(src border_ffi.cpp)

ADD_FFI_LIB(${package} ${src})

TARGET_LINK_LIBRARIES(${package} luaT TH)
