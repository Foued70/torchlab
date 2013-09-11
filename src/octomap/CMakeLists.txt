CMAKE_MINIMUM_REQUIRED(VERSION 2.6 FATAL_ERROR)
CMAKE_POLICY(VERSION 2.6)

INCLUDE(../common.cmake REQUIRED)

find_package(octomap REQUIRED)

include_directories(${OCTOMAP_INCLUDE_DIRS})

link_libraries(${OCTOMAP_LIBRARIES})

LINK_DIRECTORIES(${install_lib})

SET(package octomap_ffi)

SET(src octomap_ffi.cpp)

ADD_FFI_LIB(${package} ${src})

TARGET_LINK_LIBRARIES(${package} ${OCTOMAP_LIBRARIES} luaT TH)
