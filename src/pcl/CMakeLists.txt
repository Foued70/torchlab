CMAKE_MINIMUM_REQUIRED(VERSION 2.6 FATAL_ERROR)
CMAKE_POLICY(VERSION 2.6)

INCLUDE(../common.cmake REQUIRED)

find_package(PCL 1.5 REQUIRED)

INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
ADD_DEFINITIONS(${PCL_DEFINITIONS})


link_libraries(${PCL_LIBRARIES})

LINK_DIRECTORIES(${install_lib})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ftemplate-depth=1024")

SET(package pcl_ffi)

SET(src pcl_ffi.cpp)

ADD_FFI_LIB(${package} ${src})

TARGET_LINK_LIBRARIES(${package} ${PCL_LIBRARIES} luaT TH)


