CMAKE_MINIMUM_REQUIRED(VERSION 2.6 FATAL_ERROR)
CMAKE_POLICY(VERSION 2.6)

INCLUDE(../common.cmake REQUIRED)

set(POLYCLIPPING_INCLUDE_DIR ${install_root}/include/polyclipping)
set(POLYCLIPPING_LIBRARY_DIR ${install_root}/lib/libpolyclipping.dylib)

message("include dir")
message(${POLYCLIPPING_INCLUDE_DIR})

INCLUDE_DIRECTORIES(${POLYCLIPPING_INCLUDE_DIR})
link_libraries(${POLYCLIPPING_LIBRARY_DIR})

LINK_DIRECTORIES(${install_lib})

#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ftemplate-depth=1024")

SET(package polyclipping_ffi)

SET(src polyclipping_ffi.cpp)

ADD_FFI_LIB(${package} ${src})

TARGET_LINK_LIBRARIES(${package} ${POLYCLIPPING_LIBRARY_DIR} luaT TH)


