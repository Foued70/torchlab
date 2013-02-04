cmake_minimum_required (VERSION 2.6 FATAL_ERROR)
CMAKE_POLICY(VERSION 2.6)
find_package(Torch REQUIRED)

set(CMAKE_CXX_COMPILER ../../../build/usr/local/bin/g++)
set(CMAKE_CXX_FLAGS "-std=gnu++11")

# FILE(GLOB luasrc "*.lua")
set(luasrc "")
FILE(GLOB src "*.c")

ADD_TORCH_PACKAGE(protobuf "${src}" "${luasrc}" "Protocol Buffers for Lua")
target_link_libraries(protobuf luaT TH )
