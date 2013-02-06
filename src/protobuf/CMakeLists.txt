cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
cmake_policy(VERSION 2.6)
include(../common.cmake REQUIRED)


FILE(GLOB src "*.c")

ADD_TORCH_PACKAGE(protobuf "${src}" "" "Protocol Buffers for Lua")
target_link_libraries(protobuf luaT TH )
