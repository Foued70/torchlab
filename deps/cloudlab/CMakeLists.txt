cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
cmake_policy(VERSION 2.6)
include(../../src/common.cmake REQUIRED)

CONFIGURE_FILE(cloudlab.lua "${CMAKE_CURRENT_BINARY_DIR}/cloudlab.lua")


CONFIGURE_FILE(cook "${CMAKE_CURRENT_BINARY_DIR}/cook")

INSTALL(
  PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/cook
  DESTINATION bin
)


