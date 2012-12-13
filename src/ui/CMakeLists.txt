cmake_minimum_required (VERSION 2.6 FATAL_ERROR)
CMAKE_POLICY(VERSION 2.6)
find_package(Torch REQUIRED)

#SET(CMAKE_CXX_FLAGS "-std=gnu++11")
FILE(GLOB luasrc *.lua)
SET(src ui.c)

find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})

find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIR})

ADD_TORCH_PACKAGE(ui "${src}" "${luasrc}" "Floored Viewer")

target_link_libraries(ui luaT TH ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})
