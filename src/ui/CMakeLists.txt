cmake_minimum_required (VERSION 2.6 FATAL_ERROR)
CMAKE_POLICY(VERSION 2.6)
find_package(Torch REQUIRED)

set(CMAKE_CXX_COMPILER ../../../build/usr/local/bin/g++)

FILE(GLOB luasrc *.lua)
SET(src qui.cpp ScanWidget.cpp)
SET( moc_headers ScanWidget.h )

SET( QT_USE_QTOPENGL TRUE )

find_package(Qt4 REQUIRED)
INCLUDE( ${QT_USE_FILE} )
QT4_WRAP_CPP( moc_srcs ${moc_headers} )
SET(src ${src} ${moc_srcs} )


include_directories(${QT_INCLUDE_DIR})

find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})

find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIR})

ADD_TORCH_PACKAGE(ui "${src}" "${luasrc}" "Floored Viewer")

target_link_libraries(ui luaT TH ${OPENGL_LIBRARIES} ${QT_LIBRARIES} )
