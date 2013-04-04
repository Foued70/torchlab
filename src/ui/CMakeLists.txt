cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
cmake_policy(VERSION 2.6)
include(../common.cmake REQUIRED)

find_library(COCOA_LIBRARY Cocoa)

FILE(GLOB src "*.cpp" )

SET( QT_USE_QTOPENGL TRUE )
find_package(Qt4 REQUIRED)
INCLUDE( ${QT_USE_FILE} )

SET( moc_headers GLWidget.h )
QT4_WRAP_CPP( moc_srcs ${moc_headers} )
SET(src ${src} ${moc_srcs} )


include_directories(${QT_INCLUDE_DIR})

find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})

SET( precomp_object_files
  ${CMAKE_CURRENT_BINARY_DIR}/core_profile_attributes.o
)

ADD_LIBRARY(
  precomp
  STATIC
  EXCLUDE_FROM_ALL
  ${precomp_object_files}
)

SET_SOURCE_FILES_PROPERTIES(
  ${precomp_object_files}
  PROPERTIES
  EXTERNAL_OBJECT true # to say that "this is actually an object file, so it should not be compiled, only linked"
  GENERATED true       # to say that "it is OK that the obj-files do not exist before build time"
)

SET_TARGET_PROPERTIES(
  precomp
  PROPERTIES
  LINKER_LANGUAGE C # Or else we get an error message, because cmake can't figure out from the ".o"-suffix that it is a C-linker we need.
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib
)
  
ADD_CUSTOM_COMMAND(
  OUTPUT ${precomp_object_files}
  WORKING_DIRECTORY  ${CMAKE_CURRENT_BINARY_DIR}
  COMMAND  clang -c ${CMAKE_CURRENT_SOURCE_DIR}/core_profile_attributes.mm -o ${CMAKE_CURRENT_BINARY_DIR}/core_profile_attributes.o
)

ADD_TORCH_PACKAGE(ui "${src}" "" "Floored Viewer")

target_link_libraries(ui precomp luaT TH ${COCOA_LIBRARY} ${OPENGL_LIBRARIES} ${QT_LIBRARIES} )
