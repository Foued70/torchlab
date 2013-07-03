cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
cmake_policy(VERSION 2.6)
include(../common.cmake REQUIRED)

# FIND_PACKAGE(Torch REQUIRED)
FIND_PACKAGE(JPEG REQUIRED)
FIND_PACKAGE(PNG QUIET)

# get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
# foreach(dir ${dirs})
#   message(STATUS "dir='${dir}'")
# endforeach()

SET(src ppm.c)
ADD_LUVIT_LIB(ppm "${src}")
TARGET_LINK_LIBRARIES(ppm luaT TH)

SET(src jpeg.c)
# include_directories (${JPEG_INCLUDE_DIR})
ADD_LUVIT_LIB(jpeg "${src}")
TARGET_LINK_LIBRARIES(jpeg luaT TH ${JPEG_LIBRARIES})

if (PNG_FOUND)
    SET(src png.c)
    include_directories (${PNG_INCLUDE_DIR})
    ADD_LUVIT_LIB(png "${src}")
    TARGET_LINK_LIBRARIES(png luaT TH ${PNG_LIBRARIES})
else (PNG_FOUND)
    message ("WARNING: Could not find PNG libraries, PNG wrapper will not be installed")
endif (PNG_FOUND)

SET(src image.c)
ADD_LUVIT_LIB(image "${src}")
TARGET_LINK_LIBRARIES(image luaT TH)
