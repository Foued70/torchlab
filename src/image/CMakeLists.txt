CMAKE_MINIMUM_REQUIRED(VERSION 2.6 FATAL_ERROR)
CMAKE_POLICY(VERSION 2.6)
include(../common.cmake REQUIRED)

SET(sal saliency.c)
SET(hgh hough.c)

ADD_LUVIT_LIB(saliency "${sal}")
ADD_LUVIT_LIB(hough "${hgh}")
TARGET_LINK_LIBRARIES(saliency luaT TH)
TARGET_LINK_LIBRARIES(hough luaT TH)
