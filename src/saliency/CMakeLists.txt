CMAKE_MINIMUM_REQUIRED(VERSION 2.6 FATAL_ERROR)
CMAKE_POLICY(VERSION 2.6)
include(../common.cmake REQUIRED)

SET(luasrc init.lua)
SET(src saliency.c)


ADD_TORCH_PACKAGE(saliency "${src}" "${luasrc}" "Saliency using Entropy on Integer (integral) Histograms")
TARGET_LINK_LIBRARIES(saliency luaT TH)
