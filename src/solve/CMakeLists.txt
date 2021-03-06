cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
cmake_policy(VERSION 2.6)
include(../common.cmake REQUIRED)

FILE(GLOB src "*.cc")

set(EIGEN_INCLUDE_DIR ${install_root}/include/eigen3/)
include_directories(${EIGEN_INCLUDE_DIR})

set(CERES_INCLUDE_DIR ${install_root}/include/)
include_directories(${CERES_INCLUDE_DIR})

set(CERES_INTERNAL_INCLUDE_DIR ${install_root}/include/ceres/internal)
include_directories(${CERES_INTERNAL_INCLUDE_DIR})

IF (APPLE)
   find_library(VECLIB_LIBRARY veclib)
ELSE (APPLE)
     message( FATAL_ERROR "FIXME figure out ceres linking on non-apple platforms" )
     # FIXME perhaps with use some of
     # find_library(BLAS_LIBRARY blas)
     # find_library(LAPACK_LIBRARY lapack)
ENDIF(APPLE)

set(CERES_LIBRARY ceres)

# perhaps we can remove some of these mostly handle file which we do in torch anyways.
set(GOOGLE_LIBS glog gflags protobuf)

# many different ways to use ceres, suitesparse is LGPL'd full speedup
# (which we might not be able to exploit anyway which means we could
# use ceres without)
set(NUMERICAL_LIBS cholmod ccolamd camd colamd amd suitesparseconfig cxsparse)

set(CERES_DEPENDENCIES ${GOOGLE_LIBS} ${VECLIB_LIBRARY} ${NUMERICAL_LIBS} gomp)


FIND_PACKAGE( OpenCV REQUIRED )

LINK_DIRECTORIES(${install_lib} ${OPENCV_LIBRARY_DIRS})
INCLUDE_DIRECTORIES(${OPENCV_INCLUDE_DIRS})
LINK_DIRECTORIES(${OPENCV_LIBRARY_DIRS} ${CUDA_INCLUDE_DIRS})


SET(src alignment_adjustment.cc)

ADD_FFI_LIB(solve "${src}")
TARGET_LINK_LIBRARIES(solve luaT TH ${OpenCV_LIBS} ${CERES_LIBRARY} ${CERES_DEPENDENCIES})

