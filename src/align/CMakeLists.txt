cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
cmake_policy(VERSION 2.6)
include(../common.cmake REQUIRED)

SET(CMAKE_CXX_FLAGS "-DCERES_HAVE_PTHREAD -DCERES_HAVE_RWLOCK -DCERES_USE_OPENMP -fopenmp \"-DCERES_HASH_NAMESPACE_START=namespace std { namespace tr1 {" "-DCERES_HASH_NAMESPACE_END=}}\"")

set(EIGEN_INCLUDE_DIR ${install_root}/include/eigen3/)
include_directories(${EIGEN_INCLUDE_DIR})

set(CERES_INCLUDE_DIR ${install_root}/include/)
include_directories(${CERES_INCLUDE_DIR})

set(CERES_INTERNAL_INCLUDE_DIR ${install_root}/include/ceres/internal)
include_directories(${CERES_INTERNAL_INCLUDE_DIR})

# FIXME THIS needs an IF APPLE wrapper

find_library(VECLIB_LIBRARY veclib)

# find_library(BLAS_LIBRARY blas)
# find_library(LAPACK_LIBRARY lapack)

set(CERES_LIBRARY ceres)

set(CERES_LIBRARY_DEPENDENCIES ${VECLIB_LIBRARY} glog gflags cholmod ccolamd camd colamd amd suitesparseconfig cxsparse gomp protobuf )

ADD_EXECUTABLE(simple_bundle_adjuster
                simple_bundle_adjuster.cc)
TARGET_LINK_LIBRARIES(simple_bundle_adjuster ${CERES_LIBRARY} ${CERES_LIBRARY_DEPENDENCIES})

# ADD_EXECUTABLE(quadratic
#                quadratic.cc)
# TARGET_LINK_LIBRARIES(quadratic ${CERES_LIBRARY} ${CERES_LIBRARY_DEPENDENCIES})

# ADD_TORCH_PACKAGE(solve "${src}" "" "Floored Solver")

# target_link_libraries(solve luaT TH ${CERES_LIBRARY})
