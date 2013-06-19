
get_filename_component(project_dir ${CMAKE_CURRENT_LIST_DIR} PATH)
set(install_root ${project_dir}/build/usr/local)

if (DEFINED ENV{CLOUDLAB_INSTALL_ROOT})
  get_filename_component(install_root "$ENV{CLOUDLAB_INSTALL_ROOT}" ABSOLUTE)
endif (DEFINED ENV{CLOUDLAB_INSTALL_ROOT})

# set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH}:torch)

set(CMAKE_INSTALL_PREFIX ${install_root} CACHE PATH "" FORCE)
set(CMAKE_CXX_COMPILER ${install_root}/bin/g++ CACHE FILEPATH "" FORCE)
set(CMAKE_CXX_FLAGS "-std=gnu++11" CACHE STRING "" FORCE)

set(Torch_SOURCE_INCLUDES ${install_root}/include/torch ${install_root}/include/torch/TH )

MACRO(ADD_LUVIT_LIB package src)
  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${Torch_SOURCE_INCLUDES})

  ADD_LIBRARY(${package} SHARED ${src})
  
  SET_TARGET_PROPERTIES(${package} PROPERTIES
    PREFIX "lib"
    IMPORT_PREFIX "lib"
    SUFFIX ".luvit"
    INSTALL_NAME_DIR "@executable_path/../lib/luvit")
  
  INSTALL(TARGETS ${package} 
      RUNTIME DESTINATION "lib/luvit"
      LIBRARY DESTINATION "lib/luvit")
  
ENDMACRO(ADD_LUVIT_LIB)

MACRO(ADD_FFI_LIB package src)
  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${Torch_SOURCE_INCLUDES})

  ADD_LIBRARY(${package} SHARED ${src})
  
  SET_TARGET_PROPERTIES(${package} PROPERTIES
    PREFIX "lib"
    IMPORT_PREFIX "lib"
    INSTALL_NAME_DIR "@executable_path/../lib/luvit")
  
  INSTALL(TARGETS ${package} 
      RUNTIME DESTINATION "lib"
      LIBRARY DESTINATION "lib")
  
ENDMACRO(ADD_FFI_LIB)
