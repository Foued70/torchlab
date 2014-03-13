get_filename_component(project_dir ${CMAKE_CURRENT_LIST_DIR} PATH)
set(install_root ${project_dir}/build/usr/local)
message(${install_root})

if (DEFINED ENV{TORCHLAB_INSTALL_ROOT})
  get_filename_component(install_root "$ENV{TORCHLAB_INSTALL_ROOT}" ABSOLUTE)
endif (DEFINED ENV{TORCHLAB_INSTALL_ROOT})
message("${CMAKE_PREFIX_PATH}")
set(CMAKE_PREFIX_PATH "${install_root}:${CMAKE_PREFIX_PATH}")
message("${CMAKE_PREFIX_PATH}")

set(CMAKE_INSTALL_PREFIX ${install_root} CACHE PATH "" FORCE)
set(CMAKE_CXX_FLAGS "-std=gnu++11" CACHE STRING "" FORCE)
set(CMAKE_SHARED_LINKER_FLAGS "-undefined dynamic_lookup -L${CMAKE_INSTALL_PREFIX}/lib ${CMAKE_SHARED_LINKER_FLAGS}")

set(Lua_INCLUDES ${install_root}/include/luvit/luajit )
set(Luvit_INCLUDES ${install_root}/include/luvit ${install_root}/include/luvit/uv )
set(Torch_INCLUDES ${install_root}/include ${install_root}/include/torch ${install_root}/include/torch/TH )

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${Lua_INCLUDES} ${Luvit_INCLUDES} ${Torch_INCLUDES})

MACRO(ADD_LUVIT_LIB package src)
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
  ADD_LIBRARY(${package} SHARED ${src})
  
  SET_TARGET_PROPERTIES(${package} PROPERTIES
    PREFIX "lib"
    IMPORT_PREFIX "lib"
    INSTALL_NAME_DIR "@executable_path/../lib")
  
  INSTALL(TARGETS ${package} 
      RUNTIME DESTINATION "lib"
      LIBRARY DESTINATION "lib")
  
ENDMACRO(ADD_FFI_LIB)
