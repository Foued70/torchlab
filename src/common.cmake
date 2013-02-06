find_package(Torch REQUIRED)

get_filename_component(project_dir ${CMAKE_CURRENT_LIST_DIR} PATH)
set(install_root ${project_dir}/build/usr/local)

if (DEFINED ENV{CLOUDLAB_INSTALL_ROOT})
  get_filename_component(install_root "$ENV{CLOUDLAB_INSTALL_ROOT}" ABSOLUTE)
endif (DEFINED ENV{CLOUDLAB_INSTALL_ROOT})


set(CMAKE_INSTALL_PREFIX ${install_root})
set(CMAKE_CXX_COMPILER ${install_root}/bin/g++)
set(CMAKE_CXX_FLAGS "-std=gnu++11")

