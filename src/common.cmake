find_package(Torch REQUIRED)

set(project_dir ${CMAKE_CURRENT_LIST_DIR})
set(install_root ${project_dir}/build/usr/local)

if (DEFINED ENV{CLOUDLAB_INSTALL_ROOT})
  get_filename_component(install_root "$ENV{CLOUDLAB_INSTALL_ROOT}" ABSOLUTE)
  message(${install_root})
endif (DEFINED ENV{CLOUDLAB_INSTALL_ROOT})


set(CMAKE_CXX_COMPILER ${install_root}/bin/g++)
set(CMAKE_CXX_FLAGS "-std=gnu++11")

