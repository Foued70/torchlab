include ../Makefile.common

build_dir = $(install_root)/build/cloudlab

all:
	mkdir -p $(build_dir)
	cd $(build_dir); CMAKE_PREFIX_PATH=$(install_root) cmake $(CURDIR)
	cd $(build_dir); make install
