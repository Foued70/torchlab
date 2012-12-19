build_dir = $(abspath build)
install_root = $(build_dir)/usr/local


packages = $(notdir $(wildcard src/*))
packages_install = $(addprefix $(install_root)/share/torch/lua/,$(packages))

all: $(packages_install)

$(packages_install):
	cd src/$(notdir $@); $(install_root)/bin/torch-pkg deploy
	
deps: build
	cd vendor/gcc; make
	cd vendor/boost; make
	cd vendor/eigen; make
	cd vendor/flann; make
	cd vendor/qt; make
	cd vendor/vtk; make
	cd vendor/pcl; make

build: 
	mkdir -p build