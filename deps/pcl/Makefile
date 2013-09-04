include ../Makefile.common

tag=1.7.0

git_src=git@github.com:PointCloudLibrary/pcl.git

# url=http://downloads.sourceforge.net/project/pointclouds/$(ver)/PCL-$(ver)-Source.tar.bz2
git_clone=$(build_dir)/deps/pcl

install=$(install_root)/lib/libpcl_common.dylib


args = $(default_cmake_args)

all: $(install)


$(install): $(git_clone)
	cd $(git_clone); mkdir -p build
	cd $(git_clone)/build; cmake $(args) ..
	cd $(git_clone)/build; make -j4 install

$(git_clone):
	git clone $(git_src) $(git_clone)
	cd $(git_clone); git checkout tags/pcl-$(tag)

update:
	cd $(git_clone); git pull; git checkout tags/pcl-$(tag)

uninstall: 
	-rm -rf $(install_root)/include/pcl-1.7/
	-rm -rf $(install_root)/lib/libpcl*
	-rm -rf $(install_root)/lib/pkgconfig/pcl*
	-rm -rf $(install_root)/bin/pcl_*

clean: 
	-rm -rf $(git_clone)/build

purge: 
	-rm -rf $(git_clone)

rebuild: uninstall update $(install)

.PHONY: all update uninstall clean purge
