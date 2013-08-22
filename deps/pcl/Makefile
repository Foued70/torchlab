include ../Makefile.common

tag=1.7.0

git_src=git@github.com:PointCloudLibrary/pcl.git

# url=http://downloads.sourceforge.net/project/pointclouds/$(ver)/PCL-$(ver)-Source.tar.bz2
git_clone=$(build_dir)/deps/pcl

install=$(install_root)/lib/libpcl_common.dylib

args = $(default_cmake_args) \
	-D CMAKE_CXX_FLAGS:STRING=-Wno-invalid-offsetof \


all: $(install)


$(install): $(git_clone)
	cd $(git_clone)/cmake; sed -i .bak 's/PROPERTIES LINK_FLAGS -Wl)/PROPERTIES LINK_FLAGS "")/g' pcl_targets.cmake 
	cd $(git_clone); mkdir -p build
	cd $(git_clone)/build; cmake $(args) ..
	cd $(git_clone)/build; make -j4 install

$(git_clone):
	git clone $(git_source) $(git_clone)
	cd $(git_clone); git checkout tags/$(tag)

update:
	cd $(git_clone); git pull; git checkout tags/$(tag)
	rm $(install)
