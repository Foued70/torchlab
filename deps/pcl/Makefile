include ../Makefile.common

ver=1.6.0

url=http://www.pointclouds.org/assets/files/$(ver)/PCL-$(ver)-Source.tar.bz2

tar_file=$(build_dir)/deps/PCL-$(ver)-Source.tar.bz2
unpack=$(build_dir)/deps/PCL-$(ver)-Source

install=$(install_root)/lib/libpcl_common.dylib

args = $(default_cmake_args) \
	-D CMAKE_CXX_FLAGS:STRING=-Wno-invalid-offsetof \


all: $(install)
	
	
$(install): $(unpack)
	cd $(unpack)/cmake; sed -i .bak 's/PROPERTIES LINK_FLAGS -Wl)/PROPERTIES LINK_FLAGS "")/g' pcl_targets.cmake 
	cd $(unpack); mkdir -p build
	cd $(unpack)/build; cmake $(args) ..
	cd $(unpack)/build; make -j4 install

$(unpack): $(tar_file)
	cd $(build_dir)/deps; tar -xzf $(tar_file)

$(tar_file):
	curl -L -o $(tar_file) $(url)
