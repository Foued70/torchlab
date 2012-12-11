include ../Makefile.common

ver=3.1.2

url=http://bitbucket.org/eigen/eigen/get/$(ver).tar.bz2

tar_file=$(build_dir)/vendor/eigen-$(ver).tar.bz2
unpack=$(build_dir)/vendor/eigen-$(ver)

install=$(install_root)/include/eigen3

args = $(default_cmake_args)


all:$(install)
	
	
$(install): $(unpack)
	cd $(unpack); mkdir -p build
	cd $(unpack)/build; cmake $(args) ..
	cd $(unpack)/build; make install

$(unpack): $(tar_file)
	cd $(build_dir)/vendor; tar -xjf $(tar_file)
	mv $(build_dir)/vendor/eigen-eigen-* $(unpack)

$(tar_file):
	curl -L -o $(tar_file) $(url)
