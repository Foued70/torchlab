include ../Makefile.common

ver=3.2.0

url=http://bitbucket.org/eigen/eigen/get/$(ver).tar.bz2

tar_file=$(build_dir)/deps/eigen-$(ver).tar.bz2
unpack=$(build_dir)/deps/eigen-$(ver)

install=$(install_root)/include/eigen3/signature_of_eigen3_matrix_library

args=$(default_cmake_args)

all: $(install)

$(install): $(unpack)
	echo $(install)
	cd $(unpack); mkdir -p build
	cd $(unpack)/build; cmake $(args) ..
	cd $(unpack)/build; make install
	touch $(install)

$(unpack): $(tar_file)
	cd $(build_dir)/deps; tar -xjf $(tar_file)
	mv $(build_dir)/deps/eigen-eigen-* $(unpack)

$(tar_file):
	curl -L -o $(tar_file) $(url)

uninstall:
	rm -rf $(install)
	rm -f $(install_root)/lib/pkgconfig/eigen3.pc

clean: uninstall
	rm -rf $(tar_file)
	rm -rf $(unpack)
