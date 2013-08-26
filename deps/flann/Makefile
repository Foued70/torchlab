include ../Makefile.common

ver=1.8.4

url=http://www.cs.ubc.ca/research/flann/uploads/FLANN/flann-$(ver)-src.zip

tar_file=$(build_dir)/deps/flann-$(ver)-src.zip
unpack=$(build_dir)/deps/flann-$(ver)-src

install=$(install_root)/lib/libflann.$(ver).dylib

args = $(default_cmake_args) \
  -D CMAKE_INSTALL_NAME_DIR:FILEPATH=$(install_root)/lib \
  -D BUILD_MATLAB_BINDINGS:BOOL=OFF \
  -D BUILD_PYTHON_BINDINGS:BOOL=OFF \


all:$(install)


$(install): $(unpack)
	cd $(unpack); mkdir -p build
	cd $(unpack)/build; cmake $(args) ..
	cd $(unpack)/build; make -j4 install

$(unpack): $(tar_file)
	cd $(build_dir)/deps; unzip -o $(tar_file)
	touch $(unpack)

$(tar_file):
	curl -L -o $(tar_file) $(url)

uninstall:
	rm $(install)
	rm $(install_root)/include/flanlib/libflann*
	rm $(install_root)/pkgconfig/flann.pc
	rm -rf $(install_root)/include/flann
	rm $(install_root)/bin/flann_*

clean: uninstall
	rm -rf $(unpack)
	rm -rf $(tar_file)
