include ../Makefile.common

ver=1.7.1

url=http://people.cs.ubc.ca/~mariusm/uploads/FLANN/flann-$(ver)-src.zip

tar_file=$(build_dir)/deps/flann-$(ver)-src.zip
unpack=$(build_dir)/deps/flann-$(ver)-src

install=$(install_root)/lib/libflann.$(ver).dylib

args = $(default_cmake_args) \
  -D BUILD_MATLAB_BINDINGS:BOOL=OFF \
  -D BUILD_PYTHON_BINDINGS:BOOL=OFF \


all:$(install)
	
	
$(install): $(unpack)
	cd $(unpack); mkdir -p build
	cd $(unpack)/build; cmake $(args) ..
	cd $(unpack)/build; make install

$(unpack): $(tar_file)
	cd $(build_dir)/deps; unzip -o $(tar_file)
	touch $(unpack)

$(tar_file):
	curl -L -o $(tar_file) $(url)
