include ../Makefile.common

ver=2.4.4

url=http://downloads.sourceforge.net/project/opencvlibrary/opencv-unix/$(ver)/OpenCV-$(ver)-beta.tar.bz2

tar_file=$(build_dir)/deps/OpenCV-$(ver)-beta.tar.bz2
unpack=$(build_dir)/deps/OpenCV-$(ver)

install=$(install_root)/lib/libopencv_core.$(ver).dylib

args = $(default_cmake_args) \
	-DBUILD_opencv_highgui:BOOL=OFF \
	-DBUILD_opencv_video:BOOL=OFF \
	-DBUILD_opencv_java:BOOL=OFF

all: $(install)

$(install): $(unpack)
	cd $(unpack); mkdir -p build
	cd $(unpack)/build; cmake $(args) ..
	cd $(unpack)/build; make -j4 install

$(unpack): $(tar_file)
	cd $(build_dir)/deps; tar -xjf $(tar_file)

$(tar_file):
	curl -L -o $(tar_file) $(url)
