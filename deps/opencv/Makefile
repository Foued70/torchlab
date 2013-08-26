include ../Makefile.common

# Issue with gcc 4.8 and obj-c version 2
# So we use the xcode version of gcc (4.2.1)
# See: http://stackoverflow.com/questions/10553994/opencv-compilation-error-with-gcc-4-3-or-above-in-mac-os-x

# There are other errors when we use clang ...


# Also tried WITH_QT=ON to use qt backend, but this also calls obj-c
# which is interpreted as obj-c++
default_cmake_args = \
	-D CMAKE_C_COMPILER:PATH=/usr/bin/gcc \
	-D CMAKE_CXX_COMPILER:PATH=/usr/bin/g++ \
	-D CMAKE_INSTALL_NAME_DIR:FILEPATH=$(install_root)/lib \
	-D CMAKE_INSTALL_PREFIX:PATH=$(install_root) \
	-D CMAKE_BUILD_TYPE=Release

# make sure we update to versions which are official releases.  They
# are tagged in github.  This is Release 2.4.6
ver=525b6eca2ed6e790e41d204399025e8280b8bb8e
branch=2.4

git_clone=$(build_dir)/deps/opencv-$(branch)


install=$(install_root)/lib/libopencv_core.$(branch).dylib

args = $(default_cmake_args) \
	-DBUILD_opencv_highgui:BOOL=ON \
	-DBUILD_opencv_video:BOOL=ON \
	-DBUILD_opencv_java:BOOL=OFF \
	-DBUILD_opencv_python:BOOL=OFF \
	-DWITH_CUDA=OFF \
	-DENABLE_SSE41:BOOL=ON \
	-DENABLE_SSE42:BOOL=ON \
	-DENABLE_SSSE3:BOOL=ON

all: $(install)

# we override the incorrect setting of INSTALL_NAME_DIR in the opencv
# source code.  There might be some magic in how homebrew handles this.
$(install): $(git_clone)
	cd $(git_clone); mkdir -p build
	cd $(git_clone)/cmake; \
	    sed -i .bak \
	      's!\(INSTALL_NAME_DIR\).*$$!\1 $${CMAKE_INSTALL_PREFIX}/lib!' \
	      OpenCVModule.cmake
	cd $(git_clone)/build; cmake $(args) ..
	cd $(git_clone)/build; make -j8 install

$(git_clone):
	git clone -b $(branch) https://github.com/Itseez/opencv $(git_clone)
	cd $(git_clone); git checkout $(ver)

uninstall: 
	-rm $(install)
	-rm -rf $(install_root)/share/OpenCV
	-rm $(install_root)/bin/opencv_*
	-rm -rf $(install_root)/include/opencv*
	-rm $(install_root)/lib/libopencv_*

clean:
	-rm -rf $(git_clone)/build

purge: 
	-rm -rf $(git_clone)

rebuild: uninstall $(install)

.PHONY: all uninstall clean rebuild purge
