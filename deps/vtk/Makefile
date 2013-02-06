include ../Makefile.common

ver=5.10.1

ver_2=$(basename $(ver))
url=http://www.vtk.org/files/release/$(ver_2)/vtk-$(ver).tar.gz

tar_file=$(build_dir)/deps/vtk-$(ver).tar.gz
unpack=$(build_dir)/deps/VTK$(ver)

install=$(install_root)/lib/vtk-$(ver_2)

args = \
	-D CMAKE_INSTALL_PREFIX:PATH=$(install_root) \
	-D CMAKE_BUILD_TYPE=Release \
	-D VTK_REQUIRED_OBJCXX_FLAGS:STRING='' \
	-D VTK_USE_CARBON:BOOL=OFF \
	-D BUILD_TESTING:BOOL=OFF \
	-D BUILD_EXAMPLES:BOOL=OFF \
	-D BUILD_SHARED_LIBS:BOOL=ON \
	-D VTK_USE_GUISUPPORT:BOOL=ON \
	-D VTK_USE_QT:BOOL=ON \
	-D VTK_USE_QVTK:BOOL=ON



all: $(install)
	
	
$(install): $(unpack)
	cd $(unpack); mkdir -p build
	cd $(unpack)/build; cmake $(args) ..
	cd $(unpack)/build; make install

$(unpack): $(tar_file)
	cd $(build_dir)/deps; tar -xzf $(tar_file)

$(tar_file):
	curl -L -o $(tar_file) $(url)
