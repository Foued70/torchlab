include ../Makefile.common

ver=3.0.1

url=http://downloads.sourceforge.net/project/glfw/glfw/$(ver)/glfw-$(ver).tar.gz

tar_file=$(build_dir)/deps/glfw-$(ver).tar.gz
unpack=$(build_dir)/deps/glfw-$(ver)

install=$(install_root)/lib/libglfw.dylib

args = -D CMAKE_INSTALL_PREFIX:PATH=$(install_root) \
	-D GLFW_BUILD_EXAMPLES=0 \
	-D BUILD_SHARED_LIBS=1



all:$(install)


$(install): $(unpack)
	cd $(unpack); mkdir -p build
	cd $(unpack)/build; cmake $(args) ..
	cd $(unpack)/build; make install

$(unpack): $(tar_file)
	cd $(build_dir)/deps; tar -xzf $(tar_file)

$(tar_file):
	curl -L -o $(tar_file) $(url)


# replace strings to greate glfw.lua from glfw3.headers
# "/\*((?!\*/).|\n)+\*/" >>>  ""
# "#define GLFW_(\w+)(\s+)"  >>>  "glfw.\1\2= "
# "GLFW_"  >>>  "glfw."
# "GLFWAPI "  >>>  ""
