include ../Makefile.common

tag=3.0.2

git_src=git@github.com:glfw/glfw.git
git_clone=$(build_dir)/deps/glfw

install=$(install_root)/lib/libglfw.dylib

args =  -D CMAKE_INSTALL_PREFIX:PATH=$(install_root) \
	-D CMAKE_INSTALL_NAME_DIR:FILEPATH=$(install_root)/lib \
	-D GLFW_BUILD_EXAMPLES=0 \
	-D BUILD_SHARED_LIBS=1

all:$(install)

$(install): $(git_clone)
	cd $(git_clone); mkdir -p build
	cd $(git_clone)/build; cmake $(args) ..
	cd $(git_clone)/build; make -j4 install

$(git_clone):
	git clone $(git_src) $(git_clone)
	cd $(git_clone); git checkout tags/$(tag)

uninstall: 
	-rm $(install)
	-rm $(install_root)/lib/libglfw*
	-rm -rf $(install_root)/lib/cmake/glfw/
	-rm $(install_root)/lib/pkgconfig/glfw*
	-rm -rf $(install_root)/include/GLFW

clean: uninstall
	-rm -rf $(git_clone)/build

rebuild: uninstall $(install)

.PHONY: uninstall clean rebuild

# replace strings to greate glfw.lua from glfw3.headers
# "/\*((?!\*/).|\n)+\*/" >>>  ""
# "#define GLFW_(\w+)(\s+)"  >>>  "glfw.\1\2= "
# "GLFW_"  >>>  "glfw."
# "GLFWAPI "  >>>  ""
