include ../Makefile.common

ver=8e9ea60ac091af4107b2ae18ccdd8d12413945f8
url=https://github.com/mkottman/lqt.git

git_clone=$(build_dir)/deps/lqt

install=$(install_root)/xxx

args = $(default_cmake_args) \
	-D QT_QMAKE_EXECUTABLE=$(install_root)/bin/qmake \
	-D LUA_INCLUDE_DIR=$(install_root)/include/torch \
	-D LUA_LIBRARIES=$(install_root)/lib/libtorch-lua.dylib \



all:$(install)


$(install): $(git_clone)
	cd $(git_clone); git checkout $(ver)
	cd $(git_clone); rm BUILD; mkdir -p build
	cd $(git_clone)/build; cmake $(args) ..
	cd $(git_clone)/build; make
	cp $(git_clone)/build/lib/* $(install_root)/lib/torch/lua/


$(git_clone):
	git clone $(url) $(git_clone)

tmp:

