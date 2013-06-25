include ../Makefile.common

local_source=/Users/dustin/proj/torch

ver=f4731c87da806e68e5fc3385f2947ed0be427857

git_source=https://github.com/floored/torch


ifdef local_source
	git_clone=$(local_source)
else
	git_clone=$(build_dir)/deps/torch
endif	

# env MACOSX_DEPLOYMENT_TARGET=10.3 gcc -bundle -undefined dynamic_lookup -o random.so lrandom.o

install=$(install_root)/bin/torch


args = $(default_cmake_args) \
       -D CMAKE_SHARED_LINKER_FLAGS="-undefined dynamic_lookup"\
       -D LUA_INCLUDE_DIR=$(install_root)/include/luvit/luajit \



all:$(install)


$(install): $(git_clone)
	cd $(git_clone); mkdir -p build
	cd $(git_clone)/build; cmake $(args) ..
	cd $(git_clone)/build; make install VERBOSE=1
	rm -rf $(install_root)/lib/luvit/torch
	mv $(install_root)/share/torch/lua/torch $(install_root)/lib/luvit/
	mkdir -p $(install_root)/lib/luvit/torch/modules
	mv $(install_root)/lib/torch/lua/libtorch.* $(install_root)/lib/luvit/torch/modules/libtorch.luvit
	rm -rf $(install_root)/lib/torch
	cd $(install_root)/lib; $(CURDIR)/../gcc/fix_paths.sh


$(git_clone):
	git clone $(git_source) $(git_clone)
	cd $(git_clone); git checkout $(ver)

clean:
	rm -f $(install)*
	rm -rf $(git_clone)/build
	rm -rf $(install_root)/lib/torch
	rm -rf $(install_root)/lib/libTH.*
	rm -rf $(install_root)/lib/libluaT.*
	rm -rf $(install_root)/lib/libtorch-lua.*
	rm -rf $(install_root)/include/torch
	rm -rf $(install_root)/share/torch
	rm -rf $(install_root)/share/cmake/torch

	rm -rf $(install_root)/lib/luvit/compat
	rm -rf $(install_root)/lib/luvit/dok
	rm -rf $(install_root)/lib/luvit/gnuplot
	rm -rf $(install_root)/lib/luvit/nn
	rm -rf $(install_root)/lib/luvit/paths
	rm -rf $(install_root)/lib/luvit/sys
	rm -rf $(install_root)/lib/luvit/torch
	rm -rf $(install_root)/lib/luvit/torch-env.lua
	rm -rf $(install_root)/lib/luvit/wrap
	rm -rf $(install_root)/lib/luvit/xlua

	rm -rf $(install_root)/lib/luvit/libnn.luvit
	rm -rf $(install_root)/lib/luvit/libpaths.luvit
	rm -rf $(install_root)/lib/luvit/libsys.luvit
	rm -rf $(install_root)/lib/luvit/libtorch.luvit
