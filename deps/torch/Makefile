include ../Makefile.common

# local_source=/Users/dustin/proj/torch

ver=luvit

git_source=https://github.com/floored/torch


ifdef local_source
	git_clone=$(local_source)
else
	git_clone=$(build_dir)/deps/torch
endif	

# env MACOSX_DEPLOYMENT_TARGET=10.3 gcc -bundle -undefined dynamic_lookup -o random.so lrandom.o

install=$(install_root)/lib/luvit/torch/modules/libtorch.luvit

args = $(default_cmake_args) \
       -D CMAKE_SHARED_LINKER_FLAGS="-undefined dynamic_lookup"\
       -D LUA_INCLUDE_DIR=$(install_root)/include/luvit/luajit \

cmake=$(git_clone)/build/CMakeCache.txt

torch_libdir=$(install_root)/lib/torch/lua/
luvit_dir=$(install_root)/lib/luvit/

all:$(install)

#TODO make a loop for the libs to copy
$(install): $(cmake)
	cd $(git_clone)/build; make install VERBOSE=1
	cp -rf $(install_root)/share/torch/lua/* $(luvit_dir)
	mkdir -p $(luvit_dir)/torch/modules/
	$(CURDIR)/mv_lib_luvit "$(install_root)" torch
	$(CURDIR)/mv_lib_luvit "$(install_root)" nn
	$(CURDIR)/mv_lib_luvit "$(install_root)" cutorch
	$(CURDIR)/mv_lib_luvit "$(install_root)" cunn


$(cmake): $(git_clone)
	cd $(git_clone); mkdir -p build
	-rm -f $(git_clone)/build/CMakeCache.txt
	cd $(git_clone)/build; cmake $(args) ..


$(git_clone):
	git clone $(git_source) $(git_clone)
	cd $(git_clone); git checkout $(ver)

update:
	cd $(git_clone); git pull; git checkout $(ver)

uninstall:
	-rm -f $(install)*
	-rm -rf $(install_root)/lib/torch
	-rm -rf $(install_root)/lib/libTH.*
	-rm -rf $(install_root)/lib/libluaT.*
	-rm -rf $(install_root)/lib/libtorch-lua.*
	-rm -rf $(install_root)/include/torch
	-rm -rf $(install_root)/share/torch
	-rm -rf $(install_root)/share/cmake/torch

	-rm -rf $(install_root)/lib/luvit/compat
	-rm -rf $(install_root)/lib/luvit/dok
	-rm -rf $(install_root)/lib/luvit/gnuplot
	-rm -rf $(install_root)/lib/luvit/nn
	-rm -rf $(install_root)/lib/luvit/paths
	-rm -rf $(install_root)/lib/luvit/sys
	-rm -rf $(install_root)/lib/luvit/torch
	-rm -rf $(install_root)/lib/luvit/torch-env.lua
	-rm -rf $(install_root)/lib/luvit/wrap
	-rm -rf $(install_root)/lib/luvit/xlua

	-rm -rf $(install_root)/lib/luvit/libnn.luvit
	-rm -rf $(install_root)/lib/luvit/libpaths.luvit
	-rm -rf $(install_root)/lib/luvit/libsys.luvit
	-rm -rf $(install_root)/lib/luvit/libtorch.luvit

clean:
	-rm -rf $(git_clone)/build

rebuild: uninstall update $(install)

.PHONY: clean uninstall rebuild update
