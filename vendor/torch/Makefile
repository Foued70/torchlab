include ../Makefile.common

ver=fb7ef6ae69932f84502343d2b443e6e96aa34669

git_clone=$(build_dir)/vendor/torch

install=$(install_root)/bin/torch

args = $(default_cmake_args) \
	-D WITH_LUA_JIT=1



all:$(install)


$(install): $(git_clone)
	cd $(git_clone); git checkout $(ver)
	cd $(git_clone); mkdir -p build
	cd $(git_clone)/build; cmake $(args) ..
	cd $(git_clone)/build; make install


$(git_clone):
	git clone https://github.com/andresy/torch $(git_clone)

