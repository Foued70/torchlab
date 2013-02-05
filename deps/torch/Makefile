include ../Makefile.common

ver=fb7ef6ae69932f84502343d2b443e6e96aa34669

git_clone=$(build_dir)/deps/torch

install=$(install_root)/bin/torch

packages = \
	image \
	torchffi

pkgs = $(addprefix $(install_root)/share/torch/lua/,$(packages))

	
args = $(default_cmake_args) \
	-D WITH_LUA_JIT=1 \
	-D QT_QMAKE_EXECUTABLE=$(install_root)/bin/qmake



all:$(pkgs)

$(pkgs): $(install)
	$(install_root)/bin/torch-pkg install $(notdir $@)
	

$(install): $(git_clone)
	cd $(git_clone); git checkout $(ver)
	cd $(git_clone); mkdir -p build
	cd $(git_clone)/build; cmake $(args) ..
	cd $(git_clone)/build; make install


$(git_clone):
	git clone https://github.com/andresy/torch $(git_clone)

tmp:
	