include ../Makefile.common

ver=eff29a50c0d9e6a15145fa164a3a9ef8d275fd7b

git_clone=$(build_dir)/deps/torch

install=$(install_root)/bin/torch

packages = \
	image \
	xlua \
	torchffi

pkgs = $(addprefix $(install_root)/share/torch/lua/,$(packages))


args = $(default_cmake_args) \
	-D CUDA_NVCC_EXECUTABLE= \
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
	git clone https://github.com/marcoscoffier/torch $(git_clone)
#	git clone https://github.com/andresy/torch $(git_clone)

tmp:

