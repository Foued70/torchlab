include ../Makefile.common

ver=f4731c87da806e68e5fc3385f2947ed0be427857

git_clone=$(build_dir)/deps/torch
# git_clone=/Users/dustin/proj/torch

install=$(install_root)/bin/torch

packages = \
	image \
	xlua 

pkgs = $(addprefix $(install_root)/share/torch/lua/,$(packages))


args = $(default_cmake_args) \
	-D QT_QMAKE_EXECUTABLE=$(install_root)/bin/qmake

all:$(pkgs)

$(pkgs): $(install)
	$(install_root)/bin/torch-pkg install $(notdir $@)


$(install): $(git_clone)
	cd $(git_clone); mkdir -p build
	cd $(git_clone)/build; cmake $(args) ..
	cd $(git_clone)/build; make install


$(git_clone):
	git clone https://github.com/floored/torch $(git_clone)
	cd $(git_clone); git checkout $(ver)

clean:
	rm -f $(install)*
	rm -rf $(git_clone)
	rm -rf $(install_root)/share/torch/lua/torch
