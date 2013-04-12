include ../Makefile.common

git_clone=$(build_dir)/deps/libfreenect


install=$(install_root)/lib/libfreenect-xxx.dylib

args = $(default_cmake_args) 

all: $(install)

$(install): $(git_clone)
	cd $(git_clone); mkdir -p build
	cd $(git_clone)/build; cmake $(args) ..
	cd $(git_clone)/build; make -j8 install
	$(CURDIR)/fix_dylib_paths.sh ${install_root}

$(git_clone):
	git clone https://github.com/OpenKinect/libfreenect.git $(git_clone)
