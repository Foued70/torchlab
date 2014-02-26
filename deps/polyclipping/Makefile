include ../Makefile.common

svn_src=svn://svn.code.sf.net/p/polyclipping/code/trunk/cpp

svn_checkout=$(build_dir)/deps/clipper_cpp

install=$(install_root)/lib/libpolyclipping.dylib

args = $(default_cmake_args)

all: $(install)

$(install): $(svn_checkout)
	cd $(svn_checkout); mkdir -p build
	cd $(svn_checkout)/build; cmake $(args) ..
	cd $(svn_checkout)/build; make -j4 install

$(svn_checkout):
	svn checkout $(svn_src) $(svn_checkout)

update:
	cd $(git_clone); git pull; git checkout tags/pcl-$(tag)

uninstall: 
	-rm -rf $(install_root)/include/pcl-1.7/
	-rm -rf $(install_root)/lib/libpcl*
	-rm -rf $(install_root)/lib/pkgconfig/pcl*
	-rm -rf $(install_root)/bin/pcl_*

clean: 
	-rm -rf $(svn_checkout)/build

purge: 
	-rm -rf $(git_clone)

rebuild: uninstall update $(install)

.PHONY: all update uninstall clean purge
