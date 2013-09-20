include ../Makefile.common

ver=6510db9d452b26c8c87e69bf205c82a3be072921

git_src=git@github.com:OctoMap/octomap.git

git_clone=$(build_dir)/deps/octomap

install=$(install_root)/lib/liboctomap.dylib

viewer_bin=$(install_root)/bin/octovis

args = $(default_cmake_args) \
       -D CMAKE_SHARED_LINKER_FLAGS="-undefined dynamic_lookup"

all: $(install)

$(install): $(git_clone)
	cd $(git_clone)/octomap; mkdir -p build
	cd $(git_clone)/octomap/build; cmake $(args) ..
	cd $(git_clone)/octomap/build; make -j4 install

viewer: $(viewer_bin)

$(viewer_bin): $(git_clone)
	cd $(git_clone)/octovis; mkdir -p build
	cd $(git_clone)/octovis/src/extern/QGLViewer; qmake ; make install
	cd $(git_clone)/octovis/build; cmake $(args) ..
	cd $(git_clone)/octovis/build; make -j4 install

$(git_clone):
	git clone $(git_src) $(git_clone)
	cd $(git_clone); git checkout $(ver)

update:
	cd $(git_clone); git pull; git checkout $(ver)

uninstall: 
	-rm -rf $(install_root)/include/octomap/
	-rm -rf $(install_root)/share/octomap/package.xml
	-rm -rf $(install_root)/lib/pkgconfig/octomap.pc
	-rm -rf $(install_root)/lib/liboctomath.dylib
	-rm -rf $(install_root)/lib/liboctomath.a
	-rm -rf $(install_root)/lib/liboctomap.dylib
	-rm -rf $(install_root)/lib/liboctomap.a
	-rm -rf $(install_root)/bin/graph2tree
	-rm -rf $(install_root)/bin/log2graph
	-rm -rf $(install_root)/bin/binvox2bt
	-rm -rf $(install_root)/bin/bt2vrml
	-rm -rf $(install_root)/bin/edit_octree
	-rm -rf $(install_root)/bin/convert_octree
	-rm -rf $(install_root)/bin/eval_octree_accuracy
	-rm -rf $(install_root)/bin/compare_octrees

clean: 
	-rm -rf $(git_clone)/octomap/build

purge: uninstall
	-rm -rf $(git_clone)

rebuild: uninstall update $(install)

.PHONY: all update uninstall clean purge
