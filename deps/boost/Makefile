include ../Makefile.common

ver=1.54.0
ver_ubar=$(subst .,_,$(ver))

#url=http://voxel.dl.sourceforge.net/project/boost/boost/$(ver)/boost_$(ver_ubar).tar.bz2
url=http://downloads.sourceforge.net/project/boost/boost/$(ver)/boost_$(ver_ubar).tar.bz2

tar_file=$(build_dir)/deps/boost_$(ver_ubar).tar.bz2
unpack=$(build_dir)/deps/boost_$(ver_ubar)

install=$(install_root)/lib/libboost_system.a

all: $(install)

# until boost officially moves to a github + cmake build system we use
# the old bjam systemand run install_name_tool to correct the linking
# paths

$(install): $(unpack)
	cd $(unpack); ./bootstrap.sh --prefix=$(install_root) --without-libraries=python
	sed -i .bak 's^using darwin ;^using darwin : : $(CXX) ;^g' $(unpack)/project-config.jam
	cd $(unpack); ./b2 install
	./fix_dylib_paths.sh $(install_root)

$(unpack): $(tar_file)
	cd $(build_dir)/deps; tar -xjf $(tar_file)

$(tar_file):
	curl -L -o $(tar_file) $(url)

uninstall:
	rm ${install}
	rm -rf $(install_root)/lib/libboost_*

clean: uninstall
	rm -rf $(unpack)
	rm -rf $(tar_file)
