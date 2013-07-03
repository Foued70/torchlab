include ../Makefile.common

ver=1.53.0
ver_ubar=$(subst .,_,$(ver))

#url=http://voxel.dl.sourceforge.net/project/boost/boost/$(ver)/boost_$(ver_ubar).tar.bz2
url=http://downloads.sourceforge.net/project/boost/boost/$(ver)/boost_$(ver_ubar).tar.bz2

tar_file=$(build_dir)/deps/boost_$(ver_ubar).tar.bz2
unpack=$(build_dir)/deps/boost_$(ver_ubar)

install=$(install_root)/lib/libboost_system.a

all: $(install)


$(install): $(unpack)
	cd $(unpack); ./bootstrap.sh --prefix=$(install_root)
	sed -i .bak 's^using darwin ;^using darwin : : $(CXX) ;^g' $(unpack)/project-config.jam
	cd $(unpack); ./b2 install

$(unpack): $(tar_file)
	cd $(build_dir)/deps; tar -xjf $(tar_file)

$(tar_file):
	curl -L -o $(tar_file) $(url)
