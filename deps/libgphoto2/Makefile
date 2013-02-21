include ../Makefile.common

ver     = 2.5.1

dirname = libgphoto2-$(ver)
tarname = $(dirname).tar.bz2
url     = http://downloads.sourceforge.net/project/gphoto/libgphoto/$(ver)/$(tarname)

tar_file= $(build_dir)/deps/$(tarname)
unpack  = $(build_dir)/deps/$(dirname)

install = $(install_root)/lib/libgphoto2.dylib

args    = --prefix=$(install_root) \
	  --without-serial 

# from http://sourceforge.net/p/gphoto/bugs/935/
extra_flags = CPP_FLAGS=-D_DARWIN_C_SOURCE CFLAGS=-D_DARWIN_C_SOURCE

all:$(install)


$(install): $(unpack)
	cd $(unpack); $(pkg_config_path) $(extra_flags) ./configure $(args)
	cd $(unpack); make install

$(unpack): $(tar_file)
	cd $(build_dir)/deps; tar -xjf $(tar_file)

$(tar_file):
	curl -L -o $(tar_file) $(url)


