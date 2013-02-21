include ../Makefile.common

ver     = 0.1.4

libname = libusb-compat
dirname = $(libname)-$(ver)
tarname = $(dirname).tar.bz2
url     = http://downloads.sourceforge.net/project/libusb/$(libname)-0.1/$(dirname)/$(tarname)

tar_file= $(build_dir)/deps/$(tarname)
unpack  = $(build_dir)/deps/$(dirname)

install = $(install_root)/lib/libusb.a

args    = --prefix=$(install_root)


all:$(install)


$(install): $(unpack)
	cd $(unpack); $(pkg_config_path) ./configure $(args)
	cd $(unpack); make install

$(unpack): $(tar_file)
	cd $(build_dir)/deps; tar -xjf $(tar_file)

$(tar_file):
	curl -L -o $(tar_file) $(url)


