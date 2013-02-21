include ../Makefile.common

ver     = 1.0.9

libname = libusb
dirname = $(libname)-$(ver)
tarname = $(dirname).tar.bz2
url     = http://downloads.sourceforge.net/project/$(libname)/$(libname)-1.0/$(dirname)/$(tarname)

tar_file= $(build_dir)/deps/$(tarname)
unpack  = $(build_dir)/deps/$(dirname)

install = $(install_root)/lib/libusb-1.0.a

args    = --prefix=$(install_root)


all:$(install)


$(install): $(unpack)
	cd $(unpack); ./configure $(args)
	cd $(unpack); make install

$(unpack): $(tar_file)
	cd $(build_dir)/deps; tar -xjf $(tar_file)

$(tar_file):
	curl -L -o $(tar_file) $(url)


