include ../Makefile.common

ver      = 0.6.21

libname  = libexif
dirname  = $(libname)-$(ver)
tarname  = $(dirname).tar.bz2
url      = http://downloads.sourceforge.net/project/$(libname)/$(libname)/$(ver)/$(tarname)

tar_file = $(build_dir)/deps/$(tarname)
unpack   = $(build_dir)/deps/$(dirname)

libdir   = $(install_root)/lib
install  = $(libdir)/libexif.a

args     = --prefix=$(install_root) \


all:$(install)


$(install): $(unpack)
	cd $(unpack); ./configure $(args)
	cd $(unpack); make install

$(unpack): $(tar_file)
	cd $(build_dir)/deps; tar -xjf $(tar_file)

$(tar_file):
	curl -L -o $(tar_file) $(url)


