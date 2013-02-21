include ../Makefile.common

ver=1.14

libname = popt
dirname = $(libname)-$(ver)
tarname = $(dirname).tar.gz
# can't find where this lib is officially hosted, should be rolled into libgphoto
url     = http://rpm5.org/files/$(libname)/$(tarname)

tar_file= $(build_dir)/deps/$(tarname)
unpack  = $(build_dir)/deps/$(dirname)

install = $(install_root)/lib/libpopt.a

args    = --prefix=$(install_root) \


all:$(install)


$(install): $(unpack)
	cd $(unpack); ./configure $(args)
	cd $(unpack); make install

$(unpack): $(tar_file)
	cd $(build_dir)/deps; tar -xzf $(tar_file)

$(tar_file):
	curl -L -o $(tar_file) $(url)


