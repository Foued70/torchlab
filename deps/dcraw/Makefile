include ../Makefile.common

ver=9.17

url=http://www.cybercom.net/~dcoffin/dcraw/archive/dcraw-$(ver).tar.gz

tar_file=$(build_dir)/deps/dcraw-$(ver).tar.gz
unpack=$(build_dir)/deps/dcraw

install=$(install_root)/bin/dcraw



all:$(install)


$(install): $(unpack)
	cd $(unpack); $(install_root)/bin/gcc -o $(install_root)/bin/dcraw -O4 dcraw.c \
	                   -Wall -Wno-unused-result -Wno-array-bounds \
	                   -lm -ljasper -ljpeg -llcms -DNODEPS

$(unpack): $(tar_file)
	cd $(build_dir)/deps; tar -xzf $(tar_file)

$(tar_file):
	curl -L -o $(tar_file) $(url)
