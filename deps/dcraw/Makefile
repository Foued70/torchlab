include ../Makefile.common

ver=9.17

url=http://www.cybercom.net/~dcoffin/dcraw/archive/dcraw-$(ver).tar.gz

tar_file=$(build_dir)/deps/dcraw-$(ver).tar.gz
unpack=$(build_dir)/deps/dcraw

install=$(install_root)/bin/dcraw



all:$(install)


$(install): $(unpack)
	cd $(unpack); $(CC) -o $(install_root)/bin/dcraw \
			   -I$(install_root)/include -L$(install_root)/lib dcraw.c\
	                   -Wall -Wno-unused-result -Wno-unused-but-set-variable -Wno-array-bounds \
	                   -lm -ljpeg -llcms -ljasper

$(unpack): $(tar_file)
	cd $(build_dir)/deps; tar -xzf $(tar_file)

$(tar_file):
	curl -L -o $(tar_file) $(url)

clean:
	rm -r $(install)

deepclean: clean
	rm -rf $(unpack) $(tar_file) 
