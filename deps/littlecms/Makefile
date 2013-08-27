include ../Makefile.common

ver=1.19

url=http://sourceforge.net/projects/lcms/files/lcms/1.19/lcms-$(ver).tar.gz

tar_file=$(build_dir)/deps/lcms-$(ver).tar.gz
unpack=$(build_dir)/deps/lcms-$(ver)

install=$(install_root)/lib/liblcms.a

args = $(default_args) \
	--prefix=$(install_root) \
	--enable-shared \
	--disable-debug \

all:$(install)


$(install): $(unpack)
	cd $(unpack); ./configure $(args)
	cd $(unpack); make install

$(unpack): $(tar_file)
	cd $(build_dir)/deps; tar -xzf $(tar_file)

$(tar_file):
	curl -L -o $(tar_file) $(url)

uninstall:
	-rm $(install)
	-rm $(install_root)/bin/jpegicc
	-rm $(install_root)/bin/tifficc
	-rm $(install_root)/bin/tiffdiff
	-rm $(install_root)/bin/wtpt
	-rm $(install_root)/bin/icctrans
	-rm $(install_root)/bin/icclink
	-rm $(install_root)/include/lcms.h
	-rm $(install_root)/include/icc34.h
	-rm $(install_root)/lib/liblcms.*
	-rm $(install_root)/lib/pkgconfig/lcms.pc

clean:
	-rm -rf $(unpack)
	-rm $(tar_file)

rebuild: uninstall $(install)

.PHONY: clean uninstall rebuild
