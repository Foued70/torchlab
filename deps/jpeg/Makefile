include ../Makefile.common

ver=8d

url=http://www.ijg.org/files/jpegsrc.v$(ver).tar.gz

tar_file=$(build_dir)/deps/jpegsrc.v$(ver).tar.gz
unpack=$(build_dir)/deps/jpeg-$(ver)

install=$(install_root)/lib/libjpeg.a

args = \
	--prefix=$(install_root) \
	--disable-dependency-tracking


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
	-rm $(install_root)/bin/jpegtran
	-rm $(install_root)/bin/djpeg
	-rm $(install_root)/bin/cjpeg
	-rm $(install_root)/bin/wrjpgcom
	-rm $(install_root)/bin/rdjpgcom

clean: uninstall
	-rm -rf $(unpack)
	-rm $(tar_file)

.PHONY: uninstall clean
