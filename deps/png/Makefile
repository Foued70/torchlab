include ../Makefile.common

ver=1.6.8

url=http://downloads.sourceforge.net/project/libpng/libpng16/$(ver)/libpng-$(ver).tar.xz

tar_file=$(build_dir)/deps/libpng-$(ver).tar.gz
unpack=$(build_dir)/deps/libpng-$(ver)

install=$(install_root)/lib/libpng.a

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
	-rm $(install_root)/lib/libpng*
	-rm $(install_root)/bin/pngfix
	-rm $(install_root)/bin/png-fix-itxt
	-rm $(install_root)/bin/libpng-config
	-rm $(install_root)/bin/libpng16-config

rebuild: uninstall $(install)

clean:
	-rm -rf $(unpack)
	-rm $(tar_file)

.PHONY: clean rebuild uninstall
