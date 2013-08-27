include ../Makefile.common

ver=1.3.18

url=http://downloads.sourceforge.net/project/graphicsmagick/graphicsmagick/$(ver)/GraphicsMagick-$(ver).tar.bz2

tar_file=$(build_dir)/deps/GraphicsMagick-$(ver).tar.bz2
unpack=$(build_dir)/deps/GraphicsMagick-$(ver)

install=$(install_root)/bin/gm

# LDFLAGS='-undefined dynamic_lookup' caused other errors but making
# sure that the correct libstdc++ is in the linking path fixes
# errors. hopefully libstdc++.a is not OSX or unix specific
libstdcpp=libstdc++.a
libstdcppdir:=$(shell $(GCC) -print-file-name=$(libstdcpp) | sed 's!\(.*\)'$(libstdcpp)'!\1!')

args = 	$(default_args) \
	CFLAGS=-I$(install_root)/include \
	CXXFLAGS=-I$(install_root)/include \
	LDFLAGS="-L$(libstdcppdir) -L$(install_root)/lib" \
	--prefix=$(install_root) \
	--disable-dependency-tracking \
	--enable-shared \
	--disable-static \
	--with-quantum-depth=16 \
	--without-x

all: $(install)


$(install): $(unpack)
	cd $(unpack); ./configure $(args)
	cd $(unpack); make install

$(unpack): $(tar_file)
	cd $(build_dir)/deps; tar -xjf $(tar_file)

$(tar_file):
	curl -L -o $(tar_file) $(url)

uninstall:
	-rm $(install)
	-rm -rf $(install_root)/bin/GraphicsMagick*
	-rm -rf $(install_root)/include/GraphicsMagick
	-rm -rf $(install_root)/lib/libGraphicsMagick*
	-rm -rf $(install_root)/lib/libGraphicsMagick*
	-rm -rf $(install_root)/lib/pkgconfig/GraphicsMagick*pc

clean:
	-rm -rf $(unpack)
	-rm $(tar_file)

rebuild: uninstall $(install)

.PHONY: clean uninstall rebuild
