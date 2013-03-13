include ../Makefile.common

ver=0.4

url=http://scanline.ca/exrtools/exrtools-$(ver).tar.gz

tar_file=$(build_dir)/deps/exrtools-$(ver).tar.gz
unpack=$(build_dir)/deps/exrtools-$(ver)

install=$(install_root)/bin/ppmtoexr

args =  \
  CFLAGS="-I$(install_root)/include" \
  PKG_CONFIG_PATH="$(install_root)/lib/pkgconfig" \
	--prefix=$(install_root) \
	--enable-shared \
	--disable-debug \
	--disable-dependency-tracking \



all:$(install)


$(install): $(unpack)
	cd $(unpack); ./configure $(args)
	cd $(unpack); make install

$(unpack): $(tar_file)
	cd $(build_dir)/deps; tar -xzf $(tar_file)
	patch -p1 -d $(unpack) < exrtools.patch

$(tar_file):
	curl -L -o $(tar_file) $(url)
