include ../Makefile.common

ver=1.900.1

url=http://download.osgeo.org/gdal/jasper-$(ver).uuid.tar.gz

tar_file=$(build_dir)/deps/jasper-$(ver).uuid.tar.gz
unpack=$(build_dir)/deps/jasper-$(ver).uuid

install=$(install_root)/lib/libjasper.a

args = $(default_args) \
  CFLAGS="-I$(install_root)/include" \
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

$(tar_file):
	curl -L -o $(tar_file) $(url)

uninstall:
	-rm $(install)
	-rm $(install_root)/bin/imginfo
	-rm $(install_root)/bin/imgcmp
	-rm $(install_root)/bin/jasper
	-rm $(install_root)/bin/tmrdemo
	-rm -rf $(install_root)/include/jasper
	-rm $(install_root)/lib/libjasper*

clean:
	-rm -rf $(unpack)
	-rm $(tar_file)

rebuild: uninstall $(install)

.PHONY: clean uninstall rebuild
