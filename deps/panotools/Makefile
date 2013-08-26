include ../Makefile.common

ver=2.9.18

url=http://downloads.sourceforge.net/project/panotools/libpano13/libpano13-$(ver)/libpano13-$(ver).tar.gz

tar_file=$(build_dir)/deps/libpano13-$(ver).tar.gz
unpack=$(build_dir)/deps/libpano13-$(ver)

install=$(install_root)/bin/libpano

args = 	$(default_args) \
	CFLAGS=-I$(install_root)/include \
	LDFLAGS=-L$(install_root)/lib \
	--prefix=$(install_root) \
	--without-java

all:$(install)


$(install): $(unpack)
	cd $(unpack); ./configure $(args)
	cd $(unpack); make install

$(unpack): $(tar_file)
	cd $(build_dir)/deps; tar -xzf $(tar_file)

$(tar_file):
	curl -L -o $(tar_file) $(url)

uninstall:
	-rm -rf $(install)
	-rm $(install_root)/bin/panoinfo
	-rm $(install_root)/bin/PT*
	-rm $(install_root)/lib/libpano*
	-rm -rf $(install_root)/include/pano13

clean:
	-rm -rf $(unpack) $(tar_file) 

rebuild: uninstall $(install)

.PHONY: all uninstall clean rebuild
