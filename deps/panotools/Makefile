include ../Makefile.common

ver=2.9.18

url=http://downloads.sourceforge.net/project/panotools/libpano13/libpano13-$(ver)/libpano13-$(ver).tar.gz

tar_file=$(build_dir)/deps/libpano13-$(ver).tar.gz
unpack=$(build_dir)/deps/libpano13-$(ver)

install=$(install_root)/bin/libpano

args = 	$(default_args) \
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

clean:
	-rm -rf $(install)

deepclean: clean
	-rm -rf $(unpack) $(tar_file) 
