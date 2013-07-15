include ../Makefile.common

ver=1.6.2

url=http://downloads.sourceforge.net/project/libpng/libpng16/$(ver)/libpng-$(ver).tar.gz

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


