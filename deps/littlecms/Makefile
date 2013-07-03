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
