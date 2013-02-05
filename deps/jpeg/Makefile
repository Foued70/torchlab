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


