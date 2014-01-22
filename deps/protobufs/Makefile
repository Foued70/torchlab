include ../Makefile.common

ver=2.5.0
url=https://protobuf.googlecode.com/files/protobuf-$(ver).tar.gz
tar_file=$(build_dir)/deps/protobuf-$(ver).tar.gz
unpack=$(build_dir)/deps/protobuf-$(ver)

install=$(install_root)/lib/libprotobuf.dylib

args=--prefix=$(install_root) \

all:$(install)


$(install): $(unpack)
	cd $(unpack); ./configure $(args)
	cd $(unpack); make install

$(unpack): $(tar_file)
	cd $(build_dir)/deps; tar -xzf $(tar_file)

$(tar_file):
	curl -L -o $(tar_file) $(url)


