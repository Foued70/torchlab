include ../Makefile.common

ver=2.5.0rc1


url=http://protobuf.googlecode.com/files/protobuf-$(ver).tar.bz2

tar_file=$(build_dir)/deps/protobuf-$(ver).tar.bz2
unpack=$(build_dir)/deps/protobuf-$(ver)

install=$(install_root)/bin/protoc

args=--prefix=$(install_root) \


all:$(install)


$(install): $(unpack)
	cd $(unpack); ./configure $(args)
	# cd $(unpack); make check
	cd $(unpack); make install

$(unpack): $(tar_file)
	cd $(build_dir)/deps; tar -xjf $(tar_file)

$(tar_file):
	curl -L -o $(tar_file) $(url)


