include ../Makefile.common

ver=2.0


url=http://gflags.googlecode.com/files/gflags-$(ver)-no-svn-files.tar.gz
tar_file=$(build_dir)/deps/gflags-$(ver)-no-svn-files.tar.gz
unpack=$(build_dir)/deps/gflags-$(ver)

install=$(install_root)/lib/libgflags.dylib

args=--prefix=$(install_root) \


all:$(install)


$(install): $(unpack)
	cd $(unpack); ./configure $(args)
	cd $(unpack); make install

$(unpack): $(tar_file)
	cd $(build_dir)/deps; tar -xzf $(tar_file)

$(tar_file):
	curl -L -o $(tar_file) $(url)


