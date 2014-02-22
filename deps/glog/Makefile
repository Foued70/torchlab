include ../Makefile.common

ver=0.3.3

url=http://google-glog.googlecode.com/files/glog-$(ver).tar.gz
tar_file=$(build_dir)/deps/glog-$(ver).tar.gz
unpack=$(build_dir)/deps/glog-$(ver)

install=$(install_root)/lib/libglog.dylib

args=--prefix=$(install_root) \


all:$(install)


$(install): $(unpack)
	cd $(unpack); CC=gcc-4.8 CXX=g++-4.8 ./configure $(args)
	cd $(unpack); make install

$(unpack): $(tar_file)
	cd $(build_dir)/deps; tar -xzf $(tar_file)

$(tar_file):
	curl -L -o $(tar_file) $(url)


