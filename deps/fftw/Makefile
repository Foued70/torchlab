include ../Makefile.common

ver=3.3.3

url=http://www.fftw.org/fftw-$(ver).tar.gz

tar_file=$(build_dir)/deps/fftw-$(ver).tar.bz2
unpack=$(build_dir)/deps/fftw-$(ver)

install=$(install_root)/lib/libfftw3.a

args = $(default_args) \
	--prefix=$(install_root) \
	--enable-shared \
	--disable-debug \
	--enable-threads \
	--disable-dependency-tracking \




all:$(install)


$(install): $(unpack)
	cd $(unpack); ./configure $(args)
	cd $(unpack); make install

$(unpack): $(tar_file)
	cd $(build_dir)/deps; tar -xjf $(tar_file)

$(tar_file):
	curl -L -o $(tar_file) $(url)
