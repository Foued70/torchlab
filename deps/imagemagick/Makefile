include ../Makefile.common

ver=6.7.7-6

url=http://downloads.sf.net/project/machomebrew/mirror/ImageMagick-$(ver).tar.bz2

tar_file=$(build_dir)/deps/ImageMagick-$(ver).tar.bz2
unpack=$(build_dir)/deps/ImageMagick-$(ver)

install=$(install_root)/bin/convert

args = \
	--prefix=$(install_root) \
	-disable-osx-universal-binary \
	--without-perl \
	--disable-dependency-tracking \
	--enable-shared \
	--disable-static \
	--without-pango \
	--with-included-ltdl \
	--with-modules \
	--without-gslib \
	--without-x \
	--without-fontconfig \
	--without-lzma \
	--without-rsvg \
	CC=$(install_root)/bin/gcc \
	CXX=$(install_root)/bin/g++


all:$(install)


$(install): $(unpack)
	cd $(unpack); ./configure $(args)
	cd $(unpack); make install

$(unpack): $(tar_file)
	cd $(build_dir)/deps; tar -xjf $(tar_file)

$(tar_file):
	curl -L -o $(tar_file) $(url)





