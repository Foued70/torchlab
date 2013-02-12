include ../Makefile.common

ver=4.8.4

url=http://releases.qt-project.org/qt4/source/qt-everywhere-opensource-src-$(ver).tar.gz

tar_file=$(build_dir)/deps/qt-everywhere-opensource-src-$(ver).tar.gz
unpack=$(build_dir)/deps/qt-everywhere-opensource-src-$(ver)

install=$(install_root)/lib/QtCore.la

args= \
	-prefix $(install_root) \
	-system-zlib \
	-confirm-license \
	-opensource \
	-cocoa \
	-fast \
	-nomake demos \
	-nomake examples 

#	-platform macx-g++-local



all:$(install)

$(install): $(unpack)
	# cd $(unpack); sed -i .bak 's^echo "CARBON_CFLAGS =-fconstant-cfstrings"^#^g' configure
	# cd $(unpack); cp -r mkspecs/macx-g++42 mkspecs/macx-g++-local
	# cd $(unpack)/mkspecs/macx-g++-local; sed -i .bak 's^gcc-4.2^$(install_root)/bin/gcc^g' qmake.conf
	# cd $(unpack)/mkspecs/macx-g++-local; sed -i .bak 's^g++-4.2^$(install_root)/bin/g++^g' qmake.conf
	export PATH
	echo $(PATH)
	cd $(unpack); ./configure $(args)
	cd $(unpack); make
	cd $(unpack); make install

$(unpack): $(tar_file)
	cd $(build_dir)/deps; tar -xzf $(tar_file)

$(tar_file):
	curl -L -o $(tar_file) $(url)
