include ../Makefile.common

ver=1.7.0
ilm_ver=1.0.2

url=http://download.savannah.gnu.org/releases/openexr/openexr-$(ver).tar.gz
ilm_url=http://download.savannah.gnu.org/releases/openexr/ilmbase-$(ilm_ver).tar.gz

tar_file=$(build_dir)/deps/openexr-$(ver).tar.gz
ilm_tar_file=$(build_dir)/deps/ilmbase-$(ilm_ver).tar.gz
unpack=$(build_dir)/deps/openexr-$(ver)
ilm_unpack=$(build_dir)/deps/ilmbase-$(ilm_ver)

install=$(install_root)/bin/exrheader
ilm_install=$(install_root)/lib/libIlmThread.a

args = CC="$(install_root)/bin/gcc" CXX="$(install_root)/bin/g++" \
	--prefix=$(install_root) \
	--disable-debug \
	--disable-dependency-tracking \




all:$(install)


$(install): $(ilm_install) $(unpack)
	cd $(unpack); ./configure $(args)
	cd $(unpack); make install

$(unpack): $(tar_file)
	cd $(build_dir)/deps; tar -xzf $(tar_file)
	patch -p1 -d $(unpack) < openexr.patch

$(tar_file):
	curl -L -o $(tar_file) $(url)

$(ilm_install): $(ilm_unpack)
	cd $(ilm_unpack); ./configure $(args)
	cd $(ilm_unpack); make install

$(ilm_unpack): $(ilm_tar_file)
	cd $(build_dir)/deps; tar -xzf $(ilm_tar_file)
	patch -p1 -d $(ilm_unpack) < ilm.patch

$(ilm_tar_file):
	curl -L -o $(ilm_tar_file) $(ilm_url)
