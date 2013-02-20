include ../Makefile.common

ver=4.0.2


url=http://www.cise.ufl.edu/research/sparse/SuiteSparse/SuiteSparse-$(ver).tar.gz
tar_file=$(build_dir)/deps/SuiteSparse-$(ver).tar.gz
unpack=$(build_dir)/deps/SuiteSparse

install=$(install_root)/lib/temp

args=--prefix=$(install_root)


all:$(install)


$(install): $(unpack)
	cd $(unpack)/SuiteSparse_config; mv SuiteSparse_config_Mac.mk SuiteSparse_config.mk
	cd $(unpack); make ; make install

$(unpack): $(tar_file)
	cd $(build_dir)/deps; tar -xzf $(tar_file)

$(tar_file):
	curl -L -o $(tar_file) $(url)


