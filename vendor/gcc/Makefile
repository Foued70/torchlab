include ../Makefile.common

URL=http://downloads.sourceforge.net/project/hpc/hpc/gcc/gcc-mlion.tar.gz

tar_file=$(build_dir)/vendor/gcc-mlion.tar.gz

install=$(install_root)/bin/gcc

all: $(install)
	

$(install): $(tar_file)
	cd $(build_dir); tar -xzf $(tar_file)
	mv $(build_dir)/usr/local/* $(install_root)/
	cd $(install_root)/lib; $(CURDIR)/fix_paths

$(tar_file):
	mkdir -p $(build_dir)/vendor
	curl -L -o $(tar_file) $(URL)

	