include ../Makefile.common

URL=http://downloads.sourceforge.net/project/hpc/hpc/gcc/gcc-mlion.tar.gz

tar_file=$(build_dir)/vendor/gcc-mlion.tar.gz

install=$(build_dir)/usr/local/bin/gcc

all: $(install)
	

$(install): $(tar_file)
	cd $(build_dir); tar -xzf $(tar_file)
	cd $(install_root); $(CURDIR)/fix_paths

$(tar_file):
	mkdir -p $(build_dir)/vendor
	curl -L -o $(tar_file) $(URL)

test:
	