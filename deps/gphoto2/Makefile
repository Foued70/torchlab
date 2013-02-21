include ../Makefile.common

ver        = 2.5.1

dirname    = gphoto2-$(ver)
tarname    = $(dirname).tar.bz2
url        = http://downloads.sourceforge.net/project/gphoto/gphoto/$(ver)/$(tarname)

tar_file   = $(build_dir)/deps/$(tarname)
unpack     = $(build_dir)/deps/$(dirname)

bindir     = $(install_root)/bin
libdir     = $(install_root)/lib
includedir = $(install_root)/include
install    = $(bindir)/gphoto2

extra_flags = CPP_FLAGS=-D_DARWIN_C_SOURCE CFLAGS=-D_DARWIN_C_SOURCE

# there is no configure flag for libpopt so set the environment variables
env        = POPT_CFLAGS=" -I$(includedir) " \
	     POPT_LIBS=" -L$(libdir) -lpopt "

args=--prefix=$(install_root) \
	--without-cdk \
	--without-readline 

all:$(install)


$(install): $(unpack)
	cd $(unpack); $(env) $(extra_flags) $(pkg_config_path) ./configure $(args)
	cd $(unpack); make install

$(unpack): $(tar_file)
	cd $(build_dir)/deps; tar -xjf $(tar_file)

$(tar_file):
	curl -L -o $(tar_file) $(url)


