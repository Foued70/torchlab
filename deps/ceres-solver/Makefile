include ../Makefile.common

ver=1.4.0

url=http://ceres-solver.googlecode.com/files/ceres-solver-$(ver).tar.gz
tar_file=$(build_dir)/deps/ceres-solver-$(ver).tar.gz
unpack=$(build_dir)/deps/ceres-solver-$(ver)

install=$(install_root)/lib/libceres.a

args = $(default_cmake_args)  \
	-DEIGEN_INCLUDE=$(install_root)/include/eigen3

all:$(install)

# the sed line is a hack to remove the -fast flag which our recent g++ does not understand.
# make test is not working for me (100% Exception). Though simple_bundle_adjuster gives correct answer on test data.
$(install): $(unpack)
	cd $(unpack); mkdir -p build
	cd $(unpack); sed 's! -fast ! !' CMakeLists.txt > tmp ; mv tmp CMakeLists.txt
	cd $(unpack)/build; cmake $(args) ..
	cd $(unpack)/build; make -j3
	cd $(unpack)/build; make install

$(unpack): $(tar_file)
	cd $(build_dir)/deps; tar -xzf $(tar_file)

$(tar_file):
	curl -L -o $(tar_file) $(url)


