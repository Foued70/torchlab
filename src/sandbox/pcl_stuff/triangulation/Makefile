include ../Makefile.common

args = $(default_cmake_args)

install:
	mkdir -p build
	cd build; cmake $(args) ..
	cd build; make -j8 install

clean: 
	rm -rf build

.PHONY: install clean
