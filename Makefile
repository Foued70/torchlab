packages = image

all: deps $(packages)

$(packages):
	cd src/$@; cook

.PHONY: deps gphoto ceres opencv

deps:
	mkdir -p build
	cd deps/luvit; make
	cd deps/graphicsmagick; make
	cd deps/jpeg; make
	cd deps/torch; make
	cd deps/cloudlab; make
	cd deps/glfw; make


gphoto:
	cd deps/libusb; make
	cd deps/libusb-compat; make
	cd deps/libexif; make
	cd deps/libgphoto2; make
	cd deps/libpopt; make
	cd deps/gphoto2; make

ceres: 
	cd deps/glog; make
	cd deps/gflags; make
	cd deps/suitesparse; make
	cd deps/ceres-solver; make

opencv: 
	cd deps/opencv; make
