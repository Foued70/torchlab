packages = ui protobuf

all: $(packages)

$(packages):
	cd src/$@; cook
	
deps: build
	cd deps/gcc; make
	cd deps/qt; make
	cd deps/imagemagick; make
	cd deps/jpeg; make
	cd deps/torch; make
	cd deps/luarocks; make
	cd deps/cloudlab; make

build: 
	mkdir -p build