packages = $(notdir $(wildcard src/*))

all: $(packages)

$(packages):
	cd src/$@; torch-pkg deploy
	
deps: build
	cd vendor/gcc; make
	cd vendor/qt; make
	cd vendor/imagemagick; make
	cd vendor/jpeg; make
	cd vendor/torch; make
	cd vendor/luarocks; make

build: 
	mkdir -p build