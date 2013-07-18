include ../Makefile.common

ver=2.0.2

url=http://files.luaforge.net/releases/luasocket/luasocket/luasocket-$(ver)/luasocket-$(ver).tar.gz

tar_file=$(build_dir)/deps/luasocket-$(ver).tar.gz
unpack=$(build_dir)/deps/luasocket-$(ver)

install=$(install_root)/lib/xxx

args = $(default_args) \
	--prefix=$(install_root) \
	--enable-shared \
	--disable-debug \



all:$(install)


$(install): $(unpack)
	cd $(unpack)/src; LUAINC=-I$(install_root)/include/luvit/luajit LDFLAGS='-O -shared -fpic -undefined dynamic_lookup' make -e
	mkdir -p $(install_root)/lib/luvit/socket/modules/socket
	mkdir -p $(install_root)/lib/luvit/socket/modules/mime
	cp $(unpack)/src/*.lua $(install_root)/lib/luvit/socket/
	cd $(install_root)/lib/luvit/socket; mv socket.lua init.lua
	cp $(unpack)/src/socket.so.* $(install_root)/lib/luvit/socket/modules/socket_core.luvit
	cp $(unpack)/src/mime.so.* $(install_root)/lib/luvit/socket/modules/mime_core.luvit

$(unpack): $(tar_file)
	cd $(build_dir)/deps; tar -xzf $(tar_file)

$(tar_file):
	curl -L -o $(tar_file) $(url)
