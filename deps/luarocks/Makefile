include ../Makefile.common

ver=2.0.12

url=http://luarocks.org/releases/luarocks-$(ver).tar.gz

tar_file=$(build_dir)/deps/luarocks-$(ver).tar.gz
unpack=$(build_dir)/deps/luarocks-$(ver)

install=$(install_root)/bin/luarocks

args = \
  --prefix=$(install_root) \
  --with-lua=$(install_root) \
  --with-lua-include=$(install_root)/include/torch \
  --rocks-tree=$(project_dir)/rocks \
  --force-config 


all:$(install)
	
	
$(install): $(unpack)
	cd $(install_root)/bin; ln -sf torch-lua lua
	cd $(unpack); ./configure $(args)
	cd $(unpack); LUADIR=$(install_root)/share/torch/lua make install
	! grep MAKE $(install_root)/etc/luarocks/config.lua && echo "variables.MAKE = 'make'" >>  $(install_root)/etc/luarocks/config.lua

$(unpack): $(tar_file)
	cd $(build_dir)/deps; tar -xzf $(tar_file)

$(tar_file):
	curl -L -o $(tar_file) $(url)

