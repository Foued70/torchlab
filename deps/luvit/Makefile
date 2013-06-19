include ../Makefile.common

ver=6407b23d13e21e3376ea5558921e6d9df325d328
url=https://github.com/luvit/luvit.git

git_clone=$(build_dir)/deps/luvit

install=$(install_root)/bin/luvit

args = PREFIX=$(install_root) \
	CC=$(install_root)/bin/gcc CXX=$(install_root)/bin/g++ 


all:$(install)


$(install): $(git_clone)
	cd $(git_clone); git checkout $(ver)
	cd $(git_clone); $(args) make
	cd $(git_clone); $(args) make test
	cd $(git_clone); $(args) make install


$(git_clone):
	git clone --recursive $(url) $(git_clone)

