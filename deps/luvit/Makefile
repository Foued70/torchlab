include ../Makefile.common

# local_source=/Users/dustin/src/luvit

ver=master

git_source=https://github.com/floored/luvit

ifdef local_source
	git_clone=$(local_source)
else
	git_clone=$(build_dir)/deps/luvit
	clean_clone=$(git_clone)
endif	


install=$(install_root)/bin/luvit

args = $(default_args)

all:$(install)


$(install): $(git_clone)
	cd $(git_clone); git checkout $(ver)
	cd $(git_clone); $(args) make
#	cd $(git_clone); $(args) make test
	cd $(git_clone); $(args) make install


$(git_clone):
	git clone --recursive $(git_source) $(git_clone)

uninstall:
	-rm $(install)
	-rm -rf $(install_root)/include/luvit
	-rm -rf $(install_root)/lib/luvit

clean: uninstall
	-rm -rf $(clean_clone)

.PHONY: clean uninstall
