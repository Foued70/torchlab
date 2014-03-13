include ../Makefile.common

all:
	sed -e "s,@project_dir@,$(project_dir)," $(CURDIR)/cloudlab > $(install_root)/bin/torchlab

	chmod a+x $(install_root)/bin/torchlab
	cp $(CURDIR)/cook $(install_root)/bin/


clean:
	-rm $(install_root)/bin/cloudlab
	-rm $(install_root)/bin/cook

.PHONY: all clean
