include ../Makefile.common

url=http://people.csail.mit.edu/sparis/code/src/tone_mapping.tar.gz

tar_file=$(build_dir)/deps/tone_mapping.tar.gz
unpack=$(build_dir)/deps/tone_mapping
install=$(install_root)/bin/tone_mapping

# OpenEXR section. You may need to change EXR_INCDIR and EXR_LIBDIR.
EXR_INCDIR = -I$(install_root)/include/OpenEXR
EXR_LIBDIR = -L$(install_root)/lib
EXR_LIBS   = $(EXR_LIBDIR) -lIlmImf -lImath -lHalf -lIex -lz

# FFTW section. You may need to change FFTW_INCDIR and FFTW_LIBDIR.
FFTW_INCDIR = -I$(install_root)/include
FFTW_LIBDIR = -L$(install_root)/lib
FFTW_LIBS   = $(FFTW_LIBDIR) -lfftw3

# General section
INCDIR    = -I. -Ifft_3D -Iinclude $(EXR_INCDIR) $(FFTW_INCDIR)
LIBDIR    =
LIBS      = $(LIBDIR) $(EXR_LIBS) $(FFTW_LIBS) -lm
OBJECTS   = $(unpack)/tone_mapping.o $(unpack)/fft_3D/fft_3D.o
TARGET    = $(install_root)/bin/tone_mapping

all:$(install)

$(install): $(unpack)
	cd $(unpack)/fft_3D; $(CXX) support_3D.cpp -c -o fft_3D.o -I../include $(FFTW_INCDIR)
	cd $(unpack); $(CXX) tone_mapping.cpp -c -o tone_mapping.o $(INCDIR)
	cd $(unpack); $(LINK) $(OBJECTS) $(INCDIR) $(LIBS) -o $(TARGET)

$(unpack): $(tar_file)
	cd $(build_dir)/deps; tar -s '/TONE_MAPPING/tone_mapping/' -xzf $(tar_file)
	-- patch -p1 -d $(unpack) < tone_mapping.patch

$(tar_file):
	curl -L -o $(tar_file) $(url)
