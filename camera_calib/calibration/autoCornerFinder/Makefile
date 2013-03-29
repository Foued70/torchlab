# Project: Project1
# Makefile created by Dev-C++ 4.9.9.2

# **** CHANGE THESE ****
CLOUDLABLIBPATH = /Users/marco/local/lib
CLOUDLABINCLUDEPATH = /Users/marco/local/include

 
CPP  = g++
CC   = gcc
RES  = 
OBJ  = main.o cvcalibinit3.o $(RES)
LINKOBJ  = main.o cvcalibinit3.o $(RES)
LIBS =  -L"/usr/lib" -L${CLOUDLABLIBPATH} -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_video
INCS =  -I"/usr/include"  -I${CLOUDLABINCLUDEPATH} -I${CLOUDLABINCLUDEPATH}/opencv 
CXXINCS = -I"/usr/include" -I${CLOUDLABINCLUDEPATH} -I${CLOUDLABINCLUDEPATH}/opencv
BIN  = FindCorners.exe
CXXFLAGS = $(CXXINCS)  
CFLAGS = $(INCS)  
RM = rm -f

.PHONY: all clean

all: $(BIN)

clean:
	${RM} $(OBJ) $(BIN)

$(BIN): $(OBJ)
	$(CPP) $(LINKOBJ) -g -o $(BIN) $(LIBS)

main.o: main.cpp
	$(CPP) -c -g main.cpp -o main.o $(CXXFLAGS)

cvcalibinit3.o: cvcalibinit3.cpp
	$(CPP) -c -g cvcalibinit3.cpp -o cvcalibinit3.o $(CXXFLAGS)
