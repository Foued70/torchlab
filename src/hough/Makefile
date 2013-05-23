CC=g++
CFLAGS=-c -Wall
TARGET=hough
INCLUDE=-I/Users/lihui815/cloudlab/build/usr/local/include/
LIBS=-lopencv_core -lopencv_highgui

all: hough

hough: hough.o
	$(CC) hough.o $(LIBS) -o $(TARGET)

hough.o: hough.cpp
	$(CC) $(CFLAGS) $(INCLUDE) hough.cpp

clean:
	rm -rf *o $(TARGET)
