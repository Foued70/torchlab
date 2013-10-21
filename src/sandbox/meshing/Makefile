all: meshing

meshing: meshing.cpp
	g++ meshing.cpp -g -o meshing -lopencv_core -lopencv_highgui -lopencv_nonfree -lopencv_features2d -lopencv_calib3d -lopencv_flann -lopencv_imgproc -I/Users/stavbraun/cloudlab/build/deps/boost_1_54_0

debug: meshing.cpp
	g++ meshing.cpp -g -o meshing -lopencv_core -lopencv_highgui -lopencv_nonfree -lopencv_features2d -lopencv_calib3d -lopencv_flann -lopencv_imgproc -DDEBUG
