# you need to have CLOUDLAB_INSTALL_ROOT set in your environment for this to work

matching: corners.cpp
	g++ corners.cpp -o corners \
	-I${CLOUDLAB_INSTALL_ROOT}/include \
	-L${CLOUDLAB_INSTALL_ROOT}/lib \
	-lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_nonfree \
	-lopencv_features2d -lopencv_calib3d -lopencv_flann \
	-framework Accelerate
