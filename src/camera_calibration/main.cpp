/*
 * This file contains code to identify the camera distortion parameters
 * using OpenCV's camera calibration functions.
 */

#include <opencv2/core/core.hpp>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <dirent.h>
#include <string>
#include <sstream>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
	int numberOfHorizontalCorners;
	int numberOfVerticalCorners;
	string calibrationImagesDirectory;
	string enableChessboardPatternVerification;
	
	cout << "Please specify directory with calibration images \n";
	cout << "(Remember to specify directory path with terminal '/' character): \n";
	cin >> calibrationImagesDirectory;
	
	cout << "Enable visual verification of detected chessboard pattern? (Y/N) \n";
	cin >> enableChessboardPatternVerification;
	
	cout << "Enter number of horizontal corners: \n";
	cin >> numberOfHorizontalCorners;
	
	cout << "Enter number of vertical corners: \n";
	cin >> numberOfVerticalCorners;

	int numberOfSquares = numberOfHorizontalCorners * numberOfVerticalCorners;
	Size pattern_size = Size(numberOfHorizontalCorners, numberOfVerticalCorners);
	
	vector<vector<Point3f>> object_points;
	vector<vector<Point2f>> image_points;
	
	vector<Point2f> corners;
	
	Mat image;
	Mat gray_image;
	
	vector<Point3f> obj;
	
	// Create list of coordinates for chessboard corners
	for (int j = 0; j < numberOfSquares; j++) {
		obj.push_back(Point3f(j/numberOfHorizontalCorners, j%numberOfHorizontalCorners, 0.0f));
	}
	
	// Open calibration images directory
	DIR *pdir = NULL;
	struct dirent *pent = NULL;
	pdir = opendir(calibrationImagesDirectory.c_str());
	
	if (pdir == NULL) {
		printf("\nERROR: Calibration images directory appears to be incorrect");
		exit(1);
	}
	
	// Iterate through and process images in the calibration images directory
	while ((pent = readdir(pdir))) {
		if (pent == NULL) {
			printf("\nERROR: Calibration images directory appears to be empty");
			exit(3);
		}
		
		stringstream imagePath;
		imagePath << calibrationImagesDirectory.c_str() << pent->d_name;
		
		image = imread(imagePath.str().c_str(), CV_LOAD_IMAGE_UNCHANGED);
		
		// Skip non-image files (e.g. ".", "..", ".DS_STORE", etc)
		if (image.data == NULL) {
			continue;
		}
		
		cvtColor(image, gray_image, CV_BGR2GRAY);
		bool found = findChessboardCorners(image, pattern_size, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
		
		if (found) {
			cornerSubPix(gray_image, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			drawChessboardCorners(gray_image, pattern_size, corners, found);
			image_points.push_back(corners);
			object_points.push_back(obj);
			
			cout << ".";
		} else {
			printf("ERROR: Cannot detect chessboard pattern for %s", imagePath.str().c_str());
		}
		
		if (enableChessboardPatternVerification == "Y" || enableChessboardPatternVerification == "y") {
			namedWindow("Processed", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
			resize(gray_image, gray_image, Size(1024, 768));
			imshow("Processed", gray_image);
			waitKey();
		}
	}

	printf("All images processed!\n");

	Mat intrinsic = Mat(3, 3, CV_32FC1); // camera matrix
	Mat distCoeffs; // distortion coefficients
	vector<Mat> rvecs; // rotation vectors
	vector<Mat> tvecs; // translation vectors
	
	calibrateCamera(object_points, image_points, image.size(), intrinsic, distCoeffs, rvecs, tvecs);
	
	cout << "\nIntrinsic matrix: \n";
	cout << intrinsic;
	cout << "\n";
	cout << "\nDistortion vector (k_1, k_2, p_1, p_2, k3): \n";
	cout << distCoeffs;
	
	Mat undistorted_image;
	undistort(image, undistorted_image, intrinsic, distCoeffs);
		
	resize(image, image, Size(1024, 768));
	imshow("Original", image);

	resize(undistorted_image, undistorted_image, Size(1024, 768));
	imshow("Undistorted", undistorted_image);
	
	imwrite("/Users/ming/Desktop/processed-original.png", image);
	imwrite("/Users/ming/Desktop/processed-undistorted.png", undistorted_image);
	
	waitKey();
	
	return 0;
}