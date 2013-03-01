/*
 * This file contains code to identify the camera distortion parameters
 * using OpenCV's camera calibration functions.
 *
 * It saves the calibration data as a Lua source file that can be read
 * directly in Lua using dofile([filepath])
 */

#include <opencv2/core/core.hpp>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <dirent.h>
#include <string>
#include <sstream>

using namespace cv;
using namespace std;

struct userOptions {
	string enableChessboardPatternVerification;
	string useRationalModel;
	string fixFocalLength;
	string fixPrincipalPoint;
	string provideIntrinsicMatrixData;
	
	int calibrationFlags = 0;
};

class UserInput {
public:
	UserInput(const string& _directory, int _horizontalCorners, int _verticalCorners);
	UserInput();
	~UserInput();
	
	// Setters and Getters
	string getImagesDirectory();
	void setImagesDirectory(const string& _imagesDirectory);
	
	void setHorizontalCornersCount(int count);
	void setVerticalCornersCount(int count);
	int getHorizontalCornersCount();
	int getVerticalCornersCount();
	int getNumberOfSquares();
	Size getPatternSize();
	
	void setIntrinsicFx(int fx);
	void setIntrinsicFy(int fy);
	void setIntrinsicCx(int cx);
	void setIntrinsicCy(int cy);
	int getIntrinsicFx();
	int getIntrinsicFy();
	int getIntrinsicCx();
	int getIntrinsicCy();
	
private:
	string imagesDirectory;
	int horizontalCornersCount;
	int verticalCornersCount;
	int intrinsic_fx;
	int intrinsic_fy;
	int intrinsic_cx;
	int intrinsic_cy;
};


UserInput::UserInput() {
	imagesDirectory = "";
	horizontalCornersCount = -1;
	verticalCornersCount = -1;
}

void UserInput::setImagesDirectory(const string& _imagesDirectory) {
	imagesDirectory = _imagesDirectory;
}

string UserInput::getImagesDirectory() {
	return imagesDirectory;
}

void UserInput::setHorizontalCornersCount(int count) {
	horizontalCornersCount = count;
}

void UserInput::setVerticalCornersCount(int count) {
	verticalCornersCount = count;
}

int UserInput::getHorizontalCornersCount() {
	return horizontalCornersCount;
}

int UserInput::getVerticalCornersCount() {
	return verticalCornersCount;
}

int UserInput::getNumberOfSquares() {
	return horizontalCornersCount * verticalCornersCount;
}

Size UserInput::getPatternSize() {
	return Size(horizontalCornersCount, verticalCornersCount);
}

void UserInput::setIntrinsicFx(int fx) {
	intrinsic_fx = fx;
}

void UserInput::setIntrinsicFy(int fy) {
	intrinsic_fy = fy;
}

void UserInput::setIntrinsicCx(int cx) {
	intrinsic_cx = cx;
}

void UserInput::setIntrinsicCy(int cy) {
	intrinsic_cy = cy;
}

int UserInput::getIntrinsicFx() {
	return intrinsic_fx;
}

int UserInput::getIntrinsicFy() {
	return intrinsic_fy;
}

int UserInput::getIntrinsicCx() {
	return intrinsic_cx;
}

int UserInput::getIntrinsicCy() {
	return intrinsic_cy;
}


void promptAndSaveCalibrationOptions(userOptions& userOptions, UserInput* userInput) {
	string stringInput;
	int intInput;
	
	cout << "Please specify directory with calibration images \n";
	cout << "(Remember to specify directory path with terminal '/' character): \n";
	cin >> stringInput;
	userInput->setImagesDirectory(stringInput);
	
	
	cout << "Enable visual verification of detected chessboard pattern? (Y/N) \n";
	cin >> userOptions.enableChessboardPatternVerification;
	
	cout << "Enter number of horizontal corners: \n";
	cin >> intInput;
	userInput->setHorizontalCornersCount(intInput);
	
	cout << "Enter number of vertical corners: \n";
	cin >> intInput;
	userInput->setVerticalCornersCount(intInput);
	
	cout << "Use rational model? (Y/N) \n";
	cin >> userOptions.useRationalModel;
	
	cout << "Fix focal length? (Y/N) \n";
	cin >> userOptions.fixFocalLength;
	
	cout << "Fix principal point (i.e. cx, cy)? (Y/N) \n";
	cin >> userOptions.fixPrincipalPoint;
	
	cout << "Provide intrinsic matrix data? (Y/N) \n";
	cin >> userOptions.provideIntrinsicMatrixData;
	
	if (userOptions.provideIntrinsicMatrixData == "Y" || userOptions.provideIntrinsicMatrixData == "y") {
		
		cout << "Please provide intrinsic matrix data information. \n";
		
		cout << "fx? \n";
		cin >> intInput;
		userInput->setIntrinsicFx(intInput);
		
		cout << "fy? \n";
		cin >> intInput;
		userInput->setIntrinsicFy(intInput);
		
		cout << "cx? \n";
		cin >> intInput;
		userInput->setIntrinsicCx(intInput);
		
		cout << "cy? \n";
		cin >> intInput;
		userInput->setIntrinsicCy(intInput);
	}
	
	if (userOptions.provideIntrinsicMatrixData == "Y" || userOptions.provideIntrinsicMatrixData == "y") {
		userOptions.calibrationFlags |= CV_CALIB_USE_INTRINSIC_GUESS;
	}
	
	if (userOptions.useRationalModel == "Y" || userOptions.useRationalModel == "y") {
		userOptions.calibrationFlags |= CV_CALIB_RATIONAL_MODEL;
	}
	
	if (userOptions.fixFocalLength == "Y" || userOptions.fixFocalLength == "y") {
		userOptions.calibrationFlags |= CV_CALIB_FIX_FOCAL_LENGTH;
	}
	
	if (userOptions.fixPrincipalPoint == "Y" || userOptions.fixPrincipalPoint == "y") {
		userOptions.calibrationFlags |= CV_CALIB_FIX_PRINCIPAL_POINT;
	}
}

int main(int argc, char** argv)
{
	userOptions userOptions;
	UserInput* userInput = new UserInput();
	
	promptAndSaveCalibrationOptions(userOptions, userInput);
	
	vector<vector<Point3f>> object_points;
	vector<vector<Point2f>> image_points;
	
	Mat image;
	Mat gray_image;
	
	vector<Point3f> obj;
	
	// Create list of coordinates for chessboard corners
	for (int j = 0; j < userInput->getNumberOfSquares(); j++) {
		obj.push_back(Point3f(j/(userInput->getHorizontalCornersCount()), j%(userInput->getHorizontalCornersCount()), 0.0f));
	}
	
	// Open calibration images directory
	DIR *pdir = NULL;
	struct dirent *pent = NULL;
	pdir = opendir(userInput->getImagesDirectory().c_str());
	
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
		imagePath << userInput->getImagesDirectory().c_str() << pent->d_name;
		
		image = imread(imagePath.str().c_str(), CV_LOAD_IMAGE_UNCHANGED);
		cout << imagePath.str() << "\n";
		// Skip non-image files (e.g. ".", "..", ".DS_STORE", etc)
		if (image.data == NULL) {
			continue;
		}
		
		cvtColor(image, gray_image, CV_BGR2GRAY);

		vector<Point2f> corners;
		bool found = findChessboardCorners(gray_image, userInput->getPatternSize(), corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

		if (found) {
			cornerSubPix(gray_image, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			drawChessboardCorners(gray_image, userInput->getPatternSize(), corners, found);
			image_points.push_back(corners);
			object_points.push_back(obj);
		} else {
			printf("ERROR: Cannot detect chessboard pattern for %s", imagePath.str().c_str());
		}
		
		if (userOptions.enableChessboardPatternVerification == "Y" || userOptions.enableChessboardPatternVerification == "y") {
			namedWindow("Processed", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
			resize(gray_image, gray_image, Size(1024, 768));
			imshow("Processed", gray_image);
			waitKey();
		}
	}

	printf("All images processed!\n");
	
	Mat intrinsic;	// camera matrix
	if (userOptions.provideIntrinsicMatrixData == "Y" || userOptions.provideIntrinsicMatrixData == "y") {
		intrinsic = (Mat_<double>(3,3) << userInput->getIntrinsicFx(), 0, userInput->getIntrinsicCx(), 0, userInput->getIntrinsicFy(), userInput->getIntrinsicCy(), 0, 0, 1);
	} else {
		intrinsic = Mat(3, 3, CV_32FC1);
	}

	Mat			distCoeffs;		// distortion coefficients
	vector<Mat> rvecs;			// rotation vectors
	vector<Mat> tvecs;			// translation vectors

	calibrateCamera(object_points, image_points, image.size(), intrinsic, distCoeffs, rvecs, tvecs, userOptions.calibrationFlags);

	stringstream output;
	
	output << "calibrationData = { ";
	output << "fx = " << intrinsic.at<double>(0, 0) << ", ";
	output << "fy = " << intrinsic.at<double>(1, 1) << ", ";
	output << "cx = " << intrinsic.at<double>(0, 2) << ", ";
	output << "cy = " << intrinsic.at<double>(1, 2) << ", ";
	output << "k1 = " << distCoeffs.at<double>(0, 0) << ", ";
	output << "k2 = " << distCoeffs.at<double>(0, 1) << ", ";
	output << "p1 = " << distCoeffs.at<double>(0, 2) << ", ";
	output << "p2 = " << distCoeffs.at<double>(0, 3) << ", ";
	output << "k3 = " << distCoeffs.at<double>(0, 4) << ", ";
	
	if (distCoeffs.size().width > 5) { // Calibration using rational model will give additional parameters
		output << "k4 = " << distCoeffs.at<double>(0, 5) << ", ";
		output << "k5 = " << distCoeffs.at<double>(0, 6) << ", ";
		output << "k6 = " << distCoeffs.at<double>(0,7);
	}
	
	output << "} \n";

	printf("\n");
	cout << output.str();
	
	ofstream outputFile;
	outputFile.open("/Users/ming/Desktop/calibrationData");
	outputFile << output.str();
	outputFile.close();
	
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
