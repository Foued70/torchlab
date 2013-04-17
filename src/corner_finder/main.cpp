/*
 * This file contains code to identify the camera distortion parameters
 * using OpenCV's camera calibration functions.
 *
 * It saves the calibration data as a Lua source file that can be read
 * directly in Lua using dofile([filepath])
 */

#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/core/core.hpp>
#include <opencv/cv.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <dirent.h>
#include <string>
#include <sstream>

using namespace cv;
using namespace std;

struct userOptions {
	
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
	
private:
	string imagesDirectory;
	int horizontalCornersCount;
	int verticalCornersCount;
};


UserInput::UserInput() {
	imagesDirectory = "";
	horizontalCornersCount = -1;
	verticalCornersCount = -1;
}

Mat img;
Mat mptexture_img;
int scale_factor=7;
//int scale_factor=1;
string mptexture_path;

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


void promptAndSaveCalibrationOptions(userOptions& userOptions, UserInput* userInput) {
	string stringInput;
	int intInput;
	
	cout << "Please specify directory with calibration images \n";
	cout << "(Remember to specify directory path with terminal '/' character): \n";
	cin >> stringInput;
	userInput->setImagesDirectory(stringInput);
	
	userInput->setHorizontalCornersCount(3);
	
	userInput->setVerticalCornersCount(3);
    
    cout << "Please specify path with matterport tecture \n";
	cout << "(Remember to specify directory path with terminal '/' character): \n";
	cin >> stringInput;
    
    mptexture_path = stringInput;
	
}

void saveCornerPoints(vector<vector<Point2f> > image_points, int numberOfImages, string path) {
	ofstream cornerPointsFile;
	cornerPointsFile.open(path.c_str());
	
	//int numberOfImages = (int)image_points.size();
	cornerPointsFile << "-- The code below saves the checkerboard corner points identified by OpenCV \n";
	cornerPointsFile << "-- in a Torch tensor with the dimensions: \n";
	cornerPointsFile << "-- [number of images used in calibration] x [number of corners in checkerboard pattern] x [2 (corresponding to the x and y pixel positions] \n";
	cornerPointsFile << "\n";
	
	cornerPointsFile << "local cornerPoints = ";
	
	cornerPointsFile << "{";
	for (int i = 0; i < numberOfImages; i++) {
		cornerPointsFile << "{";
        int numberOfCorners = (int)image_points[i].size();
		for (int j = 0; j < numberOfCorners; j++) {
			cornerPointsFile << "{";
			cornerPointsFile << image_points[i][j].x << "," << image_points[i][j].y;
			cornerPointsFile << "}";
            if (j < numberOfCorners - 1) {
                cornerPointsFile << ",";
            }
		}
		
		cornerPointsFile << "}";
        
        if (i < numberOfImages - 1) {
            cornerPointsFile << ",";
        }
	}
	cornerPointsFile << "}";
	cornerPointsFile << ";\n";
    
    cornerPointsFile << "return cornerPoints;";
	
	cornerPointsFile.close();
}

void onMouse_p(int event, int x, int y, int flags, void* param)
{
    Point2f pt((float)(x),(float)(y));
    
    vector<Point2f> * corners = reinterpret_cast<vector<Point2f> *>(param);
    vector<Point2f> corners_d;
    
    if (event == CV_EVENT_LBUTTONDOWN)
    {
        corners_d.push_back(pt);
        cornerSubPix(img, corners_d, Size(11,11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
        corners->push_back(corners_d[0]);
        printf("[ %f x %f ]\n", corners_d[0].x, corners_d[0].y);
        drawChessboardCorners(img, Size(3,3), corners_d, false);
        imshow("Processed", img);
    }

}

void onMouse_r(int event, int x, int y, int flags, void* param)
{
    Point2f pt((float)(x),(float)(y));
    
    vector<Point2f> * corners = reinterpret_cast<vector<Point2f> *>(param);
    vector<Point2f> corners_d;
    
    if (event == CV_EVENT_LBUTTONDOWN)
    {
        corners_d.push_back(pt);
        cornerSubPix(mptexture_img, corners_d, Size(11,11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
        corners->push_back(corners_d[0]);
        printf("[ %f x %f ]\n", corners_d[0].x, corners_d[0].y);
        drawChessboardCorners(mptexture_img, Size(3,3), corners_d, false);
        imshow("Reference", mptexture_img);
    }

}

int main(int argc, char** argv)
{
	userOptions userOptions;
	UserInput* userInput = new UserInput();
	
	promptAndSaveCalibrationOptions(userOptions, userInput);
	
	vector<vector<Point3f> > object_points;
    vector<vector<Point2f> > image_points; // list of corner points
    vector<vector<Point2f> > image_points_ref; // list of corner points
	
	Mat image;
	Mat gray_image;
    Mat color_reference_image;
    Mat reference_image;
	
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
	
    int numberOfImages=0;
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
        color_reference_image = imread(mptexture_path.c_str(), CV_LOAD_IMAGE_UNCHANGED);
		// Skip non-image files (e.g. ".", "..", ".DS_STORE", etc)
		if (image.data == NULL or color_reference_image.data == NULL) {
			continue;
		}
		numberOfImages++;
		cvtColor(image, gray_image, CV_BGR2GRAY);
        cvtColor(color_reference_image, reference_image, CV_BGR2GRAY);

		vector<Point2f> corners_tmp;
        vector<Point2f> corners;
        
        vector<Point2f> corners_tmp_ref;
        vector<Point2f> corners_ref;
        
        namedWindow("Processed", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
        setMouseCallback("Processed", onMouse_p, (void*)&corners_tmp);
        resize(gray_image, img, Size(gray_image.cols/scale_factor, gray_image.rows/scale_factor));
        imshow("Processed", img);
        
        namedWindow("Reference", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
        setMouseCallback("Reference", onMouse_r, (void*)&corners_tmp_ref);
        resize(reference_image, mptexture_img, Size(reference_image.cols, reference_image.rows));
        imshow("Reference", mptexture_img);
        
        resize(gray_image, img, Size(gray_image.cols/scale_factor, gray_image.rows/scale_factor));
        resize(reference_image, mptexture_img, Size(reference_image.cols, reference_image.rows));
        
        waitKey();
        
        while (corners_tmp.size() != corners_tmp_ref.size()){
            if (corners_tmp.size() < corners_tmp_ref.size()){
                printf("Waiting for more points on calibration image");
            }
            else{
                printf("Waiting for more points on reference image");
            }
            waitKey();
        }
        
        setMouseCallback("Processed", 0, 0);
        setMouseCallback("Reference", 0, 0);
        
        if (corners_tmp.size()>0){
            for (int i=0; i<corners_tmp.size(); i++){
                Point2f pt(corners_tmp[i].x*scale_factor, corners_tmp[i].y*scale_factor);
                Point2f pt_ref(corners_tmp_ref[i].x, corners_tmp_ref[i].y);
                corners.push_back(pt);
                corners_ref.push_back(pt_ref);
            }
            cornerSubPix(gray_image, corners, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
            cornerSubPix(mptexture_img, corners_ref, Size(3, 3), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
            resize(gray_image, img, Size(gray_image.cols/scale_factor, gray_image.rows/scale_factor));
            resize(reference_image, mptexture_img, Size(reference_image.cols, reference_image.rows));
            for (int i=0; i<corners.size(); i++){
                vector<Point2f> corner_draw;
                Point2f pt(corners[i].x/scale_factor, corners[i].y/scale_factor);
                vector<Point2f> corner_draw_ref;
                Point2f pt_ref(corners_ref[i].x, corners_ref[i].y);
                corner_draw.push_back(pt);
                corner_draw_ref.push_back(pt_ref);
                drawChessboardCorners(img, Size(3,3), corner_draw, false);
                drawChessboardCorners(mptexture_img, Size(3,3), corner_draw_ref, false);
                printf("[ %f x %f ]; [ %f x %f ]\n", corners[i].x, corners[i].y, corners_ref[i].x, corners_ref[i].y);
            }
            
            imshow("Processed", img);
            imshow("Reference", mptexture_img);
            waitKey();
                
        }
        
        
        image_points.push_back(corners);
        image_points_ref.push_back(corners_ref);
        object_points.push_back(obj);
        
	}

	printf("All images processed! %d\n", numberOfImages);
    
    string path = "/Users/lihui815/Projects/cornerPoints.txt";
    string path_ref = "/Users/lihui815/Projects/cornerPoints_ref.txt";
	saveCornerPoints(image_points, numberOfImages, path);
    saveCornerPoints(image_points_ref, numberOfImages, path_ref);
	
	waitKey();
	
	return 0;
}
