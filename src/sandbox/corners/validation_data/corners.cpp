#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <Accelerate/Accelerate.h>
#include <iostream>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <math.h>

using namespace cv;
using namespace std;

/// Global variables
Mat src, src_gray, target, target_gray;
int thresh = 70;
int corr_thresh = 2;
int radius = 12;
int max_radius = 50;
int max_thresh = 255;

char* source_window = "Source image";
char* corners_window = "Corners detected";

std::vector<Point> src_points;
std::vector<Point> target_points;

/// Function header
void cornerHarris_demo(Mat img, std::vector<Point> &points);

void solveAffinePoint(Point p0, Point p1, Point p0_, Point p1_, double * b)
{
    double a[16] = { -1*p0.y , p0.x , -1*p1.y , p1.x, p0.x , p0.y , p1.x , p1.y , 1 , 0 , 1 , 0 , 0 , 1 , 0 , 1 };
    
    b[0] = p0_.x; 
    b[1] = p0_.y; 
    b[2] = p1_.x; 
    b[3] = p1_.y;
    
    __CLPK_integer m = 4, n = 4, nrhs = 1, lda = 4, ldb = 4, info, lwork;
    
    double wkopt;
    double* work;
    
    lwork = -1;
    
    dgels_("N", &m, &n, &nrhs, a, &lda, b, &ldb, &wkopt, &lwork, &info );
    lwork = (int)wkopt;
    work = (double*)malloc( lwork*sizeof(double) );
    
    dgels_( "N", &m, &n, &nrhs, a, &lda, b, &ldb, work, &lwork, &info );
    
    if (info)
        printf("Could not solve system; dgels exited with error %d\n", (int) info);
    /*else
	{
		printf("Solution is :\n");
		for (int i=0; i<4; i++)
		{
			printf("%f\n", b[i]);
		}
    }*/
}

float dist(Point a, Point b)
{
	return sqrt( pow((float)(a.x - b.x), 2) + pow((float)(a.y - b.y), 2));
}

Point transformPoint(Point p, double * b)
{
    Point ret = Point(0, 0);
    ret.x = b[1]*p.x - b[0]*p.y + b[2];
    ret.y = b[0]*p.x + b[1]*p.y + b[3];
    
    return ret;
}

int debug_ctr = 0;

void writeImage(Mat src, Mat target, double * max_b, string result_path)
{	
	Point transformed_src_bounds_1 = transformPoint(Point(0, 0), max_b);
	Point transformed_src_bounds_2 = transformPoint(Point(0, src.rows), max_b);
	Point transformed_src_bounds_3 = transformPoint(Point(src.cols, 0), max_b);
	Point transformed_src_bounds_4 = transformPoint(Point(src.cols, src.rows), max_b);
	
	int aa = std::min(std::min(transformed_src_bounds_1.x, transformed_src_bounds_2.x), std::min(transformed_src_bounds_3.x, transformed_src_bounds_4.x));
	int bb = std::min(std::min(transformed_src_bounds_1.y, transformed_src_bounds_2.y), std::min(transformed_src_bounds_3.y, transformed_src_bounds_4.y));
	
	int cc = std::max(std::max(transformed_src_bounds_1.x, transformed_src_bounds_2.x), std::max(transformed_src_bounds_3.x, transformed_src_bounds_4.x));
	int dd = std::max(std::max(transformed_src_bounds_1.y, transformed_src_bounds_2.y), std::max(transformed_src_bounds_3.y, transformed_src_bounds_4.y));
	
	Point transformed_src_min_bounds = Point(aa, bb);
	Point transformed_src_max_bounds = Point(cc, dd);
	
	//cerr<<"transformed_src_min_bounds :"<< transformed_src_min_bounds <<endl;
	//cerr<<"transformed_src_max_bounds :"<< transformed_src_max_bounds <<endl;
	
	int transformed_src_rows = transformed_src_max_bounds.y - transformed_src_min_bounds.y;
	int transformed_src_cols = transformed_src_max_bounds.x - transformed_src_min_bounds.x;
	
	Point transformed_src_translation = Point(std::max(transformed_src_min_bounds.x, 0), std::max(transformed_src_min_bounds.y, 0));
	Point target_translation = Point(std::max(-1*transformed_src_min_bounds.x, 0), std::max(-1*transformed_src_min_bounds.y, 0));
	
	//cerr<<"transformed_src_translation :"<< transformed_src_translation <<endl;
	//cerr<<"target_translation :"<< target_translation <<endl;
	
	Point transformed_src_center = transformPoint(Point(src.cols/2, src.rows/2), max_b) - transformed_src_min_bounds + transformed_src_translation;
	Point target_center = Point(target.cols/2, target.rows/2) + target_translation;

	cerr<<"Source Center :"<<transformed_src_center<<endl;
	cerr<<"Target Center :"<<target_center<<endl;

	Mat transformed_src = Mat::zeros( transformed_src_rows, transformed_src_cols, src.type() );
	
	Mat warp_mat = (Mat_<float>(2, 3) << max_b[1], -max_b[0], max_b[2] - transformed_src_min_bounds.x , max_b[0], max_b[1], max_b[3]- transformed_src_min_bounds.y );
	
	warpAffine( src, transformed_src, warp_mat, transformed_src.size() );
		
	Mat result = Mat::zeros(max(transformed_src_rows + transformed_src_translation.y, target.rows + target_translation.y), 	max(transformed_src_cols + transformed_src_translation.x, target.cols + target_translation.x), src.type());
	
	circle( result, transformed_src_center, 5,  Scalar(155,0,0), 2, 8, 0 );
	circle( result, target_center, 5,  Scalar(255,0,0), 2, 8, 0 );
	
	for (int i = 0; i < result.rows; i++)
	{
		for (int j = 0; j < result.cols; j++)
		{
			//careful about convention here. row is 'y', col is 'x'
			int transformed_src_row = i - transformed_src_translation.y;
			int transformed_src_col = j - transformed_src_translation.x;
			int target_row = i - target_translation.y;
			int target_col = j - target_translation.x;
			
			if(transformed_src_row > 0 && transformed_src_row < transformed_src.rows && transformed_src_col > 0 && transformed_src_col < transformed_src.cols)
				result.data[result.step[0]*i + result.step[1]* j + 2] += transformed_src.data[transformed_src.step[0]*transformed_src_row + transformed_src.step[1]* transformed_src_col + 2];
				
			if(target_row > 0 && target_row < target.rows && target_col > 0 && target_col < target.cols)
				result.data[result.step[0]*i + result.step[1]* j + 1] += target.data[target.step[0]*target_row + target.step[1]* target_col + 1];

		}
	}
	
// 	stringstream ss;
// 	ss << debug_ctr++;
// 	string str = ss.str();
// 	
// 	imwrite( str+std::string("_transformed_src.png"), transformed_src );
	
	imwrite( result_path, result );
}

/** @function main */
int main( int argc, char** argv )
{
	/// Load source image and convert it to gray
	
	cout<< argv[1]<<endl;
	cout<< argv[2]<<endl;
	cout<< argv[3]<<endl;
	
	src = imread( argv[1], 1 );
	target = imread ( argv[2], 1);
	char* result_path = argv[3];

	cvtColor( src, src_gray, CV_BGR2GRAY );
	cvtColor( target, target_gray, CV_BGR2GRAY );

	int erosion_size = 4;
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
												cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
												cv::Point(erosion_size, erosion_size) );

	cv::dilate(src_gray, src_gray, element);
	cv::erode(src_gray, src_gray, element);
	
	cv::dilate(target_gray, target_gray, element);
	cv::erode(target_gray, target_gray, element);


	cornerHarris_demo(src_gray, src_points);
	cornerHarris_demo(target_gray, target_points);
	
	int max_inliers = -1;
	int max_tx = 0;
	int max_ty = 0;
	double max_b[4] = {0, 1, 0, 0};
	
	int candidate = 0;
	
	cout<<argv[1]<<" to "<<argv[2]<<endl;
	
	for (std::vector<Point>::iterator it = src_points.begin(); it != src_points.end(); ++it)
    {
    	Point p_src = *it;
		for (std::vector<Point>::iterator it1 = target_points.begin(); it1 != target_points.end(); ++it1)
		{
			Point p_target = *it1;
			
				for (std::vector<Point>::iterator it2 = src_points.begin(); it2 != src_points.end(); ++it2)
				{
					Point p1_src = *it2;
					for (std::vector<Point>::iterator it3 = target_points.begin(); it3 != target_points.end(); ++it3)
					{
						Point p1_target = *it3;
						double b[4] = {0, 1, 0, 0};
						
						//points are equidistant in both scans and we haven't picked points that are too close together
						if(dist(p_src, p1_src) > 2*corr_thresh && dist(p_target, p1_target) > 2*corr_thresh &&  abs(dist(p_src, p1_src) -  dist(p_target, p1_target)) < corr_thresh)
						{
							//find inliers

							solveAffinePoint(p_src, p1_src, p_target, p1_target, b);
							
							int inliers = 0;
							for (std::vector<Point>::iterator pt = src_points.begin(); pt != src_points.end(); ++pt)
							{
								Point q_src = *pt;
								for (std::vector<Point>::iterator pt1 = target_points.begin(); pt1 != target_points.end(); ++pt1)
								{
									Point q_target = *pt1;
									
									Point q_src_transformed = transformPoint(q_src, b);

									if(dist(q_target, q_src_transformed) < corr_thresh)
										inliers++;
								}
							}
							
							if(inliers >= max_inliers)
							{
								//cout<<"Max found at "<<p_src << "-->"<< p_target << p1_src<< "-->" << p1_target<<endl;
															
								max_inliers = inliers;								
								std::copy(b, b+4, max_b);
								
								std::stringstream ss1;
								ss1 << (candidate);

								std::string result_num(ss1.str());
	
								string result_path_str = std::string(result_path) + std::string("/") + result_num + std::string(".png");
																		
								cerr<<"Candidate "<<candidate<<endl;
								cerr<<"Inliers : "<<inliers<<endl;
								
								writeImage(src, target, b, result_path_str);
								
								//cerr<<endl;
								
								candidate++;
							}
						}
						
					}
				}
		}
    }
	
	return(0);
}

/** @function cornerHarris_demo */
void cornerHarris_demo(Mat img, std::vector<Point> &points)
{

	Mat dst, dst_norm, dst_norm_scaled;
	dst = Mat::zeros( img.size(), CV_32FC1 );

	/// Detector parameters
	int blockSize = 2;
	int apertureSize = 3;
	double k = 0.04;

	/// Detecting corners
	cornerHarris( img, dst, blockSize, apertureSize, k, BORDER_DEFAULT );

	/// Normalizing
	normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
	convertScaleAbs( dst_norm, dst_norm_scaled );

	int h = dst_norm.rows;
	int w = dst_norm.cols;

	cout << "Finding interest points at threshold "<<thresh << " & radius " << radius << endl;

	/// Drawing a circle around corners
	for( int y = 0; y < h ; y++ )
	 { for( int x = 0; x < w; x++ )
		  {
		  
			float max = -1.0;
			int max_x = -1;
			int max_y = -1;

			//loop within 3x3 window
			for (int wy = y-radius; wy <= y + radius; wy++) {
				for (int wx = x-radius; wx <= x + radius; wx++) {
					if(wx >=0 && wx < w && wy >=0 && wy < h)
					{
						if(dst_norm.at<float>(wy,wx) > max)
						{
							max = dst_norm.at<float>(wy,wx);
							max_x = wx;
							max_y = wy;
						}
					}
				}
			}

			//ensure interest point is above threshold and is local minimum
			if((int)dst_norm.at<float>(y,x) > thresh && x == max_x && y==max_y)
				//circle( dst_norm_scaled, Point(x, y), 5,  Scalar(0), 2, 8, 0 );
				points.push_back(Point(x, y));
		  }
	 }
	 
	 cout<<"Number of points found: "<<points.size()<<endl;
	/// Showing the result
	//imshow( corners_window, dst_norm_scaled );
}