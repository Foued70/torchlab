#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <Accelerate/Accelerate.h>
#include <iostream>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <math.h>
#include <string>
#include <sstream>

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

/// Function header
void extractHarrisCorners(Mat img, std::vector<Point> &points, int threshold);

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

void resizeMat(Mat& m, size_t rows, size_t cols, const Scalar& s)
{
    Mat tm(m.rows + rows, m.cols + cols, m.type());
    tm.setTo(s);
    m.copyTo(tm(Rect(Point(0, 0), m.size())));
    m = tm;
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

Point transformPoint(Point p, Mat xform)
{
    Mat point = (Mat_<float>(3, 1) << p.x, p.y, 1);
    
    Mat transformed_point = xform * point;
    
    return Point(transformed_point.at<float>(1,1)/transformed_point.at<float>(3,1), transformed_point.at<float>(2,1)/transformed_point.at<float>(3,1));
}

float scoreTransformation(Mat src, Mat target, double * max_b)
{	
	float score; 
	int overlapping = 0;
	int total = 0;

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
	
	int transformed_src_rows = transformed_src_max_bounds.y - transformed_src_min_bounds.y;
	int transformed_src_cols = transformed_src_max_bounds.x - transformed_src_min_bounds.x;
	
	Point transformed_src_translation = Point(std::max(transformed_src_min_bounds.x, 0), std::max(transformed_src_min_bounds.y, 0));
	Point target_translation = Point(std::max(-1*transformed_src_min_bounds.x, 0), std::max(-1*transformed_src_min_bounds.y, 0));
	
	Point transformed_src_center = transformPoint(Point(src.cols/2, src.rows/2), max_b) - transformed_src_min_bounds + transformed_src_translation;
	Point target_center = Point(target.cols/2, target.rows/2) + target_translation;

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
			
			//calculate score
			if(result.data[result.step[0]*i + result.step[1]* j + 2] > 0 || result.data[result.step[0]*i + result.step[1]* j + 1] > 0)
			{
				total++;
				if(result.data[result.step[0]*i + result.step[1]* j + 2] == result.data[result.step[0]*i + result.step[1]* j + 1])
					overlapping++;
			}
		}
	}
	
	//write debug image
	//imwrite( result_path, result );
	
	score = (float) overlapping / (float) total * 100.0;
	
	return score;
}

Mat getTransfromation(Mat src, Mat target)
{
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
	
	std::vector<Point> src_points;
	std::vector<Point> target_points;

	extractHarrisCorners(src_gray, src_points, thresh);
	extractHarrisCorners(target_gray, target_points, thresh);
	
	int max_inliers = -1;
	float max_score = -1.0;
	int max_candidate = -1;
	int max_tx = 0;
	int max_ty = 0;
	double max_b[4] = {0, 1, 0, 0};
	
	int candidate = 0;

	
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
								//do validation here
								
								if(inliers > max_inliers)
								{
									max_inliers = inliers;	
									max_score = scoreTransformation(src, target, b);							
									max_candidate = candidate;
									std::copy(b, b+4, max_b);
								}
								else if(inliers == max_inliers)
								{
									float score = scoreTransformation(src, target, b);
								
									if(score > max_score)
									{
									//cout<<"Max found at "<<p_src << "-->"<< p_target << p1_src<< "-->" << p1_target<<endl;															
										max_inliers = inliers;	
										max_score = score;
										max_candidate = candidate;					
										std::copy(b, b+4, max_b);
									}								
								}							
								
								candidate++;
							}
						}
						
					}
				}
		}
    }
    
    cout<<"Candidates :"<<candidate<<endl;
    cout<<"Max Candidate :"<<max_candidate<<endl;
    cout<<"Max Score :"<<max_score<<endl;
	
	Mat warp_mat = (Mat_<float>(3, 3) << max_b[1], -max_b[0], max_b[2], max_b[0], max_b[1], max_b[3], 0, 0, 1);

	cout<<"Affine Warp has "<<max_inliers<<" inliers"<<endl;
	
	cout<<warp_mat<<endl;
	
	return warp_mat;
}

/** @function main */
int main( int argc, char** argv )
{

	int start = 90;
	int end = 70;
	
	char* result_path = argv[1];
	
	int result_size = 2700;
	int result_center_x = 500;
	int result_center_y = 1700;
	
	//x-form accumulator;
	Mat Acc = (Mat_<float>(3, 3) << 1, 0, result_center_x, 0, 1, result_center_y, 0, 0, 1);
	
	Mat result;

	int ctr = 10;
	
	for(int img_num = start; img_num > end; img_num--)
	{
		std::stringstream ss;
		ss << img_num;
		
		std::stringstream ss1;
		ss1 << (img_num - 1);

		
		std::string target_num(ss.str());
		std::string src_num(ss1.str());
	
		string src_filename = std::string("flattened_input_15/") + src_num + std::string(".png");
		string target_filename = std::string("flattened_input_15/") + target_num + std::string(".png");
		
		cout<<"Target: "<<target_filename<<endl;
		cout<<"Source: "<<src_filename<<endl;
		
		src = imread( src_filename, 1 );
		target = imread ( target_filename, 1);
		
		//copy first target into result
		if(img_num == start)
		{
			result = Mat::zeros(result_size, result_size, target.type());			
			warpPerspective( target, result, Acc, result.size() );
		}
		
		//get transformation of source
		Mat xform = getTransfromation(src, target);
		
		//accumulate transformation
		Acc = Acc * xform; 
		
		//save target
		Mat tempresult = result.clone();

		warpPerspective( src, result, Acc, result.size() );
		
		result = result + tempresult;
		
		imwrite( src_num+result_path, result );
	}
	
	imwrite( result_path, result );
	
	return(0);
}

/** @function cornerHarris_demo */
void extractHarrisCorners(Mat img, std::vector<Point> &points, int threshold)
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

	cout << "Finding interest points at threshold "<<threshold << " & radius " << radius << endl;

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
			if((int)dst_norm.at<float>(y,x) > threshold && x == max_x && y==max_y)
				//circle( dst_norm_scaled, Point(x, y), 5,  Scalar(0), 2, 8, 0 );
				points.push_back(Point(x, y));
		  }
	 }
	 
	 cout<<"Number of points found: "<<points.size()<<endl;
	/// Showing the result
	//imshow( corners_window, dst_norm_scaled );
}