#include <stdio.h>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#define Pi ((float)CV_PI)

using namespace cv;
using namespace std;

float get_max_float(Mat matrix){

	int rows = matrix.rows;
	int cols = matrix.cols;

	float ret = 0.0;

	for (int i = 0; i < rows; i++){
		for (int j = 0; j < cols; j++){
			if (matrix.at<float>(i,j) > ret)
			{
				ret = matrix.at<float>(i,j);
			}
		}
	}
	return ret;
}

float get_min_float(Mat matrix){

	int rows = matrix.rows;
	int cols = matrix.cols;

	float ret = 0.0;

	for (int i = 0; i < rows; i++){
		for (int j = 0; j < cols; j++){
			if (matrix.at<float>(i,j) > ret)
			{
				ret = matrix.at<float>(i,j);
			}
		}
	}
	return ret;
}


float get_mean_float(Mat matrix){

	int rows = matrix.rows;
	int cols = matrix.cols;

	float ret = 0.0;
    float cnt = 0.0;
    
	for (int i = 0; i < rows; i++){
		for (int j = 0; j < cols; j++){
            float val = matrix.at<float>(i,j);
            if (val > 0)
            {
                ret += matrix.at<float>(i,j);
                cnt ++;
            }
		}
	}
    ret = ret/cnt;
	return ret;
}

float get_max_float_in_window(Mat matrix, int r, int c, int winr, int winc){

	int rows = matrix.rows;
	int cols = matrix.cols;

	float ret = 0.0;

	for (int i = r-winr; i < r+winr; i++){
		for (int j = c-winc; j < c+winc; j++){
            int ir = min(max(0, i), rows-1);
            int jc = j % cols;
			if (matrix.at<float>(ir,jc) > ret)
			{
				ret = matrix.at<float>(ir,jc);
			}
		}
    }
    
	return ret;
}

float get_window_average(Mat matrix, int r, int c, int winr, int winc){

	int rows = matrix.rows;
	int cols = matrix.cols;

	float ret = 0.0;
    float cnt = 0.0;

	for (int i = r-winr; i < r+winr; i++){
		for (int j = c-winc; j < c+winc; j++){
            int ir = min(max(0, i), rows-1);
            int jc = j % cols;
            ret += matrix.at<float>(ir,jc);
            cnt ++;
		}
    }

    ret = ret/cnt;
    
	return ret;
}

bool is_max_float_in_window(float query, Mat matrix, int r, int c, int winr, int winc){

	int rows = matrix.rows;
	int cols = matrix.cols;

	for (int i = r-winr; i < r+winr; i++){
		for (int j = c-winc; j < c+winc; j++){
            int ir = min(max(0, i), rows-1);
            int jc = j % cols;
			if (matrix.at<float>(ir,jc) > query)
			{
				return false;
			}
		}
	}
	return true;
}

static void
find_n_local_best_hough_lines(Mat houghTransform, int n, vector<int> * retr, vector<int> * reta)
{

    if (n <= 0)
    {
        return;
    }
    
    int numR = houghTransform.rows;
	int numA = houghTransform.cols;

    int winr = (int)(numR/100);
    int wina = (int)(numA/30);
    
    float maxVal = get_max_float(houghTransform);
    
    for (int nA = 0; nA < numA; nA++)
	{
		for (int nR = 0; nR < numR; nR++)
		{
            float val = houghTransform.at<float>(nR,nA);
            if (val > 0.05*maxVal && is_max_float_in_window(val, houghTransform, nR, nA, 2*winr, 2*wina))
            {
                int s = reta->size();
                if (s > 0)
                {
                
                    vector<int>::iterator itr;
                    vector<int>::iterator ita;
                    
                    itr = retr->begin();
                    ita = reta->begin();
                    
                    bool needinsertion = true;
                    for(; itr < retr->end(); itr++, ita++)
                    {
                        if (houghTransform.at<float>(*itr,*ita) < val)
                        {
                            retr->insert(itr, nR);
                            reta->insert(ita, nA);
                            needinsertion = false;
                            break;
                        }
                    }
                    if (needinsertion)
                    {
                        retr->push_back(nR);
                        reta->push_back(nA);
                    }
                    if (reta->size() > n)
                    {
                        retr->pop_back();
                        reta->pop_back();
                    }
                }
                else
                {
                    retr->push_back(nR);
                    reta->push_back(nA);
                }
            }
        }
    }
}

static void
draw_hough_lines(Mat image, Mat houghTransform, vector<int> retr, vector<int> reta)
{
	int numR = houghTransform.rows;
	int numA = houghTransform.cols;

	int numRows = image.rows;
	int numCols = image.cols;

	float maxRadius = sqrt(pow(numRows,2.0) + pow(numCols,2.0))/2.0;

	double incA = 2 * Pi / (numA);
	double incR = maxRadius / (numR - 1);
    
    vector<int>::iterator itr;
    vector<int>::iterator ita;
    
    itr = retr.begin();
    ita = reta.begin();
    
    for(; itr < retr.end(); itr++, ita++)
    {
        int nA = *ita;
        int nR = *itr;
        
        float val = houghTransform.at<float>(nR,nA);
        float angle = nA * incA;
        float rad = nR * incR;

        int x1, x2, y1, y2;
        if ( (angle <=3 * Pi/4 && angle >= Pi/4) || (angle <= 7*Pi/4 && angle >= 5*Pi/4))
        {
            x1 = 0;
            x2 = numCols;
            y1 = (int)((rad - cos(angle) * (x1-numCols/2)) / sin(angle)) + numRows/2;
            y2 = (int)((rad - cos(angle) * (x2-numCols/2)) / sin(angle)) + numRows/2;
        }
        else
        {
            y1 = 0;
            y2 = numRows;
            x1 = (int)((rad - sin(angle) * (y1-numRows/2)) / cos(angle)) + numCols/2;
            x2 = (int)((rad - sin(angle) * (y2-numRows/2)) / cos(angle)) + numCols/2;
        }

        line(image, Point(x1,y1), Point(x2,y2), Scalar( 250, 250, 250), 2, 0);
        printf("%f, %f, %f, [%d x %d], [%d x %d]\n", val, rad, angle, x1,y1, x2, y2);
        
    }

}

static void
find_hough_transform(Mat image, Mat houghTransform, int filtsize)
{
	int numR = houghTransform.rows;
	int numA = houghTransform.cols;

	int numRows = image.rows;
	int numCols = image.cols;

	float maxRadius = sqrt(pow(numRows,2.0) + pow(numCols,2.0))/2.0;

	double incA = 2 * Pi / (numA);
	double incR = maxRadius / (numR - 1);

	double minVal, maxVal;
	minMaxLoc(image, &minVal, &maxVal);

	for (int row = 0; row < numRows; row ++)
	{
		for (int col = 0; col < numCols; col ++)
		{
			Scalar Pt = image.at<uchar>(row,col);
			double val = Pt.val[0];

			if (val > 0.05 * maxVal)
			{			
				//increment vote
				for (int nA = 0; nA < numA; nA ++)
				{
					double angle = nA * incA;
					double rad = sin(angle) * (row-numRows/2) + cos(angle) * (col-numCols/2);
                    int rbucket = (int)(rad/incR);
					
					if (rad/incR >=0 && rad/incR < numR)
					{
						houghTransform.at<float>(rbucket,nA)+=(float)(val/maxVal+2.0);
                        
					}					
				}
			}
		}
	}
    
    Mat hcopy = houghTransform.clone();
    
    
    for (int nA = 0; nA < numA; nA ++){
        for (int nR = 0; nR < numR; nR ++){
            if (numA % 8 == 0 && nA % (int)(numA/8) == 0){
                float avg1 = get_window_average(hcopy, nR, nA, 1, 1);
                houghTransform.at<float>(nR,nA)=avg1;
                hcopy.at<float>(nR,nA)=avg1;
            }
            if(houghTransform.at<float>(nR,nA) > 0){
                float avg2 = get_window_average(hcopy, nR, nA, filtsize, filtsize);
                if (houghTransform.at<float>(nR,nA) <= avg2) {
                    houghTransform.at<float>(nR,nA) = 0;
                }
                else {
                    houghTransform.at<float>(nR,nA)=(houghTransform.at<float>(nR,nA)- avg2);
                }
            }
        }
    }
}

static void
find_hough_transform_vert(Mat image, Mat houghTransform, int filtsize)
{
	int numR = houghTransform.rows;
	int numA = houghTransform.cols;

	int numRows = image.rows;
	int numCols = image.cols;

	float maxRadius = sqrt(pow(numRows,2.0) + pow(numCols,2.0))/2.0;

	double incA = 2 * Pi / (numA);
	double incR = maxRadius / (numR - 1);

	double minVal, maxVal;
	minMaxLoc(image, &minVal, &maxVal);
    
    double aset = 7.5*Pi/180.0;

	for (int row = 0; row < numRows; row ++)
	{
		for (int col = 0; col < numCols; col ++)
		{
			Scalar Pt = image.at<uchar>(row,col);
			double val = Pt.val[0];

			if (val > 0.05 * maxVal)
			{			
				//increment vote
				for (int nA = 0; nA < numA; nA ++)
				{
					double angle = nA * incA;
                    if ((angle <= aset && angle >= 0) || (-angle+2*Pi <= aset && angle-2*Pi <= 0) || (angle-Pi <= aset && angle-Pi >= 0) || (-angle+Pi <= aset && angle-Pi <= 0))
					{
                        double rad = sin(angle) * (row-numRows/2) + cos(angle) * (col-numCols/2);
                        int rbucket = (int)(rad/incR);
					
                        if (rad/incR >=0 && rad/incR < numR)
                        {
                            houghTransform.at<float>(rbucket,nA)+=(float)(val/maxVal+2.5);
                        }
                    }
				}
			}
		}
	}
    
    Mat hcopy = houghTransform.clone();
    
    
    aset = aset/2;
    for (int nA = 0; nA < numA; nA ++){
        double angle = nA * incA;
        for (int nR = 0; nR < numR; nR ++){
            if ((angle <= aset && angle >= 0) || (-angle+2*Pi <= aset && angle-2*Pi <= 0) || (angle-Pi <= aset && angle-Pi >= 0) || (-angle+Pi <= aset && angle-Pi <= 0))
					{
                if (numA % 8 == 0 && nA % (int)(numA/8) == 0){
                    float avg1 = get_window_average(hcopy, nR, nA, 1, 1);
                    houghTransform.at<float>(nR,nA)=avg1;
                    hcopy.at<float>(nR,nA)=avg1;
                }
                if (houghTransform.at<float>(nR,nA) > 0){
                    float avg2 = get_window_average(hcopy, nR, nA, filtsize, filtsize);
                    if (houghTransform.at<float>(nR,nA) <= avg2) {
                        houghTransform.at<float>(nR,nA) = 0;
                    }
                    else {
                        houghTransform.at<float>(nR,nA)=(houghTransform.at<float>(nR,nA)- avg2);
                    }
                }
            }
            else {
                houghTransform.at<float>(nR,nA) = 0;
            }
        }
    }
}

int main( int argc, char** argv)
{
	
	if (argc < 6)
	{
		printf("Please use the following format: ./hough [0-hori,1-vert] [imageName] [numAngles] [numRadius] [numLines] [[outName] [houghName]]\n");
		return -1;
	}
    int flag = atoi(argv[1]);
	char* imageName = argv[2];
	int numAngles = atoi(argv[3]);
	int numRadius = atoi(argv[4]);
    int numLines = atoi(argv[5]);
    
    
    printf("%s\n", imageName);
	Mat image;
	image = imread( imageName, CV_LOAD_IMAGE_GRAYSCALE );

	Mat houghTransform(numAngles, numRadius, CV_32F);
    int filtsize = 5;//numAngles/100;
    
    if (flag == 0){
        find_hough_transform(image, houghTransform, filtsize);
    } else {
        find_hough_transform_vert(image, houghTransform, filtsize);
    }
    
    vector<int> retr;
    vector<int> reta;

    find_n_local_best_hough_lines(houghTransform, numLines, &retr, &reta);
	draw_hough_lines(image, houghTransform, retr, reta);
	
    if (argc > 6){
        char* outName = argv[6];
        imwrite(outName, image);
    }
    if (argc > 7){
        char* houghName = argv[7];
        float maxH;
        maxH = get_max_float(houghTransform);
        houghTransform = houghTransform * 255.0 /maxH;
        imwrite(houghName, houghTransform);
    }
}