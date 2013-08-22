#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/io/wkt/wkt.hpp>

#include <iostream>

using namespace cv;
using namespace std;

typedef boost::geometry::model::d2::point_xy<double> boost_point;
typedef boost::geometry::model::polygon<boost_point > polygon;


void help()
{
    cout << "\nThis program demonstrates line finding with the Hough transform.\n"
    "Usage:\n"
    "./houghlines <image_name>, Default is pic1.jpg\n" << endl;
}

Mat resultImg;
Mat graystore;
cv::Point selectedQuad[4];


float scoreLine(cv::Point p1, cv::Point p2)
{
    float score = 0;
    
    int T = p2.x - p1.x;
    int U = p1.x;
    int V = p2.y - p1.y;
    int W = p1.y;
    
    float t = 0.0;
    
    int steps = 100;
    
    for (int i = 0; i<steps; i++) {
        
        int x = (int) T*t + U;
        int y = (int) V*t + W;
        
        if(x > 0 && y > 0 && y < graystore.cols && x < graystore.rows )
        {
            score += graystore.at<uchar>(y, x);
        }
        
        t = t + 1.0/steps;
    }
    
    return score;
}

cv::Point findIntersection(cv::Point p1, cv::Point p2, cv::Point q1, cv::Point q2)
{
    cv::Point i(0,0);
    
    float d = (p1.x-p2.x)*(q1.y-q2.y) - (p1.y-p2.y)*(q1.x-q2.x);
    if (d == 0)
        return i;
    
    i.x = ((q1.x-q2.x)*(p1.x*p2.y-p1.y*p2.x)-(p1.x-p2.x)*(q1.x*q2.y-q1.y*q2.x))/d;
    i.y = ((q1.y-q2.y)*(p1.x*p2.y-p1.y*p2.x)-(p1.y-p2.y)*(q1.x*q2.y-q1.y*q2.x))/d;
    
    //float tx = (i.x - q1.x) /(q1.x - q2.x);
    
    //if(tx > 0)
    {
        //return cv::Point(0,0);
    }
    
    //
    
    return i;
}

int lineDistance(cv::Point p1, cv::Point p2)
{
    return (int) sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
}


float scoreQuad(cv::Point p1, cv::Point p2, cv::Point p3, cv::Point p4)
{
    return (float)(scoreLine(p1, p2) + scoreLine(p2, p3) + scoreLine(p3, p4) + scoreLine(p4, p1));// / (float)(lineDistance(p1, p2) + lineDistance(p2, p3) + lineDistance(p3, p4) + lineDistance(p4, p1));
}

Point _p1;
Point _p2;
int pointcounter = 0;

static void onMouse( int event, int x, int y, int, void* )
{
    if( event != EVENT_LBUTTONDOWN )
        return;
    
    Point seed = Point(x,y);
    
//     if (pointcounter == 0) {
//         _p1 = seed;
//         pointcounter++;
//     }
//     else if(pointcounter == 1)
//     {
//         _p2 = seed;
//         pointcounter = 0;
//         drawLine(_p1, _p2);
//     }
//     
    printf("%d, %d\n", seed.x, seed.y);
}

int main(int argc, char** argv)
{   
    const char* filename = argc >= 2 ? argv[1] : "slice1.png";// "pic1_small.jpg";
    
    Mat src = imread(filename, CV_LOAD_IMAGE_COLOR);
    if(src.empty())
    {
        help();
        cout << "can not open " << filename << endl;
        return -1;
    }
    
    resultImg = src.clone();
    
    Mat dst, cdst, gray, m;
    cvtColor(src, gray, CV_RGB2GRAY);
    //Canny(gray, dst, 50, 200, 3);
    dst = gray.clone();
    cvtColor(dst, cdst, CV_GRAY2BGR);
    
    
    int erosion_size = 1;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
                                                cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                                                cv::Point(erosion_size, erosion_size) );
    
    //cv::dilate(gray, gray, element);
    //cv::erode(gray, gray, element);
    //cv::dilate(gray, gray, element);
    
    graystore = gray.clone();
    //graystore = cv::Scalar(0);
        
    cvtColor(gray, m, CV_GRAY2RGBA);
    
    Mat m_lines = m.clone();
    
    Mat vert = gray.clone();
    Mat hori = gray.clone();
    
    vert = cv::Scalar(0);
    hori = cv::Scalar(0);
    
    vector<Vec4i> lines;
    vector<Vec4i> vertlines;
    vector<Vec4i> horizlines;
    vector<Point> intersections;
    
    intersections.clear();
    
    //GaussianBlur(gray, gray, cv::Size(5, 5), 2.5, 2.5);
    //gray = gray * 10;

    
    HoughLinesP(gray, lines, 1, CV_PI/360, 15, 25, 80 );
    
    for( size_t i = 0; i < lines.size(); i++ )
    {
        Vec4i l = lines[i];
        if(abs(l[0]-l[2]) < 10)  //for verticals only
        {
            vertlines.push_back(l);
            line( m, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,255,0), 2, CV_AA);
            //line( graystore, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255), 2);
            intersections.push_back(cv::Point(l[0], l[1]));
            intersections.push_back(cv::Point(l[2], l[3]));

        }
        if(abs(l[1]-l[3]) < 10)  //for horizontals
        {
            horizlines.push_back(l);
            line( m, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 2, CV_AA);
            //line( graystore, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255), 2);
            intersections.push_back(cv::Point(l[0], l[1]));
            intersections.push_back(cv::Point(l[2], l[3]));
        }
    }
    
    cout<<lines.size()<<endl;
    
    
    //distanceTransform(vert, vert, CV_DIST_C, 5);
    //cv::dilate(vert, vert, element);
    //cv::dilate(hori, hori, element);
    
    GaussianBlur(graystore, graystore, cv::Size(11, 11), 5.0, 5.0);
    //GaussianBlur(hori, hori, cv::Size(1, 2*kernel_size+1), 1.2, 1.2);
    
    graystore = graystore * 10;
    
    for( size_t i = 0; i < vertlines.size(); i++ )
    {
        Vec4i vl = vertlines[i];
        
        for( size_t j = 0; j < horizlines.size(); j++ )
        {
            Vec4i hl = horizlines[j];
            
            cv::Point intersection = findIntersection(cv::Point(vl[0], vl[1]), cv::Point(vl[2], vl[3]), cv::Point(hl[0], hl[1]), cv::Point(hl[2], hl[3]));
            
            const int dist_int = 30;
            
            //if(lineDistance(cv::Point(vl[0], vl[1]), intersection) < dist_int || lineDistance(cv::Point(vl[2], vl[3]), intersection) < dist_int || lineDistance(cv::Point(hl[0], hl[1]), intersection) < dist_int || lineDistance(cv::Point(hl[2], hl[3]), intersection) < dist_int)
            {
                int mindist = 9999;
                for( size_t i = 0; i < intersections.size(); i++ )
                {
                    int dist = lineDistance(intersection, intersections[i]);
                    if( dist < mindist)
                    {
                        mindist = dist;
                    }
                }
                
                if(mindist > 5)
                {
                    intersections.push_back(intersection);
                }
            }
        }
        
    }
    
    for(int i =0; i< intersections.size(); i++)
    {
        circle(m, intersections[i], 10, cv::Scalar(255,0,0));
    }
    
    //center
    circle(m, Point(m.cols/2, m.rows/2), 5, cv::Scalar(255,255,255));
    
    float max_score = 0;
    int max_p = -1;
    int max_q = -1;
    int max_r = -1;
    int max_s = -1;
    vector<cv::Point> p1v, p2v, p3v, p4v;
    const int search_window = 5;
    
    std::vector<polygon> room_poly;

    for(int i =0; i< intersections.size(); i++)
    {
    	cv::Point strokeStart = intersections[i];
    	
		for(int j =0; j< intersections.size(); j++)
		{
			cv::Point strokeEnd = intersections[j];
			
			if(i == j || lineDistance(intersections[i], intersections[j]) < search_window || abs(strokeStart.x-strokeEnd.x)*abs(strokeStart.y-strokeEnd.y) < 10000)
				continue;
				
			p1v.clear(); p2v.clear(); p3v.clear(); p4v.clear();
			
			for( size_t k = 0; k < intersections.size(); k++ )
			{
				
				if (lineDistance(intersections[k], strokeStart) < search_window)
					p1v.push_back(intersections[k]);
				else if(lineDistance(intersections[k], cv::Point(strokeEnd.x, strokeStart.y)) < search_window)
					p2v.push_back(intersections[k]);
				else if(lineDistance(intersections[k], strokeEnd) < search_window)
					p3v.push_back(intersections[k]);
				else if(lineDistance(intersections[k], cv::Point(strokeStart.x, strokeEnd.y)) < search_window)
					p4v.push_back(intersections[k]);
	
			}
			
			if(p1v.size() > 0 && p2v.size() > 0 && p3v.size() > 0 && p4v.size() > 0)
			{
				printf("candidate sizes - %ld, %ld, %ld, %ld\n", p1v.size(), p2v.size(), p3v.size(), p4v.size());
			
				for( size_t p = 0; p < p1v.size(); p++ )
				{
					for( size_t q = 0; q < p2v.size(); q++ )
					{
						for( size_t r = 0; r < p3v.size(); r++ )
						{
							for( size_t s = 0; s < p4v.size(); s++ )
							{
							
								float score = scoreQuad(p1v[p], p2v[q], p3v[r], p4v[s]);
												
								if(score > max_score)
								{
									cout<<max_score<<endl;

									max_score = score;
									selectedQuad[0] = p1v[p];
									selectedQuad[1] = p2v[q];
									selectedQuad[2] = p3v[r];
									selectedQuad[3] = p4v[s];
									
								}
								
								if(score > 5000)
								{
									line( m_lines, p1v[p], p2v[q], cv::Scalar(255,0,255), 2, CV_AA);
									line( m_lines, p2v[q], p3v[r], cv::Scalar(255,0,255), 2, CV_AA);
									line( m_lines, p3v[r], p4v[s], cv::Scalar(255,0,255), 2, CV_AA);
									line( m_lines, p4v[s], p1v[p], cv::Scalar(255,0,255), 2, CV_AA);
									
									polygon quad;
									std::vector<Point> quad_points;
									quad_points.push_back(boost_point(p1v[p].x, p1v[p].y);
									quad_points.push_back(boost_point(p2v[q].x, p2v[q].y);
									quad_points.push_back(boost_point(p3v[r].x, p3v[r].y);
									quad_points.push_back(boost_point(p4v[s].x, p4v[s].y);
									quad.set(quad_points.begin(), quad_points.end());
									
									room_poly
								}
						
							}
						}
					}
				}
			}
		}
    }

    
    
    cout<<selectedQuad[0]<<" "<<selectedQuad[1]<<" "<<selectedQuad[2]<<" "<<selectedQuad[3]<<endl;
    
//     if(max_score > 0 )
//     {
//         line( m, selectedQuad[0], selectedQuad[1], cv::Scalar(255,0,255), 2, CV_AA);
//         line( m, selectedQuad[1], selectedQuad[2], cv::Scalar(255,0,255), 2, CV_AA);
//         line( m, selectedQuad[2], selectedQuad[3], cv::Scalar(255,0,255), 2, CV_AA);
//         line( m, selectedQuad[3], selectedQuad[0], cv::Scalar(255,0,255), 2, CV_AA);
//     }
//     
    printf("%lu\n", intersections.size());
        
    imshow("detected lines", m);
    imshow("slice", graystore);
    imshow("detected quads", m_lines);
    
    setMouseCallback( "detected lines", onMouse, 0 );
    
    waitKey();
    
    return 0;
}