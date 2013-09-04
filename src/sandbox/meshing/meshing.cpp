#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/foreach.hpp>

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
	if(p1.x == p2.x && p1.y == p2.y)
		return 0;

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
            
            if(graystore.at<uchar>(y, x) == 0)
            {
            	score -= 155;
            }
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

cv::Point cvPointFromBoostPoint(boost_point b)
{
	return cv::Point(b.x(), b.y());
}

int lineDistance(cv::Point p1, cv::Point p2)
{
    return (int) sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
}

int tests = 0;

float scoreQuad(cv::Point p1, cv::Point p2, cv::Point p3, cv::Point p4)
{
	tests++;
    return (float)(scoreLine(p1, p2) + scoreLine(p2, p3) + scoreLine(p3, p4) + scoreLine(p4, p1));// / (float)(lineDistance(p1, p2) + lineDistance(p2, p3) + lineDistance(p3, p4) + lineDistance(p4, p1));
}

void drawPoly(polygon poly, Mat img, Scalar color)
{
	std::vector<boost_point> const& points = poly.outer(); 
										
	if(points.size()>3)
	{
		for (int i = 0; i < points.size() -1; ++i) 
		{ 
			line( img, cvPointFromBoostPoint(points[i]), cvPointFromBoostPoint(points[i+1]), color, 2, CV_AA);
			//cout<<cvPointFromBoostPoint(points[i]);
		} 

		if(cvPointFromBoostPoint(points[points.size()-1]) != cvPointFromBoostPoint(points[0]))
		{
			line( img, cvPointFromBoostPoint(points[points.size()-1]), cvPointFromBoostPoint(points[0]), color, 2, CV_AA);
			//cout<<cvPointFromBoostPoint(points[points.size()-1])<<endl;
		}
	}
}

float scorePoly(polygon p)
{
	float poly_score = 0;
	std::vector<boost_point> const& points = p.outer(); 
	for (int i = 0; i < points.size() -1; ++i) 
	{ 
		poly_score += scoreLine(cvPointFromBoostPoint(points[i]), cvPointFromBoostPoint(points[i+1]));
	} 
	
	if(cvPointFromBoostPoint(points[points.size()-1]) != cvPointFromBoostPoint(points[0]))
		poly_score += scoreLine(cvPointFromBoostPoint(points[points.size()-1]), cvPointFromBoostPoint(points[0]));
		
	return poly_score;
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
    Mat m_lines_bak = m.clone();
    
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
    
    //cout<<lines.size()<<endl;
    
    
    //distanceTransform(vert, vert, CV_DIST_C, 5);
    //cv::dilate(vert, vert, element);
    //cv::dilate(hori, hori, element);
    
    GaussianBlur(graystore, graystore, cv::Size(3, 3), 1, 1);
    //GaussianBlur(hori, hori, cv::Size(1, 2*kernel_size+1), 1.2, 1.2);
    
    //graystore = graystore * 10;
    
    for( size_t i = 0; i < vertlines.size(); i++ )
    {
        Vec4i vl = vertlines[i];
        
        for( size_t j = 0; j < horizlines.size(); j++ )
        {
            Vec4i hl = horizlines[j];
            
            cv::Point intersection = findIntersection(cv::Point(vl[0], vl[1]), cv::Point(vl[2], vl[3]), cv::Point(hl[0], hl[1]), cv::Point(hl[2], hl[3]));
            
            const int dist_int = 150;
            
            if(lineDistance(cv::Point(vl[0], vl[1]), intersection) < dist_int || lineDistance(cv::Point(vl[2], vl[3]), intersection) < dist_int || lineDistance(cv::Point(hl[0], hl[1]), intersection) < dist_int || lineDistance(cv::Point(hl[2], hl[3]), intersection) < dist_int)
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
    const int search_window = 4;
    
    polygon room_poly;
    
    int img_num = 0;
	
	std::map<float, polygon> quads;

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
				//printf("candidate sizes - %ld, %ld, %ld, %ld\n", p1v.size(), p2v.size(), p3v.size(), p4v.size());
			
				for( size_t p = 0; p < p1v.size(); p++ )
				{
					for( size_t q = 0; q < p2v.size(); q++ )
					{
						for( size_t r = 0; r < p3v.size(); r++ )
						{
							for( size_t s = 0; s < p4v.size(); s++ )
							{
							
								float score = scoreQuad(p1v[p], p2v[q], p3v[r], p4v[s]);
								
								polygon quad;
									
								quad.clear();
								
								boost::geometry::append(  quad, boost_point(p1v[p].x, p1v[p].y));
								boost::geometry::append(  quad, boost_point(p2v[q].x, p2v[q].y));
								boost::geometry::append(  quad, boost_point(p3v[r].x, p3v[r].y));
								boost::geometry::append(  quad, boost_point(p4v[s].x, p4v[s].y));
								
								boost::geometry::correct(quad);
								
								quads.insert(std::pair<float, polygon>(score,quad));
								
// 								std::vector<boost_point> const& points2 = quad.outer(); 
// 											
// 								if(points2.size()>3)
// 								{
// 									for (int i = 0; i < points2.size() -1; ++i) 
// 									{ 
// 										line( m_lines, cvPointFromBoostPoint(points2[i]), cvPointFromBoostPoint(points2[i+1]), cv::Scalar(255,0,255), 2, CV_AA);
// 										cout<<cvPointFromBoostPoint(points2[i]);
// 									} 
// 
// 									line( m_lines, cvPointFromBoostPoint(points2[points2.size()-1]), cvPointFromBoostPoint(points2[0]), cv::Scalar(255, 0,255), 2, CV_AA);
// 									cout<<cvPointFromBoostPoint(points2[points2.size()-1])<<endl;
// 								}
												
// 								if(score > max_score)
// 								{
// 									//cout<<max_score<<endl;
// 
// 									max_score = score;
// 									selectedQuad[0] = p1v[p];
// 									selectedQuad[1] = p2v[q];
// 									selectedQuad[2] = p3v[r];
// 									selectedQuad[3] = p4v[s];
// 									
// 								}
								//cout<<score;

						
							}
						}
					}
				}
			}
		}
    }
    
    std::map<float, polygon>::reverse_iterator it = quads.rbegin();
    
    float prev_max_poly_score = 0;
    float max_poly_score = -1;
    int count = 0;
    
  	for (it=quads.rbegin(); it!=quads.rend(); ++it)
  	{
    	//std::cout << it->first<<endl;// << " => " << it->second << '\n';
    	
    	std::vector<polygon> poly_union_results;
		poly_union_results.clear();
		
// 		m_lines = m_lines_bak.clone();
// 		drawPoly(it->second, m_lines, cv::Scalar(0, 0 ,255)); // * (float)(quads.size()-count++)/quads.size())
// 		imshow("detected quads", m_lines);
// 		waitKey();

	
		if (! boost::geometry::intersects(room_poly) &&  ! boost::geometry::intersects(it->second)) 
		{ 
			boost::geometry::union_(room_poly, it->second, poly_union_results);
		} 
		// else
// 		{
// 			cout<<"self intersection error!"<<endl;
// 		}
		
		
		int max_poly_count = -1;
	
		int poly_count = 0;
	
		BOOST_FOREACH(polygon const& p, poly_union_results)
		{
			float poly_score = scorePoly(p);
			
// 			m_lines = m_lines_bak.clone();
// 			drawPoly(p, m_lines, cv::Scalar(255, 0 ,0));
// 			imshow("detected quads", m_lines);
// 			waitKey();
			
			if(poly_score > max_poly_score)// && boost::geometry::area(p) > boost::geometry::area(room_poly))
			{
				//cout<<"poly_score: "<<poly_score<<"  max_poly_score: "<<max_poly_score<<"  max_poly_count: "<<max_poly_count<<endl<<endl;
				cout<<"Delta :"<<poly_score - max_poly_score<<endl;
				max_poly_score = poly_score;
				max_poly_count = poly_count;
				room_poly = poly_union_results[max_poly_count];
			}
			
			poly_count++;
		}
		
		boost::geometry::correct(room_poly);
    	
//     	m_lines = m_lines_bak.clone();
// 		drawPoly(room_poly, m_lines, cv::Scalar(0, 255 ,0));
// 		imshow("detected quads", m_lines);
// 		waitKey();
    }
    drawPoly(room_poly, m_lines, cv::Scalar(0, 255 ,0));
    imshow("detected quads", m_lines);
	waitKey();
    
    								//if(score > 10)
								/*{
									
									
									// std::vector<Point> quad_points;
// 									quad_points.push_back(boost_point(p1v[p].x, p1v[p].y));
// 									quad_points.push_back(boost_point(p2v[q].x, p2v[q].y));
// 									quad_points.push_back(boost_point(p3v[r].x, p3v[r].y));
// 									quad_points.push_back(boost_point(p4v[s].x, p4v[s].y));
// 									quad.set(quad_points.begin(), quad_points.end());
									
									std::vector<polygon> poly_union_results;
									poly_union_results.clear();
									
									//cout<<room_poly.outer().size()<<" -- " << quad.outer().size() <<endl;
									
									if (! boost::geometry::intersects(room_poly) &&  ! boost::geometry::intersects(quad)) 
									{ 
										boost::geometry::union_(room_poly, quad, poly_union_results);
									} 
									
									float max_poly_score = -1;
									int max_poly_count = -1;
									
									int poly_count = 0;
									
									BOOST_FOREACH(polygon const& p, poly_union_results)
									{
										//std::cout <<  boost::geometry::area(p) << std::endl;
										
										//score poly
											
										std::vector<boost_point> const& points2 = p.outer(); 
											
											if(points2.size()>3)
											{
												for (int i = 0; i < points2.size() -1; ++i) 
												{ 
													line( m_lines, cvPointFromBoostPoint(points2[i]), cvPointFromBoostPoint(points2[i+1]), cv::Scalar(255,0,255), 2, CV_AA);
													cout<<cvPointFromBoostPoint(points2[i]);
												} 
			
												line( m_lines, cvPointFromBoostPoint(points2[points2.size()-1]), cvPointFromBoostPoint(points2[0]), cv::Scalar(255, 0,255), 2, CV_AA);
												cout<<cvPointFromBoostPoint(points2[points2.size()-1])<<endl;
											}
											
																						imshow("detected quads", m_lines);

										
										if(poly_score > max_poly_score && boost::geometry::area(p) > boost::geometry::area(room_poly))// && !boost::geometry::covered_by(boost_point(0,0), room_poly))
										{											
											cout<<"new: "<<boost::geometry::area(p)<<", old: "<<boost::geometry::area(room_poly)<<endl;
											
											std::vector<boost_point> const& points = room_poly.outer(); 
											
											if(points.size()>3)
											{
												for (int i = 0; i < points.size() -1; ++i) 
												{ 
													line( m_lines, cvPointFromBoostPoint(points[i]), cvPointFromBoostPoint(points[i+1]), cv::Scalar(0,255,255), 2, CV_AA);
												} 
			
												line( m_lines, cvPointFromBoostPoint(points[points.size()-1]), cvPointFromBoostPoint(points[0]), cv::Scalar(0,255,255), 2, CV_AA);
											}
											

											
											max_poly_score = poly_score;
											max_poly_count = poly_count;
											room_poly = poly_union_results[max_poly_count];
											
											imshow("detected quads", m_lines);
											
// 											std::stringstream ss;
// 											ss << img_num;
// 
// 											std::string save_num(ss.str());
// 	
// 											string save_filename = std::string("frames/") + save_num + std::string(".jpg");
// 											
// 											imwrite(save_filename, m_lines);
// 											img_num++;
											
											waitKey();
											m_lines = m_lines_bak.clone();

											std::vector<boost_point> const& points1 = room_poly.outer(); 

											for (int i = 0; i < points1.size() -1; ++i) 
											{ 
												line( m_lines, cvPointFromBoostPoint(points1[i]), cvPointFromBoostPoint(points1[i+1]), cv::Scalar(0,255,255), 2, CV_AA);
											} 
			
											line( m_lines, cvPointFromBoostPoint(points1[points1.size()-1]), cvPointFromBoostPoint(points1[0]), cv::Scalar(0,255,255), 2, CV_AA);

											imshow("detected quads", m_lines);
											
// 											ss.str("");
// 											ss << img_num;
// 
// 											save_num = ss.str();
// 	
// 											save_filename = std::string("frames/") + save_num + std::string(".jpg");
// 											
// 											imwrite(save_filename, m_lines);
// 											img_num++;
																						
											waitKey();
											m_lines = m_lines_bak.clone();
										}
										
										//cout<<endl;
										poly_count++;
									}

								}*/
    
// 	std::vector<boost_point> const& points = room_poly.outer(); 
// 
//     for (int i = 0; i < points.size() -1; ++i) 
// 	{ 
// 		line( m_lines, cvPointFromBoostPoint(points[i]), cvPointFromBoostPoint(points[i+1]), cv::Scalar(0,255,255), 2, CV_AA);
// 	} 
// 			
// 	line( m_lines, cvPointFromBoostPoint(points[points.size()-1]), cvPointFromBoostPoint(points[0]), cv::Scalar(0,255,255), 2, CV_AA);

    
    //cout<<selectedQuad[0]<<" "<<selectedQuad[1]<<" "<<selectedQuad[2]<<" "<<selectedQuad[3]<<endl;
    
//     if(max_score > 0 )
//     {
//         line( m_lines, selectedQuad[0], selectedQuad[1], cv::Scalar(255,255,255), 2, CV_AA);
//         line( m_lines, selectedQuad[1], selectedQuad[2], cv::Scalar(255,255,255), 2, CV_AA);
//         line( m_lines, selectedQuad[2], selectedQuad[3], cv::Scalar(255,255,255), 2, CV_AA);
//         line( m_lines, selectedQuad[3], selectedQuad[0], cv::Scalar(255,255,255), 2, CV_AA);
//     }
//     
    printf("%lu\n", intersections.size());
        
    imshow("detected lines", m);
    //imshow("slice", graystore);
    imshow("detected quads", m_lines);
    
    setMouseCallback( "detected lines", onMouse, 0 );
    
    cout<<"tests: "<<tests<<endl;
    waitKey();
    
    waitKey();waitKey();waitKey();waitKey();waitKey();waitKey();waitKey();waitKey();waitKey();waitKey();
    
    return 0;
}