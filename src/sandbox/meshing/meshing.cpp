#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/io/wkt/wkt.hpp>
#include <boost/foreach.hpp>

#include <iostream>
#include <fstream>
#include <cstdlib>

//#define DEBUG

using namespace cv;
using namespace std;

typedef boost::geometry::model::d2::point_xy<float> boost_point;
typedef boost::geometry::model::polygon<boost_point > polygon;

const float scale = 0.01;

Mat resultImg;
Mat graystore;
cv::Point selectedQuad[4];

//calculate score for a line by looking at the value of pixels along the line
//could be vastly improved!
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
            	score -= 250;
            }
        }
        
        t = t + 1.0/steps;
    }
    
    return score;
}

//find intersection of two lines defined by two pairs of points
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

//data format conversion
cv::Point cvPointFromBoostPoint(boost_point b)
{
	return cv::Point(b.x(), b.y());
}

//add epsilon to geometry to avoid issues of self intersection
void set_epsilon(boost_point & p)
{
	using boost::geometry::get;
	using boost::geometry::set;
	set<0>(p, get<0>(p) + 0.001);
	set<1>(p, get<1>(p) + 0.001);
}

int lineDistance(cv::Point p1, cv::Point p2)
{
    return (int) sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
}

int tests = 0;

float scoreQuad(cv::Point p1, cv::Point p2, cv::Point p3, cv::Point p4)
{
	tests++;
    return (float)(scoreLine(p1, p2) + scoreLine(p2, p3) + scoreLine(p3, p4) + scoreLine(p4, p1)) / (float)(lineDistance(p1, p2) + lineDistance(p2, p3) + lineDistance(p3, p4) + lineDistance(p4, p1));
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

//write polygon to obj file
void poly_to_obj(polygon p, float z1, float z2, ofstream& file)
{
	std::vector<boost_point> const& points = p.outer(); 
	
	int vctr = 1;
										
	for (int i = 0; i < points.size() -1; ++i) 
	{ 
		file<<"v "<<points[i].x()*scale<<" "<<points[i].y()*scale*-1<<" "<<z1<<endl;
		file<<"v "<<points[i].x()*scale<<" "<<points[i].y()*scale*-1<<" "<<z2<<endl;
		file<<"v "<<points[i+1].x()*scale<<" "<<points[i+1].y()*scale*-1<<" "<<z1<<endl;
		file<<"v "<<points[i+1].x()*scale<<" "<<points[i+1].y()*scale*-1<<" "<<z2<<endl;
		
		file<<"f "<<vctr<<" "<<vctr+2<<" "<<vctr+1<<endl;
		file<<"f "<<vctr+2<<" "<<vctr+3<<" "<<vctr+1<<endl;
		
		vctr += 4;
	} 

// 	if(cvPointFromBoostPoint(points[points.size()-1]) != cvPointFromBoostPoint(points[0]))
// 	{
// 		line( img, cvPointFromBoostPoint(points[points.size()-1]), cvPointFromBoostPoint(points[0]), color, 2, CV_AA);
// 	}
}

//score polygon by scoring each line
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
     
    printf("%d, %d\n", seed.x, seed.y);
}

int main(int argc, char** argv)
{
	if(argc < 6)
    {
    	cout<<"Usage: ./meshing slice z_value_low z_value_high obj_file data_file"<<endl;exit(0);
    }   

    const char* filename = argc >= 2 ? argv[1] : "slice1.png";// "pic1_small.jpg";
    
    Mat src = imread(filename, CV_LOAD_IMAGE_COLOR);
    if(src.empty())
    {
        cout<<"Usage: ./meshing slice z_value_low z_value_high obj_file data_file"<<endl;exit(0);
        cout << "can not open " << filename << endl;
        return -1;
    }
    
    float z1 = atof(argv[2]);
    float z2 = atof(argv[3]);
    
    ofstream objfile;
    objfile.open(argv[4]);
    
    ofstream datafile;
    datafile.open(argv[5]);
    
    resultImg = src.clone();
    
    Mat dst, cdst, gray, m;
    
    cvtColor(src, gray, CV_RGB2GRAY);
    
    equalizeHist(gray, gray);
    
    
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
    Mat m_polys = m.clone();
    
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

    //detect lines and sort them into (almost) horizontal and vertical
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
    
    cout<<"Line detection done!"<<endl;   

    //distanceTransform(vert, vert, CV_DIST_C, 5);
    //cv::dilate(vert, vert, element);
    //cv::dilate(hori, hori, element);
    
    GaussianBlur(graystore, graystore, cv::Size(3, 3), 1, 1);
    //GaussianBlur(hori, hori, cv::Size(1, 2*kernel_size+1), 1.2, 1.2);
    
    //graystore = graystore * 10;
    
    // find all 'ghost' intersections that lie within dist_int of any line segment
    const int dist_int = 100;
    
    for( size_t i = 0; i < vertlines.size(); i++ )
    {
        Vec4i vl = vertlines[i];
        
        for( size_t j = 0; j < horizlines.size(); j++ )
        {
            Vec4i hl = horizlines[j];
            
            cv::Point intersection = findIntersection(cv::Point(vl[0], vl[1]), cv::Point(vl[2], vl[3]), cv::Point(hl[0], hl[1]), cv::Point(hl[2], hl[3]));
            
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
    
    cout<<"Intersection detection done! ("<<intersections.size()<<")"<<endl;   

    
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
    
    //polygon room_poly;
    std::vector<polygon> room_polys;
    std::vector<polygon> room_polys_new;
    
    int img_num = 0;
	
	std::map<float, polygon> quads;
	
	// for each pair of intersections calculate the diagonal stroke formed by those two points
	// treat the diagonal as the diagonal of a quad and search the opposite diagonal for 
	// potential end points (within search_window pixels). If found, add the quad to the list of quads
	
	const int search_window = 5;

    for(int i =0; i< intersections.size(); i++)
    {
    	cv::Point strokeStart = intersections[i];
    	
		for(int j =0; j< intersections.size(); j++)
		{
			cv::Point strokeEnd = intersections[j];
			
			// if diagonal is between the same points or the diagonal is too small, discard
			if(i == j || lineDistance(intersections[i], intersections[j]) < search_window || abs(strokeStart.x-strokeEnd.x)*abs(strokeStart.y-strokeEnd.y) < 10000)
				continue;
				
			p1v.clear(); p2v.clear(); p3v.clear(); p4v.clear();
			
			// find intersections that complete the quad (there may be several in the search_window
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
			
			// for each quad, calculate the score and add it to the map of quads, sorted by score
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
								
								drawPoly(quad, m_polys, cv::Scalar(255, 0 ,255));
						
							}
						}
					}
				}
			}
		}
    }
    
    cout<<"Quad detection done!"<<endl;   
    
    std::map<float, polygon>::reverse_iterator it = quads.rbegin();
    
    float prev_max_poly_score = 0;
    float max_poly_score = -1;
    int count = 0;
    
    //iterate through quads with descending score (max first)
  	for (it=quads.rbegin(); it!=quads.rend(); ++it)
  	{   
  		// in the beginning populate the model with the first quad 	
    	if(count == 0)
		{
		  room_polys.push_back(it->second);
		  //continue;// = poly_union_results[0];
		}
    	
    	int max_intermediate_poly_score = -1;


#ifdef DEBUG
		m_lines = m_lines_bak.clone();
		drawPoly(it->second, m_lines, cv::Scalar(0, 0 ,255)); // * (float)(quads.size()-count++)/quads.size())
		imshow("detected quads", m_lines);
		waitKey();
#endif
		//there can be several possible non-intersection quads that describe a cloud, loop through
		//each one and add the current quad. if the score improves, regard this as a new room polygon
		//else ignore. If there is no intersection, create a new room polygon and add to the set
    	BOOST_FOREACH(polygon &room_poly, room_polys)
			{
		
				std::vector<polygon> poly_union_results;
				poly_union_results.clear();
		
				//boost::geometry::correct(it->second);
				//boost::geometry::correct(room_poly);
				
				//to prevent self intersection
				boost::geometry::for_each_point(room_poly, set_epsilon);
			
				//cout<<boost::geometry::intersects(room_poly)<<boost::geometry::intersects(it->second)<<endl;
			
				if(!boost::geometry::intersects(room_poly))
				{
					 boost::geometry::union_(room_poly, it->second, poly_union_results);
				}
				else
				{
					m_lines = m_lines_bak.clone();
					drawPoly(room_poly, m_lines, cv::Scalar(0, 255 ,255)); // * (float)(quads.size()-count++)/quads.size())
					imshow("detected quads", m_lines);
					waitKey();
				
				}
			
				int max_poly_count = -1;
	
				int poly_count = 0;
			
				bool intersects = false;
				
				//if union results in only 1 result (intersection occured)
				if(poly_union_results.size() == 1)
				{
					polygon p = poly_union_results[0];
					float poly_score = scorePoly(p);
				
					if(poly_score > max_poly_score)	// if score improves, add poly to new hypothesis set
						{
							max_poly_score = poly_score;
							room_polys_new.push_back(p);
						}
				}
				else if (poly_union_results.size() == 2) //no intersection, hence add both polys to new set
				{
					room_polys_new.push_back(poly_union_results[0]);
					room_polys_new.push_back(poly_union_results[1]);
				}
		
				count++;
			}
			
			//update room polys with new hypotheses after adding current quad
			if(room_polys_new.size() > 0)
			{
				room_polys.clear();
	
				room_polys = room_polys_new;
				max_poly_score = max_intermediate_poly_score;
	
				room_polys_new.clear();
			}
			
			//cout<<room_polys.size()<<endl;
			
#ifdef DEBUG
			m_lines = m_lines_bak.clone();

			BOOST_FOREACH(polygon &room_poly, room_polys)
			{
				drawPoly(room_poly, m_lines, cv::Scalar(0, 255 ,0));
			}
			imshow("detected quads", m_lines);
			waitKey();
#endif
    }
    
    srand (time(NULL));
    
    m_lines = m_lines_bak.clone();
    
    //draw/write/render result
    BOOST_FOREACH(polygon &room_poly, room_polys)
	{
		int r = 255*((double) rand() / (RAND_MAX));
		int g = 255*((double) rand() / (RAND_MAX));
		int b = 255*((double) rand() / (RAND_MAX));
		drawPoly(room_poly, m_lines, cv::Scalar(b, g, r));
		poly_to_obj(room_poly, z1, z2, objfile);
		objfile.close();
		//cout<<endl<<endl<<boost::geometry::wkt(room_poly)<<endl<<endl;
		datafile<<boost::geometry::wkt(room_poly);
		datafile.close();
		printf("%d, %d, %d\n", r, g, b);
		imshow("detected quads", m_lines);
		waitKey();
	}
    
    printf("Intersections: %lu\n", intersections.size());
        
    imshow("detected lines", m);
    imshow("slice", m_polys);
    
    setMouseCallback( "detected lines", onMouse, 0 );
    
    cout<<"Tests: "<<tests<<endl;
    waitKey();
    
#ifdef DEBUG
    waitKey();waitKey();waitKey();waitKey();waitKey();waitKey();
#endif
    
    return 0;
}