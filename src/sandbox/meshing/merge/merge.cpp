#include <iostream>
#include <vector>
#include <fstream>
#include <cstdlib>


#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/io/wkt/wkt.hpp>
#include <boost/foreach.hpp>
#include "poly2tri/poly2tri.h"


using namespace std;

typedef p2t::Point p2t_point;
typedef p2t::Triangle p2t_triangle;
typedef p2t::CDT p2t_CDT;
typedef boost::geometry::model::d2::point_xy<double> boost_point;
typedef boost::geometry::model::ring<boost_point> ring;
typedef boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double> > polygon;


const float scale = 0.01;

void set_epsilon(boost_point & p)
{
	using boost::geometry::get;
	using boost::geometry::set;
	set<0>(p, get<0>(p) - 0.0001);
	set<1>(p, get<1>(p) - 0.0001);
}

int main(int argc, char** argv)
{
    ofstream objfile;
    ifstream infile1;
    ifstream infile2;
    polygon slice1, slice2;
    float z_value;
    string line;
    
    vector<polygon> difference_result;
    
    if(argc < 4)
    {
    	cout<<"Usage: ./meshing slice1 [slice2] z_value obj_file"<<endl;exit(0);
    }
    
    if(argc == 4)
    {
    	infile1.open(argv[1]);
    	
    	if (infile1.is_open())
		  {
			getline (infile1,line);
			boost::geometry::read_wkt(line, slice1);

			infile1.close();
		  }
		  
		boost::geometry::correct(slice1);
		boost::geometry::unique(slice1);
		
		difference_result.push_back(slice1);
		
		z_value = atof(argv[2]);
		
		objfile.open(argv[3], std::fstream::out | std::fstream::trunc);
    }
    
    if(argc == 5)
    {
    	infile1.open(argv[1]);
    	infile2.open(argv[2]);
    	
    	if (infile1.is_open())
		  {
			getline (infile1,line);
			boost::geometry::read_wkt(line, slice1);

			infile1.close();
		  }
		  
		if (infile2.is_open())
		  {
			getline (infile2,line);
			boost::geometry::read_wkt(line, slice2);

			infile2.close();
		  }
		
		boost::geometry::correct(slice1);
		boost::geometry::correct(slice2);
		boost::geometry::unique(slice1);
		boost::geometry::unique(slice2);
		boost::geometry::for_each_point(slice2, set_epsilon);
		
		z_value = atof(argv[3]);
		
		objfile.open(argv[4], std::fstream::out | std::fstream::trunc);
		
		vector<polygon> union_result;
    	vector<polygon> intersection_result;
    	
    	boost::geometry::union_(slice1, slice2, union_result);
    	boost::geometry::intersection(slice1, slice2, intersection_result);
    	boost::geometry::difference(union_result[0], intersection_result[0], difference_result);
    }
	

//    polygon s1, s2, s3, s4;
    
//    float z12 = 37.0505, z23 = 38.6754, z34 = 38.9828;

//     boost::geometry::read_wkt(
//         "POLYGON((209.98 594.28,210.024 605.023,210.026 605.023,210.026 605.025,210.027 605.025,210.027 605.026,210.094 605.027,210.095 605.092,210.096 605.092,210.096 605.093,210.104 605.093,210.104 605.101,218.111 605.151,218.057 606.056,218.077 606.056,218.076 606.073,218.084 606.073,218.084 606.081,218.095 606.081,218.094 606.091,219.328 606.097,219.086 612.083,219.092 612.083,219.092 612.089,219.097 612.089,219.097 612.094,219.098 612.094,219.098 612.095,219.105 612.095,219.105 612.102,219.11 612.102,219.11 612.106,219.121 612.107,219.121 612.117,219.122 612.117,219.122 612.118,220.034 612.114,220.119 612.115,220.093 613.09,220.106 613.09,220.106 613.103,220.107 613.103,220.107 613.104,220.108 613.104,220.108 613.104,220.109 613.104,220.109 613.105,220.111 613.105,220.111 613.107,220.118 613.107,220.118 613.114,220.124 613.114,220.124 613.12,220.581 613.117,221.117 613.121,221.117 614.721,221.112 615.108,221.114 615.108,221.114 615.11,221.117 615.11,221.117 615.113,221.12 615.113,221.12 615.719,221.115 616.111,221.119 616.111,221.119 616.115,221.12 616.115,221.12 616.116,221.126 616.116,221.126 616.122,221.585 616.117,849.116 617.116,849.086 616.052,848.919 609.124,849.122 609.122,849.101 608.542,849.101 227.104,849.1 227.104,849.1 227.103,849.099 227.103,849.099 227.102,849.098 227.102,849.098 227.101,849.093 227.101,849.093 227.096,849.092 227.096,849.092 227.095,849.085 227.095,849.085 175.088,849.084 175.088,849.084 175.087,776.122 178.735,769.033 179.034,769.252 226.457,206.078 222.078,206.041 223.077,205.07 223.07,205.07 248.561,205.047 249.047,205.07 249.047,205.07 249.07,205.078 249.071,205.078 249.078,206.317 249.081,206.318 249.102,205.099 249.099,206.839 298.269,209.98 594.28))", s1);
// 
//     boost::geometry::read_wkt(
//         "POLYGON((678.427 620.278,678.574 620.278,678.576 620.576,745.515 620.182,849.035 620.035,849.035 610.04,850.033 610.033,849.209 233.575,862.577 233.595,862.754 221.024,862.992 221.023,863.268 215.057,864.028 215.06,864.082 213.487,865.937 193.075,866.042 193.075,866.042 156.075,866.028 156.075,866.028 156.06,866.026 156.06,866.026 156.058,866.016 156.059,866.016 156.047,866.015 156.047,866.015 156.046,866.014 156.046,866.014 156.045,866.013 156.045,866.013 156.044,866.012 156.044,866.012 156.043,866.01 156.043,866.01 156.041,865.991 156.041,865.992 156.023,865.991 156.023,865.991 156.022,865.982 156.022,865.982 156.013,865.968 156.013,865.969 155.999,865.968 155.999,865.968 155.998,865.967 155.998,865.967 155.997,865.565 156.001,865.583 155.601,865.482 155.603,865.482 155.522,865.506 155.522,865.506 138.7,865.52 138.536,865.506 138.536,865.506 138.522,865.503 138.522,865.503 138.519,865.502 138.519,865.502 138.518,194.518 139.518,194.518 139.521,194.506 139.521,194.507 139.507,194.506 139.507,194.506 139.506,194.046 139.515,144.525 138.525,144.546 148.494,143.488 148.488,143.617 152.547,142.972 152.542,138.501 152.501,138.179 161.972,137.974 161.974,137.981 162.063,137.981 163.779,137.783 173.659,136.501 211.501,136.551 211.501,136.022 224.022,136.023 224.022,136.023 224.023,144.374 223.989,145.022 374.689,145.219 468.549,145.031 527.963,144.993 527.963,145.048 528.698,145.412 559.971,145.481 592.898,145.516 609.478,145.506 614.49,145.507 614.49,145.507 614.491,145.527 614.492,145.527 614.511,145.531 614.511,145.531 614.515,145.589 614.516,145.589 614.571,145.59 614.571,145.59 614.572,145.971 614.572,145.97 614.94,146.001 614.94,146 614.97,146.041 614.97,146.066 621.034,146.068 621.034,146.068 621.035,507.433 620.521,507.441 624.428,678.428 620.428,678.427 620.278),(194.525 140.222,194.529 139.734,194.555 140.223,194.525 140.222))", s2);
//         
//     boost::geometry::read_wkt(
//         "POLYGON((848.043 154.048,848.043 154.044,214.044 154.044,215.044 491.044,215.048 491.044,215.048 491.048,215.049 491.048,215.049 491.049,215.051 491.049,215.051 491.051,216.343 491.041,847.048 487.049,847.051 486.059,848.05 486.051,848.05 154.051,848.048 154.051,848.048 154.049,848.047 154.049,848.047 154.048,848.043 154.048))", s3);
//         
//     boost::geometry::read_wkt(
//         "POLYGON((865.002 155.003,865.002 155.002,864.71 155.002,848.951 154.84,849.069 110.071,849.068 110.071,849.068 110.07,849.061 110.07,849.061 110.062,849.06 110.062,849.06 110.061,849.059 110.061,849.059 110.06,849.058 110.06,849.058 110.059,849.053 110.059,849.053 110.054,849.052 110.054,849.052 110.053,849.05 110.053,849.05 110.051,849.049 110.051,849.049 110.05,849.048 110.05,849.048 110.049,849.046 110.049,849.046 110.047,849.044 110.047,849.044 110.045,849.037 110.045,849.037 110.038,849.033 110.038,849.033 110.034,849.032 110.034,849.032 110.033,849.03 110.033,849.03 110.031,849.029 110.031,849.029 110.03,849.028 110.03,849.028 110.029,849.027 110.029,849.027 110.028,849.02 110.028,849.021 110.021,848.273 110.033,848.08 110.033,848.08 109.082,848.077 109.082,848.077 109.079,848.076 109.079,848.076 109.078,848.075 109.078,848.075 109.077,848.074 109.077,848.074 109.076,848.073 109.076,848.073 109.075,848.072 109.075,848.072 109.074,848.071 109.074,848.071 109.073,848.07 109.073,848.07 108.072,848.065 108.072,848.065 108.067,848.064 108.067,848.064 108.066,848.057 108.066,848.057 108.058,848.056 108.058,848.056 108.057,848.055 108.057,848.055 108.056,848.054 108.056,848.054 108.055,848.051 108.055,848.051 108.052,848.047 108.052,848.047 108.048,848.045 108.048,848.045 108.046,848.043 108.046,848.043 108.044,848.042 108.044,848.042 108.043,848.031 108.043,848.031 108.032,848.026 108.032,848.026 108.027,848.025 108.027,848.025 108.026,848.024 108.026,848.024 108.025,848.023 108.025,848.023 108.024,848.022 108.024,848.022 108.023,848.021 108.023,848.021 108.022,664.021 110.022,664.029 111.024,664.023 111.024,664.031 112.026,664.025 112.026,664.033 112.978,663.061 113.004,662.021 113.021,662.581 155.048,215.011 157.011,215.011 492.011,667.011 488.011,667.011 488.006,667.013 488.006,667.013 488.013,667.014 488.013,667.014 488.014,667.015 488.014,667.015 488.015,667.016 488.015,667.016 488.016,667.017 488.016,667.017 488.017,667.018 488.017,667.018 488.018,667.019 488.018,667.019 488.019,667.02 488.019,667.02 488.02,667.021 488.02,667.021 488.021,667.021 488.021,667.021 488.022,667.022 488.022,667.022 488.023,667.023 488.023,667.023 488.024,667.024 488.024,667.024 488.025,667.025 488.025,667.025 488.026,667.026 488.026,667.026 488.027,667.027 488.027,667.027 488.028,667.028 488.028,667.028 488.029,667.029 488.029,667.029 488.03,667.03 488.03,667.03 488.031,667.031 488.031,667.031 488.032,667.032 488.032,667.032 488.033,667.033 488.033,667.033 488.034,667.034 488.034,667.034 488.035,667.035 488.035,667.035 488.036,667.036 488.036,667.036 488.037,667.037 488.037,667.037 488.038,667.038 488.038,667.038 488.039,667.039 488.039,667.039 488.04,667.04 488.04,667.04 488.041,667.041 488.041,667.041 488.042,667.042 488.042,667.042 488.043,667.043 488.043,667.043 488.044,667.044 488.044,667.044 488.045,667.045 488.045,667.045 488.046,667.046 488.046,667.046 488.047,667.047 488.047,667.047 488.048,667.048 488.048,667.048 488.049,667.049 488.049,667.049 488.05,667.05 488.05,667.05 488.051,667.051 488.051,667.051 488.052,667.052 488.052,667.052 488.053,667.053 488.053,667.053 488.054,667.054 488.054,667.054 488.055,667.055 488.055,667.055 488.056,667.056 488.056,667.056 488.057,667.057 488.057,667.057 488.058,667.058 488.058,667.058 488.059,667.059 488.059,667.059 488.06,667.06 488.06,667.06 488.061,667.061 488.061,667.061 488.062,667.062 488.062,667.062 488.063,667.062 488.063,667.062 488.064,667.063 488.064,667.063 488.065,667.064 488.065,667.064 488.066,667.065 488.066,667.065 488.067,667.066 488.067,667.066 488.068,667.067 488.068,667.067 488.069,667.068 488.069,667.068 488.07,667.069 488.07,667.069 488.072,667.07 488.072,667.07 488.073,667.071 488.073,667.071 488.074,667.072 488.074,667.072 488.075,667.073 488.075,667.073 488.076,667.074 488.076,667.074 488.077,667.075 488.077,667.075 488.078,667.076 488.078,667.076 488.079,667.077 488.079,667.077 488.08,667.08 488.08,667.08 488.083,848.08 487.083,848.08 486.177,865.006 486.006,865.006 155.006,865.005 155.006,865.005 155.005,865.003 155.005,865.003 155.003,865.002 155.003))", s4);


//     vector<polygon> difference_result;
//     
//     boost::geometry::correct(s2);
//     boost::geometry::correct(s3);
//     
//     boost::geometry::for_each_point(s3, set_epsilon);
//     
//     boost::geometry::union_(s2, s3, union_result);
//     boost::geometry::intersection(s2, s3, intersection_result);
//     boost::geometry::difference(union_result[0], intersection_result[0], difference_result);
//     
//     cout<<union_result.size()<<intersection_result.size()<<difference_result.size()<<endl;
//     
//     cout<<endl<<endl<<boost::geometry::wkt(difference_result[0])<<endl<<endl;
    
    int vctr = 1;
    
    BOOST_FOREACH(polygon const& p, difference_result)
    {
    	//add outer ring
        std::vector<boost_point> const& boost_points = p.outer(); 
        vector<p2t_point*> polyline;
					
		for (int i = 0; i < boost_points.size()-1; ++i) 
		{ 
			cout<<boost_points[i].x()<<","<<boost_points[i].y()<<endl;
			polyline.push_back(new p2t_point(boost_points[i].x()*scale, boost_points[i].y() * scale * -1));
		} 
		
		p2t_CDT* p2tcdt = new p2t_CDT(polyline);
		
		//add holes
		std::vector<ring> const& inners = p.inners(); 

		for (int i = 0; i < inners.size(); ++i) 
		{
			vector<boost_point> r = inners[i];
			polyline.clear();
			
			for(int j = 0; j<r.size()-1; j++)
			{
				//cout<<r[j].x()<<","<<r[j].y()<<endl;
				polyline.push_back(new p2t_point(r[j].x()*scale, r[j].y() * scale * -1)); 
			}
			
			p2tcdt->AddHole(polyline);
		} 
		
		p2tcdt->Triangulate();
		
		vector<p2t_triangle*> p2ttriangles;
		
		p2ttriangles = p2tcdt->GetTriangles();
		
		cout << "Number of triangles = " << p2ttriangles.size() << endl;
		
		for (int i = 0; i < p2ttriangles.size(); i++) {
			p2t_triangle& t = *p2ttriangles[i];
			
			for(int j = 0; j < 3; j++)
			{
				p2t_point& a = *t.GetPoint(j);
				objfile<<"v "<<a.x<<" "<<a.y<<" "<<z_value<<endl;
			}
			
			objfile<<"f "<<vctr<<" "<<vctr+1<<" "<<vctr+2<<endl;
			vctr+=3;
		}
    }
    
	objfile.close();


    return 0;
}