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

//flattening scale
const float scale = 0.01;

//to avoid self intersection
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
    	cout<<"Usage: ./merge slice1 [slice2] z_value obj_file"<<endl;exit(0);
    }
    
    //open and load slices
    
    //single slice (floor or ceiling)
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
    
    //two slices, calculate Union - intersection and fill in horizontal surfaces
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
    
    int vctr = 1;
    
    //for each polygon in the resulting set
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