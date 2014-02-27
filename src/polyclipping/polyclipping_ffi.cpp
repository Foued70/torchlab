/* 
  Polyclipping ffi so that I can use polyclipping for polygons-with-holes intersection
  testing
*/

/* extern "C" makes the functions callable from C */
extern "C"
{
#include "TH.h"
}
#include <iostream>
#include <math.h>
#include "clipper.hpp"
 
using namespace std;
using namespace ClipperLib;

extern "C"
{


/* 
	Given input of contour points and segments as output by opencv contours
	convert to clipper paths 
*/
Paths clipper_paths th_contours_to_paths( THIntTensor contours, THIntTensor seg_inds ) {
	


}

/* Test to see if point in contour */
void point_in_polygon( THIntTensor* th_mask ) {

	cout << "points size: [" << th_mask->size[0] << ", " << th_mask->size[1] << "]" << endl;	

	int height = th_mask->size[0];
	int width = th_mask->size[1];

	THIntTensor_fill( th_mask, 0 );
	int* mask = THIntTensor_data( th_mask );

	Paths subj(2);

	// DEBUG	
	//define outer blue 'subject' polygon
	subj[0].push_back(IntPoint(180,200));
	subj[0].push_back(IntPoint(260,200));
	subj[0].push_back(IntPoint(260,150));
	subj[0].push_back(IntPoint(180,150));
	
	//define subject's inner triangular 'hole' (with reverse orientation)
	subj[1].push_back(IntPoint(215,160));
	subj[1].push_back(IntPoint(230,190));
	subj[1].push_back(IntPoint(200,190));

	// For each point check if its inside the outer contour and outside of the holes
	int in_outer;
	int in_hole; 
	for ( int i=0; i < height; i++ ) { 
		for ( int j=0; j < width; j++ ) {
			// First check if point is in outer contour
			in_outer = PointInPolygon( IntPoint(j, i), subj[0] );						
			if ( in_outer == 0 )
				continue;
			// If so, check that its not in any of the inner contours			
			in_hole = false;
			for ( int k=1; k < subj.size(); k++ ) {
				if ( PointInPolygon( IntPoint(j,i), subj[k] ) != 0 ) {
					in_hole = true;
					break;
				}
			}
			if ( !in_hole )
				mask[i*width +j] = in_outer;
		}
	}	
	
	//int in_polygon = PointInPolygon( IntPoint(point[0], point[1]), subj[0] );
	
	//cout << "in_polygon?: " << in_polygon << endl;


	
	//define orange 'clipping' polygon
	/*
	clip[0].push_back(IntPoint(190,210));
	clip[0].push_back(IntPoint(240,210));
	clip[0].push_back(IntPoint(240,130));
	clip[0].push_back(IntPoint(190,130));
	
	//DrawPolygons(subj, 0x160000FF, 0x600000FF); //blue
	//DrawPolygons(clip, 0x20FFFF00, 0x30FF0000); //orange
	
	//perform intersection ...
	Clipper c;
	c.AddPaths(subj, ptSubject, true);
	c.AddPaths(clip, ptClip, true);
	c.Execute(ctIntersection, solution, pftNonZero, pftNonZero);
	*/
}

} // extern "C"
