/*
  Author  : Joshua Karges
  Date    : 25 February 2014
  Function: c++ helper function to find normals with pyramid bisections
*/

extern "C"
{
#include <math.h>
#include <limits.h>
#define MIN(x,y) ( (x) < (y) ? (x) : (y) )
#define MAX(x,y) ((x)>(y)?(x):(y))
}

#include <iostream>

extern "C"
{
int cross(double* crs, double* xyz1, double* xyz2)
{
	crs[0] = xyz1[1]*xyz2[2] - xyz1[2]*xyz2[1];
	crs[1] = xyz1[2]*xyz2[0] - xyz1[0]*xyz2[2];
	crs[2] = xyz1[0]*xyz2[1] - xyz1[1]*xyz2[0];		

}

double norm(double nx, double ny, double nz)
{
  return sqrt(pow(nx,2) + pow(ny,2) + pow(nz,2));
}

double distance(double nx1, double ny1, double nz1,
                double nx2, double ny2, double nz2)
{
  return norm(nx1-nx2,ny1-ny2,nz1-nz2);
}

int add3(double * sum, double * xyz1, double *xyz2)
{
  sum[0] = xyz1[0] + xyz2[0];
  sum[1] = xyz1[1] + xyz2[1];
  sum[2] = xyz1[2] + xyz2[2];
}

int subtract3(double * res, double * xyz1, double *xyz2)
{
  res[0] = xyz1[0] - xyz2[0];
  res[1] = xyz1[1] - xyz2[1];
  res[2] = xyz1[2] - xyz2[2];
}

double norm3(double* xyz)
{
  return norm(xyz[0],xyz[1],xyz[2]);
}

double distance3(double * xyz1, double * xyz2)
{
  return distance(xyz1[0],xyz1[1],xyz1[2], xyz2[0],xyz2[1],xyz2[2]);
}

int unit3(double* xyz)
{
	double n = norm3(xyz);
	if(n!=0){
		xyz[0] /= n;
		xyz[1] /= n;
		xyz[2] /= n;
	}
}

int pyramid_direction(double* dir, double* apex, double* base, int num_base)
{
	//the direction that the apex of a pyramid points is equal to the average of the unit vectors given by the normalized cross product of each triangle side
	int i;
	double *crs, *leg1, *leg2;

	crs = (double *) malloc(3*sizeof(double));
	leg1 = (double *) malloc(3*sizeof(double));
	leg2 = (double *) malloc(3*sizeof(double));

	subtract3(leg1, &base[(num_base-1)*3], apex); //start with the last leg
	for( i=0; i<num_base; i++ )
	{
		if(i%2 == 0){ //for memory efficiency purposes we switch back and forth between using leg1 and leg2 as the "next" leg
			subtract3(leg2, &base[i*3], apex);
		}else{
			subtract3(leg1, apex, &base[i*3]); //when it's the next leg, we'll point it the other way
		}
		cross(crs,leg1,leg2);
		unit3(crs);
		add3(dir,dir,crs);//accumulate the cross products of the pyramid triangles into dir
	}
	unit3(dir); //normalize it

	free(crs);
	free(leg1);
	free(leg2);
}

int get_normal_map(double* normal_map, double* xyz_map, char* valid_mask, int height, int width)
{
	//imagine each point is the apex of a pyramid where its base vertices are its neighbors (up,down,left,right) in the map
	int h,w,k,lind; //map indices
	int img_area = height*width;
	const int num_base = 4; //4 vertices to the pyramid base
	double *apex, *norm_hw; //vectors of current, previous, and next point
	double *base_vertices;

	norm_hw  = (double *) malloc(3*sizeof(double));
	apex  = (double *) malloc(3*sizeof(double));

	base_vertices = (double *) malloc(3*num_base*sizeof(double));

	for( h=0; h<height; h++ )
	{
		for( w=0; w<width; w++ )
		{
			//when finding the neighbors beware of out-of-bounds indices, and invalid points
			for( k=0; k<3; k++ )
			{
				lind = k*img_area + h*width + w;
				apex[k] = xyz_map[lind];
				
				//up vertex
				if( ((h+1)<height) && valid_mask[(h+1)*width+w]){
					base_vertices[0*3 + k] = xyz_map[lind+width];
				}else{
					base_vertices[0*3 + k] = xyz_map[lind];
				}
				
				//down vertex
				if( (h>0) && valid_mask[(h-1)*width+w]){
					base_vertices[1*3 + k] = xyz_map[lind-width];
				}else{
					base_vertices[1*3 + k] = xyz_map[lind];
				}

				//right vertex
				if( ((w+1)<width) && valid_mask[h*width+w+1]){
					base_vertices[2*3 + k] = xyz_map[lind+1];
				}else{
					base_vertices[2*3 + k] = xyz_map[lind];	
				}

				//left vertex
				if( (w>0) && valid_mask[h*width+w-1]){
					base_vertices[3*3 + k] = xyz_map[lind - 1];
				}else{
					base_vertices[3*3 + k] = xyz_map[lind];
				}
			}
			pyramid_direction(norm_hw, apex, base_vertices, num_base);

			for( k=0; k<3; k++ )
			{
				normal_map[k*height*width + h*width + w] = norm_hw[k];
			}
		}
	}
	free(apex)
	free(norm_hw)
	free(base_vertices)
}

}//extern "C"