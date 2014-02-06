/* 
  CPlane Functions
    A set of simple functions to operate on our single scans 
*/

/* extern "C" makes the functions callable from C */
extern "C"
{
#include "TH.h"
}
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/registration/transformation_validation_euclidean.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/surface/gp3.h>
 
using namespace std;
/*
using namespace pcl;
using namespace registration;

typedef PointXYZ PointType;
*/

//#define DEBUG

extern "C"
{

// A quick sum test, just for sanity 
void map2xyz( THDoubleTensor* xyz_map, THDoubleTensor* result ) { 

  // 3d Matrix 
  long stack = xyz_map->size[0];
  long height = xyz_map->size[1];
  long width = xyz_map->size[2];
  double *data =  THDoubleTensor_data( xyz_map );

  // DEBUG printing 
  printf("size[0]: %i\n", stack);
  printf("size[1]: %i\n", height);
  printf("size[2]: %i\n", width);

  // Ensure that the result is of the correct size 
  THDoubleTensor_resize2d( result, height, width );
  double* result_data = THDoubleTensor_data( result );

  // Apply operation to all elements in map 
  long data_index;
  long result_index;
  for ( long i=0; i<height; ++i ) {
    for ( long j=0; j<width; ++j ) {

      result_index = i*width + j;
      result_data[result_index] = data[0*height*width + result_index] + data[1*height*width + result_index] + data[2*height*width + result_index];
    }
  }
}

/*
  Classify each point as planar, invalid or non-planar using a small window of nearby points
*/
void classifyPoints( THDoubleTensor* xyz_map, int window, double dist_thresh, double plane_thresh, THDoubleTensor* classification, THDoubleTensor* errors, THDoubleTensor* normals ) { 

  // Compute halfwidth of the window ... windows should be odd 
  int window_width = (window - 1)/2;
  printf("window_width: %d \n", window_width);

  // 3d Matrix 
  long stack = xyz_map->size[0];
  long height = xyz_map->size[1];
  long width = xyz_map->size[2];
  double *data =  THDoubleTensor_data( xyz_map );

  #ifdef DEBUG
  // DEBUG printing 
  printf("size[0]: %i\n", stack);
  printf("size[1]: %i\n", height);
  printf("size[2]: %i\n", width);
  #endif 

  // Ensure that the result is of the correct size 
  THDoubleTensor_resize2d( classification, height, width );
  THDoubleTensor_fill( classification, 0 );
  double* classification_data = THDoubleTensor_data( classification);

  THDoubleTensor_resize2d( errors, height, width );
  THDoubleTensor_fill( errors, 0 );
  double* errors_data = THDoubleTensor_data( errors );

  THDoubleTensor_resize3d( normals, stack, height, width );
  THDoubleTensor_fill( normals, 0 );
  double* normals_data = THDoubleTensor_data( normals );

  Eigen::Vector3d seed_pos;
  Eigen::Vector3d pos; 
  Eigen::Vector3d normal;
  Eigen::Vector3d mean; 
  Eigen::Matrix3d covariance;  

  // Eigenvalues and eigenvectors 
  Eigen::Vector3d eigenvalues;
  Eigen::Matrix3d eigenvectors;

  Eigen::Vector3d eigenvalues_sorted;
  Eigen::Vector3d eigenvalues_sorted_inds;

  // Apply operation to all elements in map 
  for ( long i=0; i<height; i++ ) {
    for ( long j=0; j<width; j++ ) {
      // Don't compute value at the edges of the window 
      if ( i-window_width < 0 || j-window_width < 0 || i+window_width+1 > height || j+window_width+1 > width ) {
        #ifdef DEBUG
        printf("continueing: [%d, %d] \n", i, j);
        #endif
        continue;
      }
      seed_pos(0) = data[0*height*width + i*width + j];
      seed_pos(1) = data[1*height*width + i*width + j];
      seed_pos(2) = data[2*height*width + i*width + j];
      // Compute mean of the window ... TODO: i,j,k,l are shitty names  
      mean.setZero();
      int mean_cnt = 0;
      int total_cnt = 0;
      for ( long k=i-window_width; k<i+window_width+1; k++ ) { 
        for ( long l=j-window_width; l<j+window_width+1; l++ ) { 
          // Check if point is less than our distance thresh  ... distance thresholding is problematic 
          //  since we end up thresholding out points that would vote against our plane 
          pos(0) = data[0*height*width + k*width + l];
          pos(1) = data[1*height*width + k*width + l];
          pos(2) = data[2*height*width + k*width + l];
          if ( (pos - seed_pos).norm() > dist_thresh && (k<i-1 || k>i+1 || l<j-1 || l>j+1)  ) {
            total_cnt++;
            continue;
          }

          mean(0) += data[0*height*width + k*width + l];
          mean(1) += data[1*height*width + k*width + l];
          mean(2) += data[2*height*width + k*width + l];
          mean_cnt++; // This is technically unnecessary 
        }
      }
      // If we didn't get enough points then ignore
      /*
      if ( mean_cnt < 9 ) {
        classification_data[i*width+j] = 0;
        continue;
      }
      */
      mean = mean/double(mean_cnt);

      // Compute covariance/scatter 
      covariance.setZero();
      int cov_cnt = 0;
      for ( long k=i-window_width; k<i+window_width+1; k++ ) { 
        for ( long l=j-window_width; l<j+window_width+1; l++ ) { 
          pos(0) = data[0*height*width + k*width + l];
          pos(1) = data[1*height*width + k*width + l];
          pos(2) = data[2*height*width + k*width + l];
          // Check if point is less than our distance thresh 
          if ( (pos - seed_pos).norm() > dist_thresh && (k<i-1 || k>i+1 || l<j-1 || l>j+1)  ) {
            continue;
          }

          covariance += (pos - mean)*((pos - mean).transpose());
          cov_cnt++;
        }
      }
      covariance = covariance/double(cov_cnt);
      // Debug print out mean 

      #ifdef DEBUG
      printf("At idx: [%d, %d] \n", i,j);
      std::cout << "mean: " << mean << std::endl;
      std::cout << "covariance: " << covariance << std::endl;
      #endif

      // Compute eigenvalue and eigenvectors of covariance/scatter matrix 
      Eigen::EigenSolver<Eigen::Matrix3d> eig_solver(covariance); 
      // Only take real parts of eigenvalues since covariance is positive semidefinite
      eigenvalues = eig_solver.eigenvalues().real();
      eigenvectors = eig_solver.eigenvectors().real();

      // Laziest sorting code ever 
      int ind;
      double max_eig = eigenvalues.maxCoeff( &ind ); 
      eigenvalues_sorted(2) = max_eig;
      eigenvalues_sorted_inds(2) = ind;
      eigenvalues_sorted(0) = eigenvalues.minCoeff( &ind );
      eigenvalues_sorted_inds(0) = ind;
      eigenvalues(ind) = max_eig+1;
      eigenvalues_sorted(1) = eigenvalues.minCoeff( &ind );
      eigenvalues_sorted_inds(1) = ind;
      // End of pure grossness 

      #ifdef DEBUG
      // DEBUG: print out eigenvalues and eigenvectors 
      std::cout << "eigenvalues: " << eig_solver.eigenvalues() << std::endl;
      std::cout << "eigenvectors: " << eig_solver.eigenvectors() << std::endl;
      std::cout << "eigenvalues sorted: " << eigenvalues_sorted << std::endl;
      std::cout << "eigenvalues sorted inds: " << eigenvalues_sorted_inds << std::endl;
      #endif

      // Save out normals estimates, I want to see if these make any sense 
      normal = eigenvectors.col(eigenvalues_sorted_inds(0));
      normal = normal.normalized();

      // Flip the normal if it is pointing the wrong direction
      if ( mean.dot( normal ) < 0 ) { 
        normal = -1.0*normal;
      }

      // Remember to normalize the normal
      normals_data[0*height*width + i*width + j] = normal(0);
      normals_data[1*height*width + i*width + j] = normal(1);
      normals_data[2*height*width + i*width + j] = normal(2);

      // If the minimum eigenvalue is small enough then we consider this region to be a plane  
      if ( eigenvalues_sorted(0) <= plane_thresh*eigenvalues_sorted(1) ) {
        classification_data[i*width+j] = 1;
      }
      // Hold on to error for each plane ... TODO: handle super tiny eigenvalues
      if ( eigenvalues_sorted(1) < 0.01 ) {
        continue;
      }
      errors_data[i*width+j] = eigenvalues_sorted(0)/eigenvalues_sorted(1);
      //errors_data[i*width+j] = dist;
    }
  }
}

/* Region Growing based plane identification
  - Given a seed point expand a plane as follows:
    - Add edges to edges list
    - Pop edge with least error relative to current plane estimate
    - Check if that new edge is different by some threshold, if so we are done 
    - Otherwise add that point to the current set and update the plane parameters

     This algorithm is pretty naive and greedy, we must update the children errors each time we
     add a new child to the current plane estimate 
*/
void grow_region( THLongTensor* start_index, THDoubleTensor* covariances ) {
   double *index_data =  THLongTensor_data( start_index);
   printf( "start indices: [%f, %f] \n", start_index[0], start_index[1]);
}

/*
PointCloud<PointType>* PointCloud_fromFile(const char * fname)
{
  PointCloud<PointType>* cloud = new PointCloud<PointType>();
  if (io::loadPCDFile<PointType> (fname, *cloud) == -1)
  {
    PCL_ERROR ("Load PCDFile: Couldn't read file %s \n", fname);
  }
  
  cout << "Load PCDFile: Loaded "
            << cloud->width * cloud->height
            << " data points from " << fname << "."
            << endl;

  //add fudge factor... who knows what they store for point clouds
  return cloud;
}


void PointCloud_destroy(PointCloud<PointType>* cloud)
{
  delete(cloud);
}

//xyzrgb should be nx6
PointCloud<PointType>* PointCloud_fromTensor(THDoubleTensor* xyz)
{
  double * xyz_d       = THDoubleTensor_data(xyz);

  PointType newPoint;
  PointCloud<PointType>* cloud = new PointCloud<PointType>();

  for (int i=0; i< xyz->size[0] ; i++){
    // Find 3D position respect to rgb frame:
    newPoint.x = xyz_d[0];

    newPoint.y = xyz_d[1];
    newPoint.z = xyz_d[2];

    cloud->push_back(newPoint);
    xyz_d+=3;
  }
  return cloud;
}

//xyzrgb should be nx6
void PointCloud_toTensor(PointCloud<PointType>* cloud, THDoubleTensor* xyz)
{
  THDoubleTensor_resize2d(xyz, cloud->height*cloud->width, 3);
  THDoubleTensor_fill(xyz,0);


  double * xyz_d = THDoubleTensor_data(xyz);

  PointCloud<PointType>::iterator it;

  // Find 3D position respect to rgb frame:
  for (it= cloud->points.begin(); it < cloud->points.end(); it++)
  {
    xyz_d[0] = it->x;
    xyz_d[1] = it->y;
    xyz_d[2] = it->z;
    xyz_d+=3;
  }
}
   
int PointCloud_toFile(PointCloud<PointType>* test, const char * fname)
{
  if(io::savePCDFileASCII (fname, *test) == -1) 
  {
    PCL_ERROR ("Couldn't write to file %s \n", fname);
    return -1;
  }

  return 0;
}



//xyzrgb should be nx6
bool PointCloud_doICP(PointCloud<PointType>* cloud1, PointCloud<PointType>* cloud2, THDoubleTensor* transf, 
  double epsilon = 1e-6, double maxIterations = 100, double ransacIterations=10000, double maxCorrespondDis=.2)
{
  THDoubleTensor_resize2d(transf, 4, 4);
  THDoubleTensor_fill(transf,0);
  double * transf_d = THDoubleTensor_data(transf);

  PointCloud<PointType>::Ptr final (new PointCloud<PointType>);
  //PointCloud<PointType>::Ptr ptr1 = cloud_with_normals1->makeShared();
  //PointCloud<PointType>::Ptr ptr2 = cloud_with_normals2->makeShared();

  IterativeClosestPoint<PointType, PointType> icp;
  icp.setInputSource(cloud1->makeShared());
  icp.setInputTarget(cloud2->makeShared());
  icp.setTransformationEpsilon(epsilon);
  icp.setMaximumIterations(maxIterations);
  icp.setRANSACIterations(ransacIterations);
  icp.setMaxCorrespondenceDistance(maxCorrespondDis);
  
  icp.align(*final);
  Eigen::Matrix4f transfICP = icp.getFinalTransformation();
  for(int i=0; i<4; i++) {
    for(int j=0; j<4; j++) {
      *transf_d = transfICP(i,j);
      *transf_d++;
    }
  }
  return icp.hasConverged();
}


PointCloud<PointType>* PointCloud_pc_create()
{
  return new PointCloud<PointType> ();

}


PointCloud<PointType>* PointCloud_uniformSample(PointCloud<PointType>* cloud, double radius = .01)
{
  PointCloud<PointType>* downsampled = new PointCloud<PointType>();

  PointCloud<int> sampled_indices;
  UniformSampling<PointType> uniform_sampling;
  uniform_sampling.setInputCloud (cloud->makeShared());
  uniform_sampling.setRadiusSearch (radius);
  uniform_sampling.compute (sampled_indices);

  copyPointCloud (*cloud, sampled_indices.points, *downsampled);

  return downsampled;
}

double PointCloud_getEuclideanValidationScore(PointCloud<PointType>* source, PointCloud<PointType>* target, double max_range)
{
  Eigen::Matrix4f ti = Eigen::Matrix4f::Identity ();

  TransformationValidationEuclidean<PointType, PointType> tve;
  tve.setMaxRange (max_range);  // 1cm
  return tve.validateTransformation (source->makeShared(), target->makeShared(), ti);
}

  //

*/
} // extern "C"
