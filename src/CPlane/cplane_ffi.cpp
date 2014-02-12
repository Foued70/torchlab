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
#include <math.h>
 
using namespace std;
/*
using namespace pcl;
using namespace registration;

typedef PointXYZ PointType;
*/

//#define DEBUG

extern "C"
{

/* 
  Cosine distance between two vectors 
*/
double cosine_distance( Eigen::Vector3d v0, Eigen::Vector3d v1 ) {
  /* DEBUG 
  std::cout << "cos_dist test: " << v0.dot(v1) << std::endl;
  std::cout << "cos_dist test2: " << acos(0.99999999999) << std::endl;
  */
  double dist = v0.dot(v1);
  // Account for acos(1) case 
  if ((1-dist) < 1e-12)
    return 0.0;
  return abs(acos(dist));
}

double residual_distance( Eigen::Vector3d normal, Eigen::Vector3d mean, Eigen::Vector3d point ) {
  return abs(normal.dot(mean - point));
}

/* 
  Given covariance extract plane equation
*/
void extract_plane_from_covariance(  Eigen::Matrix3d covariance, Eigen::Vector3d mean, Eigen::Vector3d* normal, double* d, Eigen::Vector3d* eigvals ) { 
  Eigen::Vector3d eigenvalues_sorted;
  Eigen::Vector3d eigenvalues_sorted_inds;
  Eigen::Vector3d local_normal;
  double local_d; 
  // Compute eigenvalue and eigenvectors of covariance/scatter matrix 
  Eigen::EigenSolver<Eigen::Matrix3d> eig_solver(covariance); 
  // Only take real parts of eigenvalues since covariance is positive semidefinite
  Eigen::Vector3d eigenvalues = eig_solver.eigenvalues().real();
  Eigen::Matrix3d eigenvectors = eig_solver.eigenvectors().real();

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

  // Save out normals estimates, I want to see if these make any sense 
  local_normal = eigenvectors.col(eigenvalues_sorted_inds(0));
  // Remember to normalize the normal
  local_normal = local_normal.normalized();

  local_d = local_normal.dot( mean );
  // Flip the normal and d if it is pointing the wrong direction
  if ( local_d < 0 ) { 
    local_normal = -1.0*local_normal;
    local_d = -1.0*local_d;
  }
  // Update return values
  *normal = local_normal;
  *d = local_d;
  *eigvals = eigenvalues_sorted;
}


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
void classifyPoints( THDoubleTensor* xyz_map, int window, double dist_thresh, THDoubleTensor* errors, THDoubleTensor* normals, THDoubleTensor* th_means, THDoubleTensor* th_second_moments ) { 

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

  THDoubleTensor_resize2d( errors, height, width );
  THDoubleTensor_fill( errors, 0 );
  double* errors_data = THDoubleTensor_data( errors );

  THDoubleTensor_resize3d( normals, stack, height, width );
  THDoubleTensor_fill( normals, 0 );
  double* normals_data = THDoubleTensor_data( normals );

  THDoubleTensor_resize3d( th_means, stack, height, width );
  THDoubleTensor_fill( th_means, 0 );
  double* means = THDoubleTensor_data( th_means );

  // Second moments is a stack of 9 
  THDoubleTensor_resize3d( th_second_moments, 9, height, width );
  THDoubleTensor_fill( th_second_moments, 0 );
  double* second_moments = THDoubleTensor_data( th_second_moments );

  Eigen::Vector3d seed_pos;
  Eigen::Vector3d pos; 
  Eigen::Vector3d normal;
  Eigen::Vector3d mean; 
  Eigen::Matrix3d covariance; 
  Eigen::Matrix3d second_moment;  

  // Eigenvalues and eigenvectors 
  Eigen::Vector3d eigenvalues;
  Eigen::Matrix3d eigenvectors;

  Eigen::Vector3d eigenvalues_sorted;
  Eigen::Vector3d eigenvalues_sorted_inds;

  double d;

  // Apply operation to all elements in map 
  for ( long i=0; i<height; i++ ) {
    for ( long j=0; j<width; j++ ) {
      // Don't compute value at the edges of the window 
      if ( i-window_width < 0 || j-window_width < 0 || i+window_width+1 > height || j+window_width+1 > width ) {
        #ifdef DEBUG
        printf("continueing: [%d, %d] \n", i, j);
        #endif
        errors_data[i*width+j] = 1;
        continue;
      }
      seed_pos(0) = data[0*height*width + i*width + j];
      seed_pos(1) = data[1*height*width + i*width + j];
      seed_pos(2) = data[2*height*width + i*width + j];
      // If we are an invalid point then set errors_data to 1 
      if ( seed_pos(0) == 0 && seed_pos(1) == 0 && seed_pos(2) == 0 ) {
        errors_data[i*width+j] = 1;
        continue;
      }
      // Compute mean of the window ... TODO: i,j,k,l are shitty names  
      mean.setZero();
      int mean_cnt = 0;
      for ( long k=i-window_width; k<i+window_width+1; k++ ) { 
        for ( long l=j-window_width; l<j+window_width+1; l++ ) { 
          // Check if point is less than our distance thresh  ... distance thresholding is problematic 
          //  since we end up thresholding out points that would vote against our plane 
          pos(0) = data[0*height*width + k*width + l];
          pos(1) = data[1*height*width + k*width + l];
          pos(2) = data[2*height*width + k*width + l];
          if ( (pos - seed_pos).norm() > dist_thresh && (k<i-1 || k>i+1 || l<j-1 || l>j+1)  ) {
            continue;
          }

          // Check if this is an invalid point
          if ( pos(0) == 0 && pos(1) == 0 && pos(2) == 0 ) {
            continue;
          }

          mean(0) += data[0*height*width + k*width + l];
          mean(1) += data[1*height*width + k*width + l];
          mean(2) += data[2*height*width + k*width + l];
          mean_cnt++; // This is technically unnecessary 
        }
      }
      // If we didn't get enough points then ignore
      if ( mean_cnt < 9 ) {
        errors_data[i*width+j] = 1;
        continue;
      }
      mean = mean/double(mean_cnt);

      // Compute covariance/scatter and second moment 
      Eigen::Vector3d diffvec;
      covariance.setZero();
      second_moment.setZero();
      int cnt = 0;
      for ( long k=i-window_width; k<i+window_width+1; k++ ) { 
        for ( long l=j-window_width; l<j+window_width+1; l++ ) { 
          pos(0) = data[0*height*width + k*width + l];
          pos(1) = data[1*height*width + k*width + l];
          pos(2) = data[2*height*width + k*width + l];
          // Check if point is less than our distance thresh 
          if ( (pos - seed_pos).norm() > dist_thresh && (k<i-1 || k>i+1 || l<j-1 || l>j+1)  ) {
            continue;
          }

          // Check if this is an invalid point
          if ( pos(0) == 0 && pos(1) == 0 && pos(2) == 0 ) {
            continue;
          }

          diffvec = pos - mean;

          covariance += diffvec * diffvec.transpose();
          second_moment += pos * pos.transpose(); 
          cnt++;
        }
      }
      covariance = covariance/double(cnt);
      second_moment = second_moment/double(cnt);

      #ifdef DEBUG
      printf("At idx: [%d, %d] \n", i,j);
      std::cout << "mean: " << mean << std::endl;
      std::cout << "covariance: " << covariance << std::endl;
      #endif

      extract_plane_from_covariance(  covariance, mean, &normal, &d, &eigenvalues_sorted );

      // Write out normals 
      normals_data[0*height*width + i*width + j] = normal(0);
      normals_data[1*height*width + i*width + j] = normal(1);
      normals_data[2*height*width + i*width + j] = normal(2);

      // Write out means 
      means[0*height*width + i*width + j] = mean(0);
      means[1*height*width + i*width + j] = mean(1);
      means[2*height*width + i*width + j] = mean(2);

      // Write out second moments 
      for ( int smi=0; smi < 9; smi++ ) { 
        second_moments[smi*height*width + i*width + j] = second_moment(smi);
      }

      /*
      // If the minimum eigenvalue is small enough then we consider this region to be a plane  
      if ( eigenvalues_sorted(0) <= plane_thresh*eigenvalues_sorted(1) ) {
        classification_data[i*width+j] = 1;
      }
      */

      // Hold on to error for each plane ... TODO: handle super tiny eigenvalues
      /*
      if ( eigenvalues_sorted(1) < 0.01 ) {
        errors_data[i*width+j] = 1;
        continue;
      }
      */
      //errors_data[i*width+j] = eigenvalues_sorted(0)/(eigenvalues_sorted(0)+eigenvalues_sorted(1)+eigenvalues_sorted(2));
      errors_data[i*width+j] = eigenvalues_sorted(0)/(eigenvalues_sorted(0)+eigenvalues_sorted(1));
      //errors_data[i*width+j] = dist;
    }
  }
}


/* 
  Cull points based on normal / residual thresholds
    - Looks at each point in the window, if any are outside of the thresholds then sets the 
      mask for that the seed index to 1, otherwise 0 
*/
void cullPoints( THDoubleTensor* th_cull_mask, THDoubleTensor* th_normals, THDoubleTensor* th_points, int window, double cosine_thresh, double residual_thresh ) {
  // Compute halfwidth of the window ... windows should be odd 
  int window_width = (window - 1)/2;
  printf("window_width: %d \n", window_width);

  // 3d Matrix 
  long stack = th_points->size[0];
  long height = th_points->size[1];
  long width = th_points->size[2];
  double *point_data =  THDoubleTensor_data( th_points );
  double *normal_data =  THDoubleTensor_data( th_normals);

  THDoubleTensor_fill( th_cull_mask, 0 );
  double *cull_mask_data = THDoubleTensor_data( th_cull_mask );

  Eigen::Vector3d seed_pos;
  Eigen::Vector3d seed_normal;
  Eigen::Vector3d pos; 
  Eigen::Vector3d normal;

  double cdist;
  double rdist;

  // Apply operation to all elements in map 
  for ( long i=0; i<height; i++ ) {
    for ( long j=0; j<width; j++ ) {
      // Don't compute value at the edges of the window 
      if ( i-window_width < 0 || j-window_width < 0 || i+window_width+1 > height || j+window_width+1 > width ) {
        #ifdef DEBUG
        printf("continuing: [%d, %d] \n", i, j);
        #endif
        cull_mask_data[i*width+j] = 1;
        continue;
      }
      // Seed pos
      seed_pos(0) = point_data[0*height*width + i*width + j];
      seed_pos(1) = point_data[1*height*width + i*width + j];
      seed_pos(2) = point_data[2*height*width + i*width + j];
      // Seed normal 
      seed_normal(0) = normal_data[0*height*width + i*width + j];
      seed_normal(1) = normal_data[1*height*width + i*width + j];
      seed_normal(2) = normal_data[2*height*width + i*width + j];

      // If we are an invalid point then set errors_data to 1 
      if ( seed_pos(0) == 0 && seed_pos(1) == 0 && seed_pos(2) == 0 ) {
        cull_mask_data[i*width+j] = 1;
        continue;
      }
      // Compute mean of the window ... TODO: i,j,k,l are shitty names  
      for ( long k=i-window_width; k<i+window_width+1; k++ ) { 
        for ( long l=j-window_width; l<j+window_width+1; l++ ) { 
          // Extract pos data 
          pos(0) = point_data[0*height*width + k*width + l];
          pos(1) = point_data[1*height*width + k*width + l];
          pos(2) = point_data[2*height*width + k*width + l];
          // Extract normal data 
          normal(0) = normal_data[0*height*width + k*width + l];
          normal(1) = normal_data[1*height*width + k*width + l];
          normal(2) = normal_data[2*height*width + k*width + l];

          cdist = cosine_distance( seed_normal, normal );
          rdist = residual_distance( seed_normal, seed_pos, pos ); 

          if ( cdist > cosine_thresh || rdist > residual_thresh ) {
            cull_mask_data[i*width+j] = 1;
          }
        }
      }
    }
  }
}

/* 
  Add Neighbors to front ... ridiculously long function definition, I need some higher level datastructures, lol
*/
void addNeighbors( long node_index, Eigen::Vector3d* mean, Eigen::Vector3d* normal, std::map<double,long>* front, 
                   double* means, double* normals, double* region_mask, double* front_mask, long width, long height ) {
  long y = node_index/width;
  long x = node_index - y*width;
  Eigen::Vector3d neighbor_mean;
  Eigen::Vector3d neighbor_normal;
  double cdist;
  double rdist;
  long index; 
  for (int i=y-1; i<y+2; i++) {
    for (int j=x-1; j<x+2; j++) {
      // Don't go out of bounds
      if ( i < 1 || j < 1 || i > height-1 || j > width-1)
        continue;
      // We are not our own neighbor 
      if ( i == y && j == x )
        continue;

      index = i*width + j;
      //std::cout << "index: " << index << std::endl;
      // Check if neighbor index is in plane_inliers
      if ( region_mask[index] > 0.5 || front_mask[index] > 0.5 ) {
        //std::cout << "region mask index: " << index << std::endl;
        continue;
      }

      // Extract neighbor mean
      for (int k=0; k<3; k++) {
        neighbor_mean(k) = means[k*height*width + index];
      }
      // Extract neighbor normal
      for (int k=0; k<3; k++) {
        neighbor_normal(k) = normals[k*height*width + index];
      }
      cdist = cosine_distance( *normal, neighbor_normal );
      rdist = residual_distance( *normal, *mean, neighbor_mean );
      /*
      std::cout << "cosine_distance: " << cdist << std::endl;
      std::cout << "residual_distance: " << rdist << std::endl;
      std::cout << "combined error: " << cdist*rdist << std::endl;
      std::cout << "neighbor_mean: " << neighbor_mean << std::endl;
      */

      // Insert neighbor into the front 
      front->insert( std::pair<double,long>(cdist*rdist,index) );
      //front->insert( std::pair<double,long>(cdist,index) );
      front_mask[index] = 1;
    }
  }

  /*
  std::cout << "front contains: " << std::endl;
  for (std::map<double,long>::iterator it=front->begin(); it!=front->end(); ++it)
    std::cout << it->first << " => " << it->second << std::endl;
  */

}

/* 
  Add minimum error point to plane set, update plane parameters and recompute neighbor errors 
    // Way too many function arguments!!! aaahhh
     This code is mad-gross, a couple structs would fix this ( TODO yo )
*/
long updatePlane( std::map<double,long>* front, Eigen::Matrix3d* covariance, Eigen::Matrix3d* second_moment, Eigen::Vector3d* mean,
                  Eigen::Vector3d* normal, int* num_points, double* means, double* normals, double* second_moments, double* region_mask,  
                  long width, long height, double cosine_thresh, double residual_thresh ) {

  // Extract minimum error point
  long index = front->begin()->second;
  double error = front->begin()->first;
  // Remove first element from front 
  front->erase(front->begin());
  // Update region mask with new node
  region_mask[index] = 1;

  if ( index == front->begin()->second ) {
    std::cout << "Error index0 and index1 are the same" << std::endl;
    return -1;
  }

  // Check if minimum error point is within thresholds 
  /*
  double cosine_thresh = 0.3;    
  double residual_thresh = 20.0;
  */
  double cdist;
  double rdist; 

  Eigen::Vector3d neighbor_mean;
  Eigen::Vector3d neighbor_normal;
  Eigen::Matrix3d neighbor_second_moment;

  // Extract neighbor mean
  for (int k=0; k<3; k++) {
    neighbor_mean(k) = means[k*height*width + index];
  }
  // Extract neighbor normal
  for (int k=0; k<3; k++) {
    neighbor_normal(k) = normals[k*height*width + index];
  }
  cdist = cosine_distance( *normal, neighbor_normal );
  rdist = residual_distance( *normal, *mean, neighbor_mean ); 

  // If the best neighbor is above the thresholds then we are done
  if ( cdist > cosine_thresh || rdist > residual_thresh ) {
  //if ( cdist > cosine_thresh ) {
      /*
      std::cout << "cosine_distance: " << cdist << std::endl;
      std::cout << "residual_distance: " << rdist << std::endl;
      std::cout << "num_points: " << *num_points << std::endl;
      std::cout << "mean: "  << *mean << std::endl;
      std::cout << "second_moment: " << *second_moment << std::endl;
      std::cout << "covariance: " << *covariance << std::endl;
      */
      std::cout << "cdist or rdist above thresh exiting " << std::endl;
      return -1;
  }

  // Extract neighbor second moment 
  for (int i=0; i<9; i++) {
    neighbor_second_moment(i) = second_moments[i*height*width + index];
  }

  Eigen::Vector3d sub;

  /*
  std::cout << "Before" << std::endl;
  std::cout << "num_points: " << *num_points << std::endl;
  std::cout << "mean: "  << *mean << std::endl;
  std::cout << "second_moment: " << *second_moment << std::endl;
  std::cout << "covariance: " << *covariance << std::endl;
  */

  sub = double(*num_points)*(*mean) + neighbor_mean; 
  //std::cout << "sub: "  << sub << std::endl;
  (*num_points)++;
  // Update mean 
  *mean = sub / double(*num_points);
  // Update second_moment 
  *second_moment = *second_moment + neighbor_second_moment;
  // Update covariance
  *covariance = *second_moment - sub*((*mean).transpose());

  /*
  std::cout << "After" << std::endl;
  std::cout << "num_points: " << *num_points << std::endl;
  std::cout << "mean: "  << *mean << std::endl;
  std::cout << "second_moment: " << *second_moment << std::endl;
  std::cout << "covariance: " << *covariance << std::endl;
  */

  // TODO: compute normal, etc ... 
  double plane_d;
  Eigen::Vector3d eigenvalues;
  extract_plane_from_covariance( *covariance, *mean, normal, &plane_d, &eigenvalues );

  /* DEBUG: this is very expensive also segfaults, hehe
  // Update errors for each neighbor  ... there is certainly a better approach than i have chosen
  std::map<double, long> new_front; 
  //for (std::map<double,long>::iterator it=front->begin(); it!=front->end(); ++it) {
  std::map<double,long>::iterator it = front->begin();
  long node_index; 
  while ( it != front->end() ) {
    node_index = it->second;
    // Erase and increment
    front->erase(it);
    it++;

    // Extract neighbor mean
    for (int k=0; k<3; k++) {
      neighbor_mean(k) = means[k*height*width + node_index];
    }
    // Extract neighbor normal
    for (int k=0; k<3; k++) {
      neighbor_normal(k) = normals[k*height*width + node_index];
    }
    cdist = cosine_distance( *normal, neighbor_normal );
    rdist = residual_distance( *normal, *mean, neighbor_mean );  
    new_front.insert( std::pair<double,long>(cdist*rdist,node_index) );
  }

  for (std::map<double,long>::iterator it=new_front.begin(); it!=new_front.end(); ++it) {
    front->insert( std::pair<double,long>(it->first,it->second) );
  }
  */


  /*
  std::cout << "front contains: " << std::endl;
  for (std::map<double,long>::iterator it=front->begin(); it!=front->end(); ++it)
    std::cout << it->first << " => " << it->second << std::endl;
  */

  return index;
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
void grow_plane_region( THLongTensor* th_start_indices, THDoubleTensor* th_normals, THDoubleTensor* th_means, 
                        THDoubleTensor* th_second_moments, THDoubleTensor* th_region_mask, THDoubleTensor* th_front_mask, 
                        THDoubleTensor* th_output_plane, THDoubleTensor* th_output_mean, double cosine_thresh, double residual_thresh ) {

  // Inputs 
  // Extract relevent parameters
  long height = th_means->size[1];
  long width = th_means->size[2];

  long *start_indices =  THLongTensor_data( th_start_indices );
  long start_y = start_indices[0];
  long start_x = start_indices[1];
  //printf( "start indices: [%i, %i] \n", start_y, start_x);
  long start_index = start_y*width + start_x;

  //printf( "[height, width]: [%d, %d] \n", height, width);
  double* means = THDoubleTensor_data( th_means );
  double* normals = THDoubleTensor_data( th_normals );
  double* second_moments = THDoubleTensor_data( th_second_moments );

  THDoubleTensor_resize2d( th_region_mask, height, width );
  THDoubleTensor_fill( th_region_mask, 0 );
  double* region_mask = THDoubleTensor_data( th_region_mask );
  region_mask[start_index] = 1;

  THDoubleTensor_resize2d( th_front_mask, height, width );
  THDoubleTensor_fill( th_front_mask, 0 );
  double* front_mask = THDoubleTensor_data( th_front_mask );


  // Properties to track for the root plane
  Eigen::Matrix3d covariance;
  Eigen::Matrix3d second_moment;
  Eigen::Vector3d mean;
  Eigen::Vector3d normal;
  double plane_d;
  int num_points = 1;  

  // Store the front indices and errors 
  std::map<double, long> front; 

  // Initialize root values
  // mean
  for (int i=0; i<3; i++) {
    mean(i) = means[i*height*width + start_y*width + start_x];
  }
  // second moment 
  for (int i=0; i<9; i++) {
    second_moment(i) = second_moments[i*height*width + start_y*width + start_x];
  }
  // Covariance 
  covariance = second_moment - mean*mean.transpose();  

  Eigen::Vector3d eigenvalues;
  extract_plane_from_covariance( covariance, mean, &normal, &plane_d, &eigenvalues );

  // DEBUG: get original normal 
  Eigen::Vector3d orig_normal;
  for (int i=0; i<3; i++) {
    orig_normal(i) = normals[i*height*width + start_y*width + start_x];
  }

  // TEST: yes these are the same ... good
  std::cout << "original normal: " << orig_normal << std::endl;
  std::cout << "computed normal: " << normal << std::endl;
  std::cout << "mean: " << mean << std::endl;

  std::cout << "cosine_dist: " << cosine_distance( orig_normal, normal ) << std::endl;

  // Add Neighbors of index 
  long index = start_index;
  //for (int i=0; i<100; i++ ) {
  while ( true ) {
    // Add neighbors 
    // Extract minimum and update plane
    addNeighbors( index, &mean, &normal, &front, means, normals, region_mask, front_mask, width, height);
    while ( true ) {
      index = updatePlane( &front, &covariance, &second_moment, &mean, &normal, &num_points, means, normals, second_moments, region_mask, width, height, cosine_thresh, residual_thresh );
      if ( front.begin() == front.end() ) {
        std::cout << "Exiting no more elements in front" << std::endl;
        // Output plane equation 
        double* output_plane_data = THDoubleTensor_data( th_output_plane );
        double* output_mean_data = THDoubleTensor_data( th_output_mean );
        // Copy normal data 
        for (int k=0; k<3; k++) {
          output_plane_data[k] = normal(k);
        }
        output_plane_data[3] = plane_d;
        // Copy mean data 
        for (int k=0; k<3; k++) {
          output_mean_data[k] = mean(k);
        }
        return;
      }
      if ( index == -1 )
        continue;
      break;
    }
  }




  /*
  Eigen::Vector3d neighbor_mean;
  Eigen::Vector3d neighbor_normal;
  double cdist;
  double rdist;
  long index; 
  for (int i=start_y-1; i<start_y+2; i++) {
    for (int j=start_x-1; j<start_x+2; j++) {
      // We are not our own neighbor 
      if ( i == start_x && j == start_x )
        continue;

      index = i*width + j;
      // Check if neighbor index is in plane_inliers
      if ( region_mask[index] ) 
        continue;

      // Extract neighbor mean
      for (int k=0; k<3; k++) {
        neighbor_mean(k) = means[k*height*width + index];
      }
      // Extract neighbor normal
      for (int k=0; k<3; k++) {
        neighbor_normal(k) = normals[k*height*width + index];
      }
      cdist = cosine_distance( normal, neighbor_normal );
      rdist = residual_distance( normal, mean, neighbor_mean );
      std::cout << "cosine_distance: " << cdist << std::endl;
      std::cout << "residual_distance: " << rdist << std::endl;
      std::cout << "combined error: " << cdist*rdist << std::endl;
      std::cout << "neighbor_mean: " << neighbor_mean << std::endl;

      // Insert neighbor into the front 
      front.insert( std::pair<double,long>(cdist*rdist,index) );
    }
  }

  std::cout << "front contains: " << std::endl;
  for (std::map<double,long>::iterator it=front.begin(); it!=front.end(); ++it)
    std::cout << it->first << " => " << it->second << std::endl;
  */

}


} // extern "C"
