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
using namespace pcl;
using namespace registration;

typedef PointXYZ PointType;

extern "C"
{

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


} // extern "C"
