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

typedef PointXYZRGB PointType;
typedef PointXYZRGBNormal PointColorNormalType;
typedef Normal NormalType;
typedef SHOT352 DescriptorType;

extern "C"
{
void test(PointType* a) 
{
  cout << "new" << a->x << "   " << a->y << "   " << a->z << "   " << a->r << "   " << a->g << "   " << a->b << endl;
}

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

void PointCloudN_destroy(PointCloud<PointNormal>* cloud)
{
  delete(cloud);
}
void PointCloudMesh_destroy(PolygonMesh* mesh)
{
  delete(mesh);
}

void PointCloudDescriptors_destroy(PointCloud<DescriptorType>* cloud) 
{
  delete(cloud);
}


//xyzrgb should be nx6
PointCloud<PointType>* PointCloud_fromTensor(THDoubleTensor* xyz, THDoubleTensor* rgb)
{
  double * xyz_d       = THDoubleTensor_data(xyz);
  double * rgb_d       = THDoubleTensor_data(rgb);

  PointType newPoint;
  PointCloud<PointType>* cloud = new PointCloud<PointType>();

  for (int i=0; i< xyz->size[0] ; i++){
    // Find 3D position respect to rgb frame:
    newPoint.x = xyz_d[0];

    newPoint.y = xyz_d[1];
    newPoint.z = xyz_d[2];
    newPoint.r = (uint8_t)rgb_d[0];
    newPoint.g = (uint8_t)rgb_d[1];
    newPoint.b = (uint8_t)rgb_d[2];

    cloud->push_back(newPoint);
    xyz_d+=3;
    rgb_d+=3;
  }
  return cloud;
}


PointCloud<NormalType>* PointCloud_getNormals_fromTensor(THDoubleTensor* norm)
{
  double * norm_d       = THDoubleTensor_data(norm);

  NormalType newNorm;
  PointCloud<NormalType>* cloud = new PointCloud<NormalType>();

  for (int i=0; i< norm->size[0] ; i++){
    // Find 3D position respect to rgb frame:
    newNorm.normal_x = norm_d[0];
    newNorm.normal_y = norm_d[1];
    newNorm.normal_z = norm_d[2];
    cloud->push_back(newNorm);
    norm_d+=3;
  }
  return cloud;
}


//xyzrgb should be nx6
void PointCloud_normalToTensor(PointCloud<NormalType>* cloud, THDoubleTensor* nxyz)
{
  THDoubleTensor_resize2d(nxyz, cloud->height*cloud->width,3);
  THDoubleTensor_fill(nxyz,0);


  double * nxyz_d = THDoubleTensor_data(nxyz);


  PointCloud<NormalType>::iterator it;

  // Find 3D position respect to rgb frame:
  for (it= cloud->points.begin(); it < cloud->points.end(); it++)
  {
    nxyz_d[0] = it->normal_x;
    nxyz_d[1] = it->normal_y;
    nxyz_d[2] = it->normal_z;
    nxyz_d+=3;
  }
}

//xyzrgb should be nx6
void PointCloud_toTensor(PointCloud<PointType>* cloud, THDoubleTensor* xyz, THDoubleTensor* rgb)
{
  THDoubleTensor_resize2d(xyz, cloud->height*cloud->width, 3);
  THDoubleTensor_fill(xyz,0);


  THDoubleTensor_resize2d(rgb, cloud->height*cloud->width, 3);
  THDoubleTensor_fill(rgb,0);

  double * xyz_d = THDoubleTensor_data(xyz);
  double * rgb_d = THDoubleTensor_data(rgb);

  PointCloud<PointType>::iterator it;

  // Find 3D position respect to rgb frame:
  for (it= cloud->points.begin(); it < cloud->points.end(); it++)
  {
    xyz_d[0] = it->x;
    xyz_d[1] = it->y;
    xyz_d[2] = it->z;
    xyz_d+=3;

    rgb_d[0] = (double)it->r;
    rgb_d[1] = (double)it->g;
    rgb_d[2] = (double)it->g;
    rgb_d+=3;

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
  PointCloud<PointType>::Ptr ptr1 = cloud1->makeShared();
  PointCloud<PointType>::Ptr ptr2 = cloud2->makeShared();

  IterativeClosestPoint<PointType, PointType> icp;
  icp.setInputSource(ptr1);
  icp.setInputTarget(ptr2);
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

PointCloud<NormalType>* PointCloud_getNormals(PointCloud<PointType>* cloud, double ksearch = 10)
{
  PointCloud<NormalType>* normalMap = new PointCloud<NormalType> ();

  NormalEstimationOMP<PointType, NormalType> norm_est;
  norm_est.setKSearch (ksearch);
  norm_est.setInputCloud (cloud->makeShared());
  norm_est.compute (*normalMap);

  return normalMap;
}

PointCloud<NormalType>* PointCloud_normal_create()
{
  return new PointCloud<NormalType> ();

}

PointCloud<PointType>* PointCloud_pc_create()
{
  return new PointCloud<PointType> ();

}

PointCloud<PointType>* growRegions(PointCloud<PointType>* cloud, PointCloud<NormalType>* normals, THLongTensor* point_indices, THLongTensor* location_change, 
  double minClusterSize = 100, int numNeighbors = 30, double smoothnessThreshold = 15.0/180.0 * M_PI, double curvatureThreshold = 1.0) {
  search::Search<PointType>::Ptr tree = boost::shared_ptr<search::Search<PointType> > (new search::KdTree<PointType>);
  RegionGrowing<PointType, NormalType> reg;
  reg.setMinClusterSize (minClusterSize);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (numNeighbors); //set mu?
  reg.setInputCloud (cloud->makeShared());
  //reg.setIndices (indices);
  reg.setInputNormals (normals->makeShared());
  reg.setSmoothnessThreshold (smoothnessThreshold);
  reg.setCurvatureThreshold (curvatureThreshold);

  vector <PointIndices> clusters;
  reg.extract (clusters);

  cout << "Number of clusters is equal to " << clusters.size () << endl;
  cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;

  THLongTensor_resize1d(point_indices, cloud->height*cloud->width);
  THLongTensor_fill(point_indices,0);
  long * point_indices_d = THLongTensor_data(point_indices);

  THLongTensor_resize1d(location_change, clusters.size());
  THLongTensor_fill(location_change,0);
  long * location_change_d = THLongTensor_data(location_change);
  long counter = 0;

  for (int i=0; i<clusters.size(); i++) {
    for (int j=0; j<clusters[i].indices.size(); j++) {
      point_indices_d[0] = (clusters[i].indices[j]);
      point_indices_d++;
      counter++;
    }
    location_change_d[0] = counter;
    location_change_d++;
  }
  return new PointCloud<PointType>(*(reg.getColoredCloud ()));
}

PolygonMesh* greedy_proj_mesh(PointCloud<PointType>* cloud, PointCloud<NormalType>* normals, double mu = 3, 
  double radiusSearch =.3, double maximumNearestNeighbors = 32, double maxSurfaceAngle = M_PI/5, double minAngle=M_PI/10,
  double maxAngle = 2*M_PI/3)
{
  pcl::PointCloud<PointColorNormalType>::Ptr cloud_with_normals (new pcl::PointCloud<PointColorNormalType>);
  pcl::concatenateFields (*cloud->makeShared(), *normals->makeShared(), *cloud_with_normals);
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  pcl::search::KdTree<PointColorNormalType>::Ptr tree2 (new pcl::search::KdTree<PointColorNormalType>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<PointColorNormalType> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (radiusSearch);

  // Set typical values for the parameters
  gp3.setMu (mu);
  gp3.setMaximumNearestNeighbors (maximumNearestNeighbors);
  gp3.setMaximumSurfaceAngle(maxSurfaceAngle); // 45 degrees
  gp3.setMinimumAngle(minAngle); // 10 degrees
  gp3.setMaximumAngle(maxAngle); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();

  pcl::io::savePLYFile("greedy_proj_mesh.ply",triangles);

  std::cout << " + saved ply file" << std::endl;
  return new PolygonMesh(triangles);

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


PointCloud<DescriptorType>* PointCloud_computeDescriptors(PointCloud<PointType>* downsampled_cloud, 
  PointCloud<PointType>* orig_cloud, 
  PointCloud<NormalType>* normals, 
  double descr_radius = .02)
{
  SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
  PointCloud<DescriptorType>* desc = new PointCloud<DescriptorType> ();
  descr_est.setRadiusSearch (descr_radius);
  descr_est.setInputCloud (downsampled_cloud->makeShared());
  descr_est.setInputNormals (normals->makeShared());
  descr_est.setSearchSurface (orig_cloud->makeShared());
  descr_est.compute (*desc);
  return desc;

}


void PointCloud_computeTransformation(PointCloud<DescriptorType>* desc1,
  PointCloud<DescriptorType>* desc2,
  PointCloud<PointType>* cloud1_down,
  PointCloud<PointType>* cloud2_down,
  THDoubleTensor* transformations,
  THDoubleTensor* correspondences,
  double cg_size = 5,
  double cg_thresh = .01
  )
{
  //  Find Model-Scene Correspondences with KdTree
  //
  CorrespondencesPtr corrs (new Correspondences ());

  KdTreeFLANN<DescriptorType> match_search;
  match_search.setInputCloud (desc1->makeShared());

  //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
  for (size_t i = 0; i < desc2->size (); ++i)
  {
    vector<int> neigh_indices (1);
    vector<float> neigh_sqr_dists (1);
    if (!pcl_isfinite (desc2->at (i).descriptor[0])) //skipping NaNs
    {
      continue;
    }
    int found_neighs = match_search.nearestKSearch (desc2->at (i), 1, neigh_indices, neigh_sqr_dists);
    if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
    {
      Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
      corrs->push_back (corr);
    }
  }
  cout << "Correspondences found: " << corrs->size () << endl;

  vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
  vector<Correspondences> clustered_corrs;

  GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
  gc_clusterer.setGCSize (cg_size);
  gc_clusterer.setGCThreshold (cg_thresh);

  gc_clusterer.setInputCloud (cloud1_down->makeShared());
  gc_clusterer.setSceneCloud (cloud2_down->makeShared());
  gc_clusterer.setModelSceneCorrespondences (corrs);

  //gc_clusterer.cluster (clustered_corrs);
  gc_clusterer.recognize (rototranslations, clustered_corrs);


  THDoubleTensor_resize3d(transformations, rototranslations.size (),4,4);
  THDoubleTensor_resize1d(correspondences, rototranslations.size ());

  THDoubleTensor_fill(transformations,0);
  THDoubleTensor_fill(correspondences,0);


  double * transformations_d = THDoubleTensor_data(transformations);
  double * correspondences_d = THDoubleTensor_data(correspondences);

  for (size_t i = 0; i < rototranslations.size (); ++i)
  {
    cout << "\n    Instance " << i + 1 << ":" << endl;
    cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << endl;

    correspondences_d[0] = clustered_corrs[i].size ();

    // Print the rotation matrix and translation vector
    Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
    Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);

    printf ("\n");
    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
    printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
    printf ("\n");
    printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));

    for(int j=0; j<4; j++) {
      for(int k=0; k<4; k++) {
        transformations_d[j*4+k] = rototranslations[i](j,k);
      }
    }
    transformations_d += 16;
    correspondences_d++;
  }
}
  //


} // extern "C"
