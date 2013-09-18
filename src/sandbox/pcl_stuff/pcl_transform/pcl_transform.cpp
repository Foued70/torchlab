#include <vector>

#include <Eigen/Core>

#include <iostream>

#include <pcl/io/pcd_io.h>

#include <pcl/correspondence.h>

#include <boost/make_shared.hpp>

#include <pcl/point_types.h>
#include <pcl/point_representation.h>
#include <pcl/point_cloud.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>

#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/keypoints/keypoint.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/keypoints/sift_keypoint.h>

using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZRGBNormal PointNT;
typedef pcl::PointWithScale PointST;
typedef pcl::PFHSignature125 PointDT;
typedef pcl::PointCloud< PointT > PointCloud;
typedef pcl::PointCloud< PointNT > PointNCloud;
typedef pcl::PointCloud< PointST > PointSCloud;
typedef pcl::PointCloud< PointDT > PointDCloud;
typedef PointCloud::Ptr Ptr;
typedef PointNCloud::Ptr NPtr;
typedef PointSCloud::Ptr SPtr;
typedef PointDCloud::Ptr DPtr;
typedef pcl::IterativeClosestPoint< PointNT, PointNT > IterativeClosestPoint;
typedef pcl::SampleConsensusInitialAlignment<PointT, PointT, PointDT> SACIA;

int
loadPCDFile( char * fname, Ptr cloud)
{
  if (pcl::io::loadPCDFile<PointT> (fname, *cloud) == -1)
  {
    PCL_ERROR ("load1PCDFile: Couldn't read file %s \n", fname);
    return (-1);
  }
  
  cout << "load1PCDFile: Loaded "
            << cloud->width * cloud->height
            << " data points from " << fname << "."
            << endl;

  return 0;
}
int writeMatrixToFile(Eigen::Matrix4f mat, char * fname) {
  ofstream preicp_transform (fname);
  for(int i=0; i<4; i++) {
    for(int j=0; j<4; j++) {
        preicp_transform << setprecision(10) << mat(i,j) << (!(i==3 && j==3)) ? "," : "";
    }
  }
  preicp_transform << "\n";
  preicp_transform.close();

  return 0;
}

int
filterPointCloud(Ptr cloud, Ptr fcloud, double leafsize, double meanK, double stddevThresh, double radius, int minNeighbors , bool filter_sor, bool filter_ror)
{
  pcl::StatisticalOutlierRemoval<PointT> sor;
  pcl::RadiusOutlierRemoval<PointT> ror;
  pcl::VoxelGrid<PointT> vg;
  
  // Build filter to remove spurious NaNs
  
  vg.setLeafSize(leafsize, leafsize, leafsize);
  vg.setDownsampleAllData(true);
  
  vg.setInputCloud(cloud);
  vg.filter(*fcloud);

  if (filter_sor)
  {
    sor.setMeanK (meanK);
    sor.setStddevMulThresh (stddevThresh);
    
    sor.setInputCloud (fcloud);
    sor.filter (*fcloud);
  }
  
  if (filter_ror)
  {
    ror.setRadiusSearch(radius);
    ror.setMinNeighborsInRadius(minNeighbors);
    
    ror.setInputCloud(fcloud);
    ror.filter(*fcloud);
  }
  
  cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << endl;
  cout << "PointCloud  after filtering has: " << fcloud->points.size () << " data points." << endl;
  
  if (fcloud->points.size() == 0)
  {
    return -1;
  }
  
  return 0;
}


int transformAndICP(char * fname_source, char * fname_destination, char * transformation, char * fname_save_preicp, 
  char * fname_save_poisticp, char * fname_save_preicp_transform, char * fname_save_posticp_transform, double scale,
  double icp_correspondence, double icp_max_iterations, double icp_ransac_iterations, double icp_transform_eps, double zshift)
{
  Ptr cloud_src (new PointCloud);
  Ptr cloud_src_post_transform(new PointCloud);

  // load pcd files into point clouds
  if(loadPCDFile(fname_source, cloud_src) < 0)
  {
    cout << "registerSAC: Trouble loading source file." << endl;
    return -1;
  }

  Eigen::Matrix4f transformMatrix = Eigen::Matrix4f::Identity ();
  

  istringstream ss( transformation );
  string s1;
  getline( ss, s1, ',' );
  double a1 = atof(s1.c_str()); 

  getline( ss, s1, ',' );
  double a2 = atof(s1.c_str()); 

  getline( ss, s1, ',' );
  double a3 = atof(s1.c_str()); 

  getline( ss, s1, ',' );
  double a4 = atof(s1.c_str()); 

  getline( ss, s1, ',' );
  double a5 = atof(s1.c_str()); 

  getline( ss, s1, ',' );
  double a6 = atof(s1.c_str()); 

  getline( ss, s1, ',' );
  double a7 = atof(s1.c_str()); 
  getline( ss, s1, ',' );
  double a8 = atof(s1.c_str()); 
  getline( ss, s1, ',' );
  double a9 = atof(s1.c_str()); 


  //rotation - don't forget to switch x and y for lua -> opencv
  transformMatrix (0,0) = a1;
  transformMatrix (1,0) = a2;
  transformMatrix (0,1) = a4;
  transformMatrix (1,1) = a5;

  //translation
  transformMatrix (0,3) = a3;
  transformMatrix (1,3) = a6;

  //this is for the scale we first have to do!
  Eigen::Matrix4f scaleMatrix = Eigen::Matrix4f::Identity ();
  scaleMatrix(0,0) = 1/scale;
  scaleMatrix(1,1) = 1/scale;
  //and shift so center is where center is at!


  Eigen::Matrix4f inverseScaleMatrix = Eigen::Matrix4f::Identity ();
  inverseScaleMatrix(0,0) = scale;
  inverseScaleMatrix(1,1) = scale;
  inverseScaleMatrix(2,3) = zshift; //add the shift at end

  Eigen::Matrix4f combined = inverseScaleMatrix*transformMatrix*scaleMatrix;

  pcl::transformPointCloud (*cloud_src, *cloud_src_post_transform, combined);
  writeMatrixToFile(combined, fname_save_preicp_transform);

  pcl::io::savePCDFileBinary(fname_save_preicp, *cloud_src_post_transform);
  
  //now do icp
  Ptr cloud_destination (new PointCloud);
  if(loadPCDFile(fname_destination, cloud_destination) < 0)
  {
    cout << "registerSAC: Trouble loading source file." << endl;
    return -1;
  }

  Ptr fcloud_src_post_transform (new PointCloud);
  Ptr fcloud_destination (new PointCloud);

  double voxelGridLeafSize = 0.025;
  double sorMeanK = 25;
  double sorStddevThresh = 1.0;
  bool sorFilter = true;
  double rorRadius = 1.5;
  int rorMinNeighbors = 5000;
  bool rorFilter = false;
 // filter point clouds
  if(filterPointCloud(cloud_src_post_transform, fcloud_src_post_transform, voxelGridLeafSize, sorMeanK, sorStddevThresh, rorRadius, rorMinNeighbors, sorFilter, rorFilter) < 0)
  {
    cout << "registerSAC: Trouble filtering target point cloud" << endl;
    return -1;
  }
   // filter point clouds
  if(filterPointCloud(cloud_destination, fcloud_destination, voxelGridLeafSize, sorMeanK, sorStddevThresh, rorRadius, rorMinNeighbors, sorFilter, rorFilter) < 0)
  {
    cout << "registerSAC: Trouble filtering target point cloud" << endl;
    return -1;
  }

  pcl::IterativeClosestPoint<PointT, PointT>  icp;
  icp.setInputCloud(cloud_src_post_transform);
  icp.setInputTarget(cloud_destination);  
  icp.setMaxCorrespondenceDistance (icp_correspondence);
      // Set the maximum number of iterations (criterion 1)
  icp.setMaximumIterations (icp_max_iterations);
  icp.setRANSACIterations(icp_ransac_iterations);

      // Set the transformation epsilon (criterion 2)
  icp.setTransformationEpsilon (icp_transform_eps);
      // Set the euclidean distance difference epsilon (criterion 3)

  Ptr final_posticip (new PointCloud);

cout << "preallign" << endl;
//  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(*final_posticip);
cout << "post" << endl;

  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;

  writeMatrixToFile(icp.getFinalTransformation()*combined, fname_save_posticp_transform);

  pcl::io::savePCDFileBinary(fname_save_poisticp, *final_posticip);

  return 0;
}

int
main (int argc, char** argv)
{
  
  char * fname_source = "";
  char * fname_destination= "";
  char * fname_transformation = "";
  char * fname_save_preicp = "";
  char * fname_save_posticp = "";
  char * fname_save_preicp_transform = "";
  char * fname_save_posticp_transform = "";
  double zshift = 0;
  double scale = .015;
  double icp_correspondence = .2;
  double icp_max_iterations = 100;
  double icp_ransac_iterations = 10000;
  double icp_transform_eps = 1e-6;

  int a = 1;
  if (argc > 1 && ((strcmp(argv[a], "-h") == 0) or (strcmp(argv[a], "-help") == 0)))
  {
    cout << "PCL_REGISTER HELP MENU: " << endl;
    cout << "           --source: " << fname_source << endl; //first image pcd filename
    cout << "           --destination: " << fname_destination << endl; // second image pcd filename 
    cout << "           --transformation: " << fname_transformation << endl; //guessed/rough transformation .txt file
    cout << "           --output_preicp: " << fname_save_preicp << endl; //save file for output after just applying transformation in 3d (pcd file)
    cout << "           --output_preicp_transform: " << fname_save_preicp_transform << endl; //the transformation in 3d that takes the first pcd to the second
    cout << "           --output_posticp: " << fname_save_posticp << endl; //transformed source cloud after initial transformation and icp
    cout << "           --output_posticp_transform: " << fname_save_posticp_transform << endl; //transformation that takes source to post-icp point cloud
    cout << "           --zshift: " << zshift << endl; //transformation that takes source to post-icp point cloud
    cout << "           --scale: " << scale << endl; //scale used to flatten
    cout << "           --icp_correspondence: " << icp_correspondence << endl; //parameter of icp
    cout << "           --icp_max_iterations: " << icp_max_iterations << endl; //parameter of icp
    cout << "           --icp_ransac_iterations: " << icp_ransac_iterations << endl; //parameter of icp
    cout << "           --icp_transform_eps: " << icp_transform_eps << endl; //parameter of icp
    
    return 0;
  }
  while(a+1 < argc)
  {
    if (strcmp(argv[a], "--source") == 0)
      fname_source = argv[a+1];
    else if (strcmp(argv[a], "--destination") == 0)
      fname_destination = argv[a+1];
    else if (strcmp(argv[a], "--transformation") == 0)
      fname_transformation = argv[a+1];
    else if (strcmp(argv[a], "--output_preicp") == 0)
      fname_save_preicp = argv[a+1];
    else if (strcmp(argv[a], "--output_preicp_transform") == 0)
      fname_save_preicp_transform = argv[a+1];
    else if (strcmp(argv[a], "--output_posticp") == 0)
      fname_save_posticp = argv[a+1];
    else if (strcmp(argv[a], "--output_posticp_transform") == 0)
      fname_save_posticp_transform = argv[a+1];
    else if (strcmp(argv[a], "--scale") == 0)
      scale = atof(argv[a+1]);
    else if (strcmp(argv[a], "--icp_correspondence") == 0)
      icp_correspondence = atof(argv[a+1]);    
    else if (strcmp(argv[a], "--icp_max_iterations") == 0)
      icp_max_iterations = atof(argv[a+1]);
    else if (strcmp(argv[a], "--icp_ransac_iterations") == 0)
      icp_ransac_iterations = atof(argv[a+1]);
    else if (strcmp(argv[a], "--icp_transform_eps") == 0)
      icp_transform_eps = atof(argv[a+1]);
    else if (strcmp(argv[a], "--zshift") == 0)
      zshift = atof(argv[a+1]);
    else
      cout << "Unknow option " << argv[a] << endl;
    
    a += 2;
  }
  if (strcmp(fname_source, "") == 0 or strcmp(fname_destination, "") == 0 or strcmp(fname_transformation, "") == 0  or  strcmp(fname_save_preicp, "") == 0 or 
        strcmp(fname_save_posticp, "") == 0 or strcmp(fname_save_preicp_transform, "") == 0  or  strcmp(fname_save_posticp_transform, "") == 0)
  {
    cout << "main: oh no! \n";
    return -1;
  }
  
  int ret = transformAndICP(fname_source, fname_destination, fname_transformation, fname_save_preicp, fname_save_posticp, fname_save_preicp_transform, 
    fname_save_posticp_transform, scale, icp_correspondence, icp_max_iterations, icp_ransac_iterations, icp_transform_eps, zshift);   

  return ret;
}