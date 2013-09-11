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

int registerMYFUNC(char * fname_src, char * fname_transformation, char * fname_save, double scale)
{
  Ptr cloud_src (new PointCloud);
  Ptr cloud_final (new PointCloud);

  // load pcd files into point clouds
  if(loadPCDFile(fname_src, cloud_src) < 0)
  {
    cout << "registerSAC: Trouble loading source file." << endl;
    return -1;
  }

  Eigen::Matrix4f transformMatrix = Eigen::Matrix4f::Identity ();
  
  ifstream infile;   

  int num = 0; // num must start at 0
  infile.open(fname_transformation);// file containing numbers in 3 columns 
  if(infile.fail()) // checks to see if file opended 
  { 
    cout << "error" << endl; 
    return 1; // no point continuing if the file didn't open...
  } 
  //double a1, a2, a3, a4, a5, a6, a7, a8, a9;
  //infile >> a1 >> a2 >> a3 >> a4 >> a5 >> a6 >> a7 >> a8>> a9;

  string s;
  getline( infile, s );

  istringstream ss( s );
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


  getline( infile, s );

  istringstream ss2( s );
  getline( ss2, s1, ',' );
  double src_shift_x = atof(s1.c_str())/2; 

  getline( ss2, s1, ',' );
  double src_shift_y = atof(s1.c_str())/2; 


  getline( infile, s );

  istringstream ss3( s );
  getline( ss3, s1, ',' );
  double dest_shift_x = atof(s1.c_str())/2; 

  getline( ss3, s1, ',' );
  double dest_shift_y = atof(s1.c_str())/2; 

  infile.close(); 



  //rotation - don't forget to switch x and y for lua -> opencv
  transformMatrix (0,0) = a1;
  transformMatrix (1,0) = -a2;
  transformMatrix (0,1) = -a4;
  transformMatrix (1,1) = a5;

  //translation
  transformMatrix (0,3) = a3;
  transformMatrix (1,3) = a6;

  //this is for the scale we first have to do!
  Eigen::Matrix4f scaleMatrix = Eigen::Matrix4f::Identity ();
  scaleMatrix(0,0) = 1/scale;
  scaleMatrix(1,1) = 1/scale;
  //and shift so center is where center is at!


    Eigen::Matrix4f translationMatrix1 = Eigen::Matrix4f::Identity ();
  translationMatrix1(0,3) = src_shift_x;
  translationMatrix1(1,3) = src_shift_y;

  Eigen::Matrix4f inverseScaleMatrix = Eigen::Matrix4f::Identity ();
  inverseScaleMatrix(0,0) = scale;
  inverseScaleMatrix(1,1) = scale;

    Eigen::Matrix4f translationMatrix2 = Eigen::Matrix4f::Identity ();

  translationMatrix2(0,3) = -dest_shift_x;
  translationMatrix2(1,3) = -dest_shift_y;

  pcl::transformPointCloud (*cloud_src, *cloud_final, inverseScaleMatrix*translationMatrix2*transformMatrix*translationMatrix1*scaleMatrix);
  
  pcl::io::savePCDFileBinary(fname_save, *cloud_final);
  
  return 0;
}

int
main (int argc, char** argv)
{
  
  char * fname_source = "";
  char * fname_transformation = "";
  char * fname_save = "";
  double scale = .015;

  int a = 1;
  if (argc > 1 && ((strcmp(argv[a], "-h") == 0) or (strcmp(argv[a], "-help") == 0)))
  {
    cout << "PCL_REGISTER HELP MENU: " << endl;
    cout << "           --source: " << fname_source << endl;
    cout << "           --transformation: " << fname_transformation << endl;
    cout << "           --output: " << fname_save << endl;
    cout << "           --scale: " << scale << endl;


    return 0;
  }
  while(a+1 < argc)
  {
    if (strcmp(argv[a], "--source") == 0)
      fname_source = argv[a+1];
    else if (strcmp(argv[a], "--transformation") == 0)
      fname_transformation = argv[a+1];
    else if (strcmp(argv[a], "--output") == 0)
      fname_save = argv[a+1];
    else if (strcmp(argv[a], "--scale") == 0)
      scale = atof(argv[a+1]);
    else
      cout << "Unknow option " << argv[a] << endl;
    
    a += 2;
  }
  if (strcmp(fname_source, "") == 0 or strcmp(fname_transformation, "") == 0  or  strcmp(fname_save, "") == 0 )
  {
    cout << "main: oh no! \n";
    return -1;
  }
  
  int ret = registerMYFUNC(fname_source, fname_transformation, fname_save, scale);
  
  return ret;
}