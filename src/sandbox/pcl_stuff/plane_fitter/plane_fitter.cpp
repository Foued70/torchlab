#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud< PointT > PointCloud;
typedef PointCloud::Ptr Ptr;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud< PointNT > PointNCloud;
typedef PointNCloud::Ptr NPtr;

int
load1PCDFile( char * fname, Ptr cloud)
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

int
main (int argc, char** argv)
{
  if (argc < 3)
  {
    cout << "oh no! \n";
    return -1;
  }
  
  char* fname_in = argv[1];
  char* bname_ou = argv[2];
  
  int ksearch = 100;
  double nDistWght = 0.25;
  double distThrsh = 0.05;
  double leafSize = 0.025;
  double meanK = 25;
  double stddevThresh = 1.0;
  double radiusSearch = 0.1;
  double mniRadius = 0;
  int filterRepeat = 1;
  
  int a = 3;
  
  while(a+1 < argc)
  {
    cout << "arg # " << a << " arg str " << argv[a] << " arg val " << argv[a+1] << endl;
    if (strcmp(argv[a], "-ksearch") == 0)
    {
      ksearch = atoi(argv[a+1]);
      cout << "setting ksearch to " << ksearch << endl;
    }
    if (strcmp(argv[a], "-normalDistWeight") == 0)
    {
      nDistWght = atof(argv[a+1]);
      cout << "setting normDistWeight to " << nDistWght << endl;
    }
    if (strcmp(argv[a], "-distThresh") == 0)
    {
      distThrsh = atof(argv[a+1]);
      cout << "setting distThresh to " << distThrsh << endl;
    }
    if (strcmp(argv[a], "-leafSize") == 0)
    {
      leafSize = atof(argv[a+1]);
      cout << "setting leafSize to " << leafSize << endl;
    }
    if (strcmp(argv[a], "-meanK") == 0)
    {
      meanK = atof(argv[a+1]);
      cout << "setting meanK to " << meanK << endl;
    }
    if (strcmp(argv[a], "-stddevThresh") == 0)
    {
      stddevThresh = atof(argv[a+1]);
      cout << "setting stddevThresh to " << stddevThresh << endl;
    }
    if (strcmp(argv[a], "-radiusSearch") == 0)
    {
      radiusSearch = atof(argv[a+1]);
      cout << "setting radiusSearch to " << radiusSearch << endl;
    }
    if (strcmp(argv[a], "-mniRadius") == 0)
    {
      mniRadius = atof(argv[a+1]);
      cout << "setting mniRadius to " << mniRadius << endl;
    }
    if (strcmp(argv[a], "-filterRepeat") == 0)
    {
      filterRepeat = atoi(argv[a+1]);
      if (filterRepeat < 0)
      {
        filterRepeat = 0;
      }
      cout << "setting filterRepeat to " << filterRepeat << endl;
    }
    a += 2;
  }
  
  // All the objects needed
  pcl::PassThrough<PointT> pass;
  pcl::StatisticalOutlierRemoval<PointT> sor;
  pcl::RadiusOutlierRemoval<PointT> ror;
  pcl::VoxelGrid<PointT> vg;
  pcl::NormalEstimation<PointT, PointNT> ne;
  pcl::SACSegmentationFromNormals<PointT, PointNT> seg;
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<PointNT> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  
  // Datasets
  Ptr cloud_in (new PointCloud);
  Ptr cloud_ou_plane (new PointCloud);
  Ptr cloud_filtered (new PointCloud);
  NPtr cloud_normals (new PointNCloud);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  vector<int> nonNaNindices;
  
  // Read data
  if (load1PCDFile(fname_in, cloud_in) < 0)
  {
    cout << "main: Trouble loading file " << fname_in << endl;
    return -1;;
  }
  cout << "Point cloud data: " << cloud_in->points.size() << " points" << endl;
  
  // Build filter to remove spurious NaNs
  
  pcl::removeNaNFromPointCloud(*cloud_in, *cloud_filtered, nonNaNindices);
  
  vg.setInputCloud(cloud_filtered);
  vg.setLeafSize(leafSize, leafSize, leafSize);
  vg.setDownsampleAllData(true);
  vg.filter(*cloud_filtered);
  
  cout << "PointCloud after vg filtering has: " << cloud_filtered->points.size () << " data points." << endl;
  
  int f = 0;
  while (f < filterRepeat)
  {
    /*pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-45.0, -35.0);
    pass.filter (*cloud_filtered);
  
    cout << "PointCloud after pass filtering has: " << cloud_filtered->points.size () << " data points." << endl;*/
  
    ror.setInputCloud(cloud_filtered);
    ror.setRadiusSearch(radiusSearch);
    ror.setMinNeighborsInRadius(mniRadius);
    ror.filter(*cloud_filtered);
  
    cout << "PointCloud after ror filtering has: " << cloud_filtered->points.size () << " data points." << endl;
  
    sor.setInputCloud (cloud_filtered);
    sor.setMeanK (meanK);
    sor.setStddevMulThresh (stddevThresh);
    sor.filter (*cloud_filtered);
  
    cout << "PointCloud after sor filtering has: " << cloud_filtered->points.size () << " data points." << endl;
    
    f++;
  }
  
  stringstream fout;
  fout << bname_ou << "_filtered_cloud.pcd";
  cout << "saving filtered to " << fout.str() << endl;
  pcl::io::savePCDFileBinary(fout.str(), *cloud_filtered);
  
  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (ksearch);
  ne.compute (*cloud_normals);
  
  // Create the segmentation object for planar model and set all paramaters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight(nDistWght);
  seg.setDistanceThreshold (distThrsh);
  seg.setMaxIterations(500);
  
  int i = 0;
  int numPoints = cloud_filtered->points.size();
  
  while( cloud_filtered->points.size() > 0.3 * numPoints)
  {
  
    seg.setInputCloud (cloud_filtered);
    seg.setInputNormals(cloud_normals);
    // optain plane inliers and coefficients
    seg.segment (*inliers, *coefficients);
  
    if (inliers->indices.size () < 0.0025 * numPoints)
    {
      cout << "Found " << inliers->indices.size() << " is less than threshold " << 0.0025 * numPoints << endl;
      break;
    }
  
    cout << "Model coefficients: " << coefficients->values[0] << " "
                                  << coefficients->values[1] << " "
                                  << coefficients->values[2] << " "
                                  << coefficients->values[3] << endl;
  
    cout << "Model inliers: " << inliers->indices.size () << endl;
  
    // Extract the planar inliers from the input cloud
    extract.setInputCloud(cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative(false);
  
    // Write planar inliers to disk
    extract.filter(*cloud_ou_plane);
    cout << "PointCloud representing the planar component: " << cloud_ou_plane->points.size() << " data pointes" << endl;
  
    stringstream ss;
    ss << bname_ou <<"_plane_" << i << ".pcd";
    cout << "saving plane to " << ss.str() << endl;
    pcl::io::savePCDFileBinary(ss.str(), *cloud_ou_plane);
  
    // Remove the planar inliers, extract the rest
    extract.setNegative(true);
    extract.filter(*cloud_filtered);
    extract_normals.setNegative(true);
    extract_normals.setInputCloud(cloud_normals);
    extract_normals.setIndices(inliers);
    extract_normals.filter(*cloud_normals);
    
    cout << "PointCloud representing the leftover component: " << cloud_filtered->points.size() << " data pointes" << endl;
  
    i++;
  }
  
  stringstream ss;
  ss << bname_ou << "_rest_" << i << ".pcd";
    cout << "saving leftovers to " << ss.str() << endl;
  pcl::io::savePCDFileBinary(ss.str(), *cloud_filtered);
  
  return 0;
}