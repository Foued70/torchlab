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

int
computePointCloudNormals(Ptr cloud, NPtr ncloud, int ksearch)
{
  pcl::NormalEstimation<PointT, PointNT> ne;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>());
  ne.setSearchMethod(tree);
  ne.setKSearch(ksearch);
  
  ne.setInputCloud(cloud);
  ne.compute(*ncloud);
  pcl::copyPointCloud(*cloud, *ncloud);
  
  return 0;
}

int
computePointCloudNormalsAtKeypoints(Ptr cloud, NPtr ncloud, SPtr kcloud, int ksearch)
{
  pcl::NormalEstimation<PointT, PointNT> ne;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>());
  ne.setSearchMethod(tree);
  ne.setKSearch(ksearch);
  
  Ptr ccloud (new PointCloud);
  pcl::copyPointCloud (*kcloud, *ccloud);
  
  ne.setSearchSurface (cloud);
  ne.setInputCloud(ccloud);
  ne.compute(*ncloud);
  pcl::copyPointCloud(*ccloud, *ncloud);
  
  return 0;
}

int getSIFTKeypoints (Ptr cloud, SPtr kcloud, float min_scale, int nr_octaves, int nr_scales_per_octave, float min_contrast)
{
  
  pcl::SIFTKeypoint<PointT, PointST> sift;
  sift.setSearchMethod (pcl::search::KdTree<PointT>::Ptr (new pcl::search::KdTree<PointT>));
  sift.setScales(min_scale, nr_octaves, nr_scales_per_octave);
  sift.setMinimumContrast(min_contrast);
  
  sift.setInputCloud(cloud);
  sift.compute(*kcloud);

  return (0);
}

void
computePFHFeatures (Ptr points, NPtr normals, float featureRadius,
                      pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors_out)
{
  pcl::PFHEstimation<PointT, PointNT, pcl::PFHSignature125> pfh_est;
  pfh_est.setSearchMethod (pcl::search::KdTree<PointT>::Ptr (new pcl::search::KdTree<PointT>));
  pfh_est.setRadiusSearch (featureRadius);

  pfh_est.setInputCloud (points);
  pfh_est.setInputNormals (normals);  

  pfh_est.compute (*descriptors_out);
}

void
computePFHFeaturesAtKeypoints (Ptr points, NPtr normals, SPtr keypoints, float featureRadius, DPtr descriptors_out)
{
  pcl::PFHEstimation<PointT, PointNT, pcl::PFHSignature125> pfh_est;
  pfh_est.setSearchMethod (pcl::search::KdTree<PointT>::Ptr (new pcl::search::KdTree<PointT>));
  pfh_est.setRadiusSearch (featureRadius);

  Ptr keypoints_xyzrgb (new PointCloud);
  pcl::copyPointCloud (*keypoints, *keypoints_xyzrgb);

  pfh_est.setSearchSurface (points);  
  pfh_est.setInputNormals (normals);  
  pfh_est.setInputCloud (keypoints_xyzrgb);

  pfh_est.compute (*descriptors_out);
}

int
findICP(Ptr cloud_tgt, Ptr cloud_src, NPtr ncloud_tgt, NPtr ncloud_src, Ptr final, IterativeClosestPoint * reg, double mcd, int iter, int riter)
{

  NPtr nres (new PointNCloud);
  reg->setInputCloud(ncloud_src);
  reg->setInputTarget(ncloud_tgt);
  reg->setTransformationEpsilon (1e-6);
  reg->setMaximumIterations(iter);
  reg->setRANSACIterations(riter);
  reg->setMaxCorrespondenceDistance(mcd);

  cout << "findICP: reg set " << endl;
  
  reg->align(*nres);
  
  cout << "findICP: Has converged: " << reg->hasConverged() << "; score: " << reg->getFitnessScore() << endl;
  
  pcl::transformPointCloud (*cloud_src, *final, reg->getFinalTransformation());
  
  return 0;
}

int
findSACIA(Ptr cloud_tgt, Ptr cloud_src, DPtr dcloud_tgt, DPtr dcloud_src, Ptr final, SACIA * reg, double mcd, int iter, int riter, double msd)
/*int
findSACIA(Ptr cloud_tgt, Ptr cloud_src, NPtr ncloud_tgt, NPtr ncloud_src, Ptr final, SACIA * reg, double mcd, int iter, int riter, double msd)*/
{

  reg->setInputCloud(cloud_src);
  reg->setInputTarget(cloud_tgt);
  /*reg->setSourceFeatures(ncloud_src);
  reg->setTargetFeatures(ncloud_tgt);*/
  reg->setSourceFeatures(dcloud_src);
  reg->setTargetFeatures(dcloud_tgt);
  reg->setTransformationEpsilon (1e-6);
  reg->setMaxCorrespondenceDistance(mcd);
  reg->setMaximumIterations(iter);
  reg->setRANSACIterations(riter);
  reg->setMinSampleDistance(msd);
  //reg->setNumberOfSamples(0.001*cloud_src->points.size());
  reg->setNumberOfSamples(50);
  
  cout << "findSACIA: numSamples " << reg->getNumberOfSamples() << endl;
  
  reg->align(*final);
  
  cout << "findSACIA: Has converged: " << reg->hasConverged() << "; score: " << reg->getFitnessScore() << endl;
  
  return 0;
}

int
getCorrespondences(NPtr src, NPtr tgt, DPtr desc_src, DPtr desc_tgt, pcl::Correspondences * corr, float curv_tol, float hist_tol)
{
  
  float reject = 999999999.9*250.0;
  for (int i = 0; i < src->size(); i++)
  {
    PointNT nt_src = src->points[i];
    PointDT dt_src = desc_src->points[i];
    pcl::Correspondence corr_tmp;
    float corr_dist = reject;
    for (int j = 0; j < tgt->size(); j++)
    {
      PointNT nt_tgt = tgt->points[j];
      PointDT dt_tgt = desc_tgt->points[j];
      float dist = 0.0;
      if (nt_src.curvature > nt_tgt.curvature - curv_tol && nt_src.curvature < nt_tgt.curvature + curv_tol)
      {
        dist =+ pow(nt_tgt.curvature - nt_src.curvature, 2);
        bool flag = true;
        for (int k = 0; k < 125 ; k++)
        {
          if (dt_src.histogram[k] < dt_tgt.histogram[k] - hist_tol || dt_src.histogram[k] > dt_tgt.histogram[k] + hist_tol)
          {
            flag = false;
            break;
          }
          dist =+ pow(dt_tgt.histogram[k] - dt_src.histogram[k], 2);
        }
        if(flag && dist < corr_dist)
        {
          corr_tmp = pcl::Correspondence(i,j,dist);
          corr_dist = dist;
        }
      }
    }
    if (corr_dist < reject)
    {
      int q = corr_tmp.index_query;
      int m = corr_tmp.index_match;
      float d = corr_dist;
      PointNT nt_tgt = tgt->points[m];
      cout << "correspondence: " << q << ":" << m << " [x:" << nt_src.x << ":" << nt_tgt.x << ", y:" << nt_src.y << ":" << nt_tgt.y << ", z:" << nt_src.z << ":" << nt_tgt.z << ", c:" << nt_src.curvature << ":" << nt_tgt.curvature << ", d: " << d << "]" << endl;
      corr->push_back(corr_tmp);
    }
  }
  cout << "done with this" << endl;
  return 0;
}

double
getNormalCorrespondenceScore(pcl::Correspondences corr, Ptr src, Ptr tgt, Eigen::Matrix4f Ti)
{
  double ret = 0;
  Ptr src_trans (new PointCloud);
  pcl::transformPointCloud(*src, *src_trans, Ti);
  
  for (int i = 0; i < corr.size(); i++)
  {
    PointT t = tgt->points[corr[i].index_match];
    PointT s = src_trans->points[corr[i].index_query];
    PointT o = src->points[corr[i].index_query];
    ret =+ sqrt(pow(t.x - s.x,2) + pow(t.y - s.y,2) + pow(t.z - s.z,2));
    
    cout << "i: " << i << ", tgt-ind: " << corr[i].index_match << ", src-ind: " << corr[i].index_query << endl;
    cout << "tgt: [" << t.x << ", " << t.y << ", " << t.z << "]" << endl;
    cout << "ori: [" << o.x << ", " << o.y << ", " << o.z << "]" << endl;
    cout << "src: [" << s.x << ", " << s.y << ", " << s.z << "]" << endl;
    cout << "dst: " << sqrt(pow(t.x - s.x,2) + pow(t.y - s.y,2) + pow(t.z - s.z,2)) << endl;
  }
  
  ret = ret/corr.size();
  return ret;
}

int registerMYFUNC(char * fname_tgt, char * fname_src, Ptr final, double voxelGridLeafSize, double sorMeanK, double sorStddevThresh, bool sorFilter, double rorRadius, int rorMinNeighbors, bool rorFilter, float siftMinScale, int siftNrOctaves, int siftNrScalesPerOctave, float siftMinContrast, float featureRadius, float corrCurvTol, float corrHistTol)
{
  Ptr cloud_tgt (new PointCloud);
  Ptr cloud_src (new PointCloud);
  
  // load pcd files into point clouds
  if(loadPCDFile(fname_tgt, cloud_tgt) < 0)
  {
    cout << "registerSAC: Trouble loading target file." << endl;
    return -1;
  }
  
  if(loadPCDFile(fname_src, cloud_src) < 0)
  {
    cout << "registerSAC: Trouble loading source file." << endl;
    return -1;
  }
  
  Ptr fcloud_tgt (new PointCloud);
  Ptr fcloud_src (new PointCloud);
  
  // filter point clouds
  if(filterPointCloud(cloud_tgt, fcloud_tgt, voxelGridLeafSize, sorMeanK, sorStddevThresh, rorRadius, rorMinNeighbors, sorFilter, rorFilter) < 0)
  {
    cout << "registerSAC: Trouble filtering target point cloud" << endl;
    return -1;
  }
  
  if(filterPointCloud(cloud_src, fcloud_src, voxelGridLeafSize, sorMeanK, sorStddevThresh, rorRadius, rorMinNeighbors, sorFilter, rorFilter) < 0)
  {
    cout << "registerSAC: Trouble filtering source point cloud" << endl;
    return -1;
  }
  
  pcl::io::savePCDFileBinary("../f_tgt.pcd", *fcloud_tgt);
  pcl::io::savePCDFileBinary("../f_src.pcd", *fcloud_src);
  
  NPtr ncloud_tgt (new PointNCloud);
  NPtr ncloud_src (new PointNCloud);
  
  SPtr kcloud_src (new PointSCloud);
  SPtr kcloud_tgt (new PointSCloud);
  
  NPtr nkcloud_tgt (new PointNCloud);
  NPtr nkcloud_src (new PointNCloud);
  
  DPtr dkcloud_tgt (new PointDCloud);
  DPtr dkcloud_src (new PointDCloud);
  
  Ptr ccloud_tgt (new PointCloud);
  Ptr ccloud_src (new PointCloud);

  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity ();
  
  pcl::registration::TransformationEstimationSVD<PointT, PointT> te;
  
  for (int r = 0; r < 5; r++)
  {
  
    // compute point cloud normals
    computePointCloudNormals(fcloud_tgt, ncloud_tgt, 50);
    computePointCloudNormals(fcloud_src, ncloud_src, 50);
  
    pcl::io::savePCDFileBinary("../n_tgt.pcd", *ncloud_tgt);
    pcl::io::savePCDFileBinary("../n_src.pcd", *ncloud_src);
  
  
    // extract keypoints
    /*getSIFTKeypoints (fcloud_tgt, kcloud_tgt, siftMinScale, siftNrOctaves, siftNrScalesPerOctave, siftMinContrast);
    getSIFTKeypoints (fcloud_src, kcloud_src, siftMinScale, siftNrOctaves, siftNrScalesPerOctave, siftMinContrast);*/
    
    Ptr kkcloud_tgt (new PointCloud);
    Ptr kkcloud_src (new PointCloud);
    
    filterPointCloud(fcloud_tgt, kkcloud_tgt, 0.25, sorMeanK, sorStddevThresh, rorRadius, rorMinNeighbors, false, false);
    filterPointCloud(fcloud_src, kkcloud_src, 0.25, sorMeanK, sorStddevThresh, rorRadius, rorMinNeighbors, false, false);
    
    pcl::copyPointCloud(*kkcloud_src, *kcloud_src);
    pcl::copyPointCloud(*kkcloud_tgt, *kcloud_tgt);
    
    cout << kkcloud_src->size() << endl;
    cout << kcloud_src->size() << endl;
  
    pcl::io::savePCDFileBinary("../k_tgt.pcd", *kcloud_tgt);
    pcl::io::savePCDFileBinary("../k_src.pcd", *kcloud_src);

    // compute normals at keypoints
  
    computePointCloudNormalsAtKeypoints(fcloud_tgt, nkcloud_tgt, kcloud_tgt, 50);
    computePointCloudNormalsAtKeypoints(fcloud_src, nkcloud_src, kcloud_src, 50);
  
    pcl::io::savePCDFileBinary("../nk_tgt.pcd", *nkcloud_tgt);
    pcl::io::savePCDFileBinary("../nk_src.pcd", *nkcloud_src);
  
  
    // compute features at keypoints
    computePFHFeaturesAtKeypoints(fcloud_tgt, ncloud_tgt, kcloud_tgt, featureRadius, dkcloud_tgt);
    computePFHFeaturesAtKeypoints(fcloud_src, ncloud_src, kcloud_src, featureRadius, dkcloud_src);
  
    pcl::copyPointCloud(*kcloud_tgt, *ccloud_tgt);
    pcl::copyPointCloud(*kcloud_src, *ccloud_src);
  
    pcl::Correspondences corr;
  
    getCorrespondences(nkcloud_src, nkcloud_tgt, dkcloud_src, dkcloud_tgt, &corr, corrCurvTol, corrHistTol);
  
    cout << corr.size() << endl;
  
    Eigen::Matrix4f Tr = Eigen::Matrix4f::Identity ();
  
    te.estimateRigidTransformation(*ccloud_src, *ccloud_tgt, corr, Tr);
  
    Ti = Tr * Ti;
  
    cout << "Temp Transformation: \n" << Tr << endl;
    double score = getNormalCorrespondenceScore(corr, ccloud_src, ccloud_tgt, Tr);
    cout << score << endl;
    pcl::transformPointCloud (*fcloud_src, *fcloud_src, Tr);
  
  }
  
  cout << "Final Transformation: \n" << Ti << endl;
  
  pcl::transformPointCloud (*cloud_src, *final, Ti);
  
  pcl::io::savePCDFileBinary("../final_fc.pcd", *fcloud_src);
  pcl::io::savePCDFileBinary("../final.pcd", *final);
  
  return 0;
}

int
main (int argc, char** argv)
{
  
  char * fname_tgt = "";
  char * fname_src = "";
  char * fname_out = "";
  double voxelGridLeafSize = 0.025;
  double sorMeanK = 25;
  double sorStddevThresh = 1.0;
  bool sorFilter = true;
  double rorRadius = 1.5;
  int rorMinNeighbors = 5000;
  bool rorFilter = false;
  float siftMinScale = 0.01;
  int siftNrOctaves = 3;
  int siftNrScalesPerOctave = 3;
  float siftMinContrast = 10.0;
  float featureRadius = 0.08;
  float corrCurvTol = 0.05;
  float corrHistTol = 2.5;
  
  int a = 1;
  
  if ((strcmp(argv[a], "-h") == 0) or (strcmp(argv[a], "-help") == 0))
  {
    cout << "PCL_REGISTER HELP MENU: " << endl;
    cout << "           -targetname: " << fname_tgt << endl;
    cout << "           -sourcename: " << fname_src << endl;
    cout << "           -outputname: " << fname_out << endl;
    cout << "    -voxelGridLeafSize: " << voxelGridLeafSize << endl;
    cout << "             -sorMeanK: " << sorMeanK << endl;
    cout << "      -sorStddevThresh: " << sorStddevThresh << endl;
    cout << "            -sorFilter: " << sorFilter << endl;
    cout << "            -rorRadius: " << rorRadius << endl;
    cout << "      -rorMinNeighbors: " << rorMinNeighbors << endl;
    cout << "            -rorFilter: " << rorFilter << endl;
    cout << "         -siftMinScale: " << siftMinScale << endl;
    cout << "        -siftNrOctaves: " << siftNrOctaves << endl;
    cout << "-siftNrScalesPerOctave: " << siftNrScalesPerOctave << endl;
    cout << "      -siftMinContrast: " << siftMinContrast << endl;
    cout << "        -featureRadius: " << featureRadius << endl;
    cout << "          -corrCurvTol: " << corrCurvTol << endl;
    cout << "          -corrHistTol: " << corrHistTol << endl;
    return 0;
  }
  
  while(a+1 < argc)
  {
    if (strcmp(argv[a], "-targetname") == 0)
      fname_tgt = argv[a+1];
    else if (strcmp(argv[a], "-sourcename") == 0)
      fname_src = argv[a+1];
    else if (strcmp(argv[a], "-outputname") == 0)
      fname_out = argv[a+1];
    else if (strcmp(argv[a], "-voxelGridLeafSize") == 0)
      voxelGridLeafSize = atof(argv[a+1]);
    else if (strcmp(argv[a], "-sorMeanK") == 0)
      sorMeanK = atof(argv[a+1]);
    else if (strcmp(argv[a], "-sorStddevThresh") == 0)
      sorStddevThresh = atof(argv[a+1]);
    else if (strcmp(argv[a], "-sorFilter") == 0)
      sorFilter = atoi(argv[a+1]);
    else if (strcmp(argv[a], "-rorRadius") == 0)
      rorRadius = atof(argv[a+1]);
    else if (strcmp(argv[a], "-rorMinNeighbors") == 0)
      rorMinNeighbors = atof(argv[a+1]);
    else if (strcmp(argv[a], "-rorFilter") == 0)
      rorFilter = atoi(argv[a+1]);
    else if (strcmp(argv[a], "-siftMinScale") == 0)
      siftMinScale = atof(argv[a+1]);
    else if (strcmp(argv[a], "-siftNrOctaves") == 0)
      siftNrOctaves = atoi(argv[a+1]);
    else if (strcmp(argv[a], "-siftNrScalesPerOctave") == 0)
      siftNrScalesPerOctave = atoi(argv[a+1]);
    else if (strcmp(argv[a], "-siftMinContrast") == 0)
      siftMinContrast = atof(argv[a+1]);
    else if (strcmp(argv[a], "-featureRadius") == 0)
      featureRadius = atof(argv[a+1]);
    else if (strcmp(argv[a], "-corrCurvTol") == 0)
      corrCurvTol = atof(argv[a+1]);
    else if (strcmp(argv[a], "-corrHistTol") == 0)
      corrHistTol = atof(argv[a+1]);
    
    else
      cout << "Unknow option " << argv[a] << endl;
    
    a += 2;
  }
  
  if (strcmp(fname_tgt, "") == 0 or strcmp(fname_src, "") == 0 )
  {
    cout << "main: oh no! \n";
    return -1;
  }
  
  Ptr output_cloud (new PointCloud);
  int ret = registerMYFUNC(fname_tgt, fname_src, output_cloud, voxelGridLeafSize, sorMeanK, sorStddevThresh, sorFilter, rorRadius, rorMinNeighbors, rorFilter, siftMinScale, siftNrOctaves, siftNrScalesPerOctave, siftMinContrast, featureRadius, corrCurvTol, corrHistTol);
  
  if (strcmp(fname_out, "") != 0)
  {
    cout << "output cloud has " << output_cloud->points.size() << " points" << endl;
    pcl::io::savePCDFileBinary(fname_out, *output_cloud);
  }
  
  return ret;
}