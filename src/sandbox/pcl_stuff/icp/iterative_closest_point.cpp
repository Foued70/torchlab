#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud< PointT > PointCloud;
typedef PointCloud::Ptr Ptr;

// int
// findICP(Ptr cloud_tgt, Ptr cloud_src, NPtr ncloud_tgt, NPtr ncloud_src, Ptr final, IterativeClosestPoint * reg, double mcd, int iter, int riter)
// {
// 
//   NPtr nres (new PointNCloud);
//   reg->setInputCloud(ncloud_src);
//   reg->setInputTarget(ncloud_tgt);
//   reg->setTransformationEpsilon (1e-6);
//   reg->setMaximumIterations(iter);
//   reg->setRANSACIterations(riter);
//   reg->setMaxCorrespondenceDistance(mcd);
// 
//   cout << "findICP: reg set " << endl;
//   
//   reg->align(*nres);
//   
//   cout << "findICP: Has converged: " << reg->hasConverged() << "; score: " << reg->getFitnessScore() << endl;
//   
//   pcl::transformPointCloud (*cloud_src, *final, reg->getFinalTransformation());
//   
//   return 0;
// }

int
loadPCDFile( char * fname, Ptr cloud)
{
  if (pcl::io::loadPCDFile<PointT> (fname, *cloud) == -1)
  {
    PCL_ERROR ("Load PCDFile: Couldn't read file %s \n", fname);
    return (-1);
  }
  
  cout << "Load PCDFile: Loaded "
            << cloud->width * cloud->height
            << " data points from " << fname << "."
            << endl;

  return 0;
}

int main(int argc, char** argv)
{
	char *fname_tgt = argv[1];
	char *fname_src = argv[2];
	
	Ptr cloud_tgt (new PointCloud);
  	Ptr cloud_src (new PointCloud);
  	Ptr final (new PointCloud);

	cout << "Loading point cloud files.. " << endl;

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
	
	cout << "Running ICP"<<endl;
	
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	icp.setInputCloud(cloud_src);
	icp.setInputTarget(cloud_tgt);
	icp.setTransformationEpsilon(1e-6);
	icp.setMaximumIterations(100);
  	icp.setRANSACIterations(10000);
  	icp.setMaxCorrespondenceDistance(0.20);
	
	icp.align(*final);
	
	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
	icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;
	
	cout<<"Saving final result.."<<endl;
	
	pcl::io::savePCDFileASCII("final.pcd", *final);

}