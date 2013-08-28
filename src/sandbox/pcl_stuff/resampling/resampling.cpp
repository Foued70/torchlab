#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

int
main (int argc, char** argv)
{
  // Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  // Load bun0.pcd -- should be available with the PCL archive in test 
  pcl::io::loadPCDFile ("bun0.pcd", *cloud);

  std::cout << " + loaded pcd file" << std::endl;

  // Create a KD-Tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

  std::cout << " + build KD-tree" << std::endl;

  // Output has the PointNormal type in order to store the normals calculated by MLS
  pcl::PointCloud<pcl::PointNormal> mls_points;

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
 
  mls.setComputeNormals (true);

  // Set parameters
  mls.setInputCloud (cloud);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.03);

  std::cout << " + setup MLS" << std::endl;

  // Reconstruct
  mls.process (mls_points);

  std::cout << " + process MLS" << std::endl;

  // Save output
  pcl::io::savePCDFile ("bun0-mls.pcd", mls_points);

  std::cout << " + saved pcd file" << std::endl;

  pcl::io::savePLYFile("bun0-mls.ply",mls_points);

  std::cout << " + saved ply file" << std::endl;

}
