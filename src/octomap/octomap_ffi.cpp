/* extern "C" makes the functions callable from C */
extern "C"
{
#include "TH.h"
}


#include <iostream>
#include <fstream>
#include <sstream>

#include <octomap/Pointcloud.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

using namespace std;
using namespace octomap;

extern "C"
{
  void calcThresholdedNodes(const OcTree* tree,
                            unsigned int& num_thresholded,
                            unsigned int& num_other)
  {
    num_thresholded = 0;
    num_other = 0;

    for(OcTree::tree_iterator it = tree->begin_tree(), end=tree->end_tree(); it!= end; ++it){
      if (tree->isNodeAtThreshold(*it))
        num_thresholded++;
      else
        num_other++;
    }
  }

  void OcTree_outputStatistics(const OcTree* tree){
    unsigned int numThresholded, numOther;
    calcThresholdedNodes(tree, numThresholded, numOther);
    size_t memUsage = tree->memoryUsage();
    unsigned long long memFullGrid = tree->memoryFullGrid();
    size_t numLeafNodes = tree->getNumLeafNodes();

    cout << "Tree size: " << tree->size() <<" nodes (" << numLeafNodes<< " leafs). " <<numThresholded <<" nodes thresholded, "<< numOther << " other\n";
    cout << "Memory: " << memUsage << " byte (" << memUsage/(1024.*1024.) << " MB)" << endl;
    cout << "Full grid: "<< memFullGrid << " byte (" << memFullGrid/(1024.*1024.) << " MB)" << endl;
    double x, y, z;
    tree->getMetricSize(x, y, z);
    cout << "Size: " << x << " x " << y << " x " << z << " m^3\n";
    cout << endl;
  }

  // move this to a generic function
  OcTree* OcTree_new (float res) {

    // tree->setClampingThresMin(clampingMin);
    // tree->setClampingThresMax(clampingMax);
    // tree->setProbHit(probHit);
    // tree->setProbMiss(probMiss);

    return new OcTree(res);

  }

  void OcTree_destroy(OcTree* tree)
  {
    delete(tree); 
  }

  void OcTree_add_sweep(OcTree* tree, THDoubleTensor* points, THDoubleTensor* origin, double max_range)
  {

    THArgCheck(THDoubleTensor_nDimension(points) == 2, 1, "points should be Nx3 invalid dimension");
    THArgCheck(THDoubleTensor_nElement(origin)   == 3, 2, "origin is of dimension 3,1x3 or 3x1");

    Pointcloud cloud;

    double * pts_d  = THDoubleTensor_data(points);
    double * orig_d = THDoubleTensor_data(origin);
    point3d sensor_origin ((float)orig_d[0],(float)orig_d[1],(float)orig_d[2]);


    long nelem = THDoubleTensor_nElement(points);
    long ndim  = points->size[1];

    bool lazy_eval = false;

    long i,pi = 0;
    // TODO replace with a pointer copy if float
    for (i=0; i< nelem; i+=ndim){
      cloud.push_back((float)pts_d[0], (float)pts_d[1], (float)pts_d[2]);
      pts_d += ndim;
    }

    tree->insertPointCloud(cloud,sensor_origin, max_range, lazy_eval);

  }
  
  THDoubleTensor* OcTree_toTensor(OcTree* tree, THDoubleTensor* points)
  {
    size_t nLeafNodes = tree->getNumLeafNodes();
    THDoubleTensor_resize2d(points,nLeafNodes,3);
    double * pts_d = THDoubleTensor_data(points);
    long count = 0;
    for(OcTree::leaf_iterator it = tree->begin(), end=tree->end(); it!= end; ++it) {
      if(tree->isNodeOccupied(*it)){
        pts_d[0] = it.getX();
        pts_d[1] = it.getY();
        pts_d[2] = it.getZ();
        count++;
        pts_d += 3;
      }
    }
    THDoubleTensor_narrow(points,NULL,0,0,count);

    return points;
  }
}
