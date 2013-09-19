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
#include <octomap/ColorOcTree.h>

using namespace std;
using namespace octomap;

// TODO use abstract class eg. (confusing in a generic ...) 
// AbstractOcTree* t = it->second.octree;
// if (dynamic_cast<OcTree*>(t)) {
//   ((OcTree*) t)->toMaxLikelihood();
// }
// else if (dynamic_cast<OcTree*>(t)) {
//   ((ColorOcTree*) t)->toMaxLikelihood();
// }

// TODO set other parameters of the tree
// tree->setClampingThresMin(clampingMin);
// tree->setClampingThresMax(clampingMax);
// tree->setProbHit(probHit);
// tree->setProbMiss(probMiss);

void getInfo(const AbstractOccupancyOcTree* tree)
{
  cout << "size: " << tree->size() << endl;
  cout << "getResolution: " << tree->getResolution() << endl;
  cout << "getOccupancyThres: " << tree->getOccupancyThres() << endl;
  cout << "getOccupancyThresLog: " << tree->getOccupancyThresLog() << endl;
  
  cout << "getProbHit: " << tree->getProbHit() << endl;
  cout << "getProbHitLog: " << tree->getProbHitLog() << endl;
  cout << "getProbMiss: " << tree->getProbMiss() << endl;
  cout << "getProbMissLog: " << tree->getProbMissLog() << endl;
  
  cout << "getClampingThresMin: " << tree->getClampingThresMin() << endl;
  cout << "getClampingThresMinLog: " << tree->getClampingThresMinLog() << endl;
  cout << "getClampingThresMax: " << tree->getClampingThresMax() << endl;
  cout << "getClampingThresMaxLog: " << tree->getClampingThresMaxLog() << endl;
}

extern "C"
{
  void calcOccupiedNodes(const OcTree* tree,
                         unsigned int& num_occupied,
                         unsigned int& num_other)
  {
    num_occupied = 0;
    num_other = 0;

    for(OcTree::tree_iterator it = tree->begin_tree(), end=tree->end_tree(); it!= end; ++it){
      if (tree->isNodeOccupied(*it))
        num_occupied++;
      else
        num_other++;
    }
  }

  void OcTree_outputStatistics(const OcTree* tree){
    unsigned int numOccupied, numOther;
    calcOccupiedNodes(tree, numOccupied, numOther);
    size_t memUsage = tree->memoryUsage();
    unsigned long long memFullGrid = tree->memoryFullGrid();
    size_t numLeafNodes = tree->getNumLeafNodes();

    cout << "Tree size: " << tree->size() <<" nodes (" << numLeafNodes<< " leafs). ";
    cout << numOccupied <<" nodes occupied, "<< numOther << " other\n";
    cout << "Memory: " << memUsage << " byte (" << memUsage/(1024.*1024.) << " MB)" << endl;
    cout << "Full grid: "<< memFullGrid << " byte (" << memFullGrid/(1024.*1024.) << " MB)" << endl;
    double x, y, z;
    tree->getMetricSize(x, y, z);
    cout << "Size: " << x << " x " << y << " x " << z << " m^3\n";
    cout << endl;
  }

  void Color_calcOccupiedNodes(const ColorOcTree* tree,
                                  unsigned int& num_occupied,
                                  unsigned int& num_other)
  {
    num_occupied = 0;
    num_other = 0;
    
    for(ColorOcTree::tree_iterator it = tree->begin_tree(), end=tree->end_tree(); it!= end; ++it){
      if (tree->isNodeOccupied(*it))
        num_occupied++;
      else
        num_other++;
    }
  }
  
  void ColorOcTree_outputStatistics(const ColorOcTree* tree){
    unsigned int numOccupied, numOther;
    Color_calcOccupiedNodes(tree, numOccupied, numOther);
    size_t memUsage = tree->memoryUsage();
    unsigned long long memFullGrid = tree->memoryFullGrid();
    size_t numLeafNodes = tree->getNumLeafNodes();
    
    cout << "Tree size: " << tree->size() <<" nodes (" << numLeafNodes<< " leafs). ";
    cout << numOccupied <<" nodes occupied, "<< numOther << " other\n";
    cout << "Memory: " << memUsage << " byte (" << memUsage/(1024.*1024.) << " MB)" << endl;
    cout << "Full grid: "<< memFullGrid << " byte (" << memFullGrid/(1024.*1024.) << " MB)" << endl;
    double x, y, z;
    tree->getMetricSize(x, y, z);
    cout << "Size: " << x << " x " << y << " x " << z << " m^3\n";
    cout << endl;
  }

  OcTree* OcTree_new (double res) {
    return new OcTree(res);
  }

  void OcTree_getInfo(OcTree* tree)
  {
    getInfo(tree);
  }

  void OcTree_destroy(OcTree* tree)
  {
    delete(tree); 
  }


  bool OcTree_write(OcTree* tree, const char* filename)
  {
    bool result = tree->write(filename);
    return result;
  }

  bool OcTree_read(OcTree* tree, const char* filename)
  {
    bool result = tree->read(filename);
    return result;
  }

  void OcTree_add_sweep(OcTree* tree, THDoubleTensor* points, THDoubleTensor* origin, double max_range)
  {

    THArgCheck(THDoubleTensor_nDimension(points) == 2, 1, "points should be Nx3 invalid dimension");
    THArgCheck(THDoubleTensor_nElement(origin)   == 3, 2, "origin is of dimension 3,1x3 or 3x1");

    Pointcloud cloud;

    tree->setProbHit(0.5);
    tree->setProbMiss(0.49);

    double * pts_d  = THDoubleTensor_data(points);
    double * orig_d = THDoubleTensor_data(origin);
    point3d sensor_origin ((float)orig_d[0],(float)orig_d[1],(float)orig_d[2]);

    long nelem = THDoubleTensor_nElement(points);
    long ndim  = points->size[1];

    bool lazy_eval = false;
    long i;
    // TODO replace with a pointer copy if float
    for (i=0; i< nelem; i+=ndim){
      cloud.push_back((float)pts_d[0], (float)pts_d[1], (float)pts_d[2]);
      pts_d += ndim;
    }
    cout << "origin:    " << sensor_origin << endl;
    cout << "max_range: " << max_range << endl;
    cout << "lazy_eval: " << lazy_eval << endl;
    tree->insertPointCloud(cloud, sensor_origin, max_range, lazy_eval);
  }
  
  // TODO make generic by calling AbstractOcTree funcs
  THDoubleTensor* OcTree_OccupiedCellstoTensor(OcTree* tree, THDoubleTensor* points)
  {
    unsigned int numOccupied, numOther;
    calcOccupiedNodes(tree, numOccupied, numOther);

    if (numOccupied > 0) {
      THDoubleTensor_resize2d(points,numOccupied,3);
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
    }
    return points;
  }

  THDoubleTensor* OcTree_getThresholds(OcTree* tree, THDoubleTensor* thresholds)
  {
    unsigned int numLeafNodes = tree->getNumLeafNodes();
    if (numLeafNodes > 0) {
      THDoubleTensor_resize1d(thresholds,numLeafNodes);
      double * thr_d = THDoubleTensor_data(thresholds);
      long count = 0;
      for(OcTree::leaf_iterator it = tree->begin(), end=tree->end(); it!= end; ++it) {
        OcTreeNode* node = tree->search(it.getKey());
        thr_d[0] = node->getOccupancy();
        thr_d++;
      }
    }
    return thresholds;
  }


  THDoubleTensor* OcTree_EmptyCellstoTensor(OcTree* tree, THDoubleTensor* points)
  {
    unsigned int numOccupied, numOther;
    calcOccupiedNodes(tree, numOccupied, numOther);

    THDoubleTensor_resize2d(points,numOther,3);
    double * pts_d = THDoubleTensor_data(points);
    long count = 0;
    for(OcTree::leaf_iterator it = tree->begin(), end=tree->end(); it!= end; ++it) {
      if(!tree->isNodeOccupied(*it)){
        pts_d[0] = it.getX();
        pts_d[1] = it.getY();
        pts_d[2] = it.getZ();
        count++;
        pts_d += 3;
      }
    }
    return points;
  }

  // TODO get list of unknown cells
  // unknown cells don't exist in the tree
  /// return centers of leafs that do NOT exist (but could) in a given bounding box
  // THDoubleTensor* OcTree_UnknownCellstoTensor(OcTree* tree, THDoubleTensor* points)
  // void getUnknownLeafCenters(point3d_list& node_centers, point3d pmin, point3d pmax) const;

  ColorOcTree*  ColorOcTree_new(double res)
  {
    return new ColorOcTree(res);
  }

  void ColorOcTree_getInfo(ColorOcTree* tree)
  {
    getInfo(tree);
  }

  bool ColorOcTree_write(ColorOcTree* tree, const char* filename)
  {
    bool result = tree->write(filename);
    return result;
  }

  bool ColorOcTree_read(ColorOcTree* tree, const char* filename)
  {
    bool result = tree->read(filename);
    return result;
  }

  void ColorOcTree_destroy(ColorOcTree* tree)
  {
    delete(tree); 
  }

  void ColorOcTree_add_sweep(ColorOcTree* tree, 
                             THDoubleTensor* points, THDoubleTensor* origin, double max_range, 
                             THFloatTensor* rgb )
  {

    THArgCheck(THDoubleTensor_nDimension(points) == 2, 1, "points should be Nx3 invalid dimension");
    THArgCheck(THDoubleTensor_nElement(origin)   == 3, 2, "origin is of dimension 3,1x3 or 3x1");
    THArgCheck(THFloatTensor_nElement(rgb) == THDoubleTensor_nElement(points), 5, "number of rgb values not equal to numer of points");
    
    Pointcloud cloud;

    double * pts_d  = THDoubleTensor_data(points);
    double * orig_d = THDoubleTensor_data(origin);

    point3d sensor_origin ((float)orig_d[0],(float)orig_d[1],(float)orig_d[2]);
    
    long npts  = points->size[0];
    long nelem = THDoubleTensor_nElement(points);
    long ndim  = points->size[1];

    bool lazy_eval = false;

    long i,pi = 0;

    // TODO replace with a pointer copy if float
    for (i=0; i< npts  ; i++){
      cloud.push_back((float)pts_d[0], (float)pts_d[1], (float)pts_d[2]);
      pts_d += ndim;
    }
    
    tree->insertPointCloud(cloud,sensor_origin, max_range, lazy_eval);
    cout << "inserted points" << endl;
    ColorOcTree_outputStatistics(tree);
  
    // first try with a second loop
    long count = 0;
    pts_d  = THDoubleTensor_data(points); // reset to start of tensor
    float * rgb_d  = THFloatTensor_data(rgb);
    OcTreeKey key ;
    for (i=0; i< npts ; i++){
      if (tree->coordToKeyChecked(point3d(pts_d[0], pts_d[1], pts_d[2]),key)) {
        tree->averageNodeColor(key , (unsigned char) rgb_d[0], (unsigned char) rgb_d[1], (unsigned char) rgb_d[2]);
        count++;
      }
      pts_d += 3;
      rgb_d += 3;
    }
    cout << "colored: " << count << " of " << npts << endl;
  }

  THDoubleTensor* ColorOcTree_OccupiedCellstoTensor(ColorOcTree* tree, THDoubleTensor* points, THFloatTensor* rgb)
  {
    unsigned int numOccupied, numOther;
    Color_calcOccupiedNodes(tree, numOccupied, numOther);

    if (numOccupied > 0) {
      long count = 0;
      THDoubleTensor_resize2d(points,numOccupied,3);
      THFloatTensor_resize2d(rgb, numOccupied,3);
      double * pts_d = THDoubleTensor_data(points);
      float * rgb_d = THFloatTensor_data(rgb);
      ColorOcTreeNode::Color c;
      for(ColorOcTree::leaf_iterator it = tree->begin(), end=tree->end(); it!= end; ++it) {
        if(tree->isNodeOccupied(*it)){
          pts_d[0] = it.getX();
          pts_d[1] = it.getY();
          pts_d[2] = it.getZ();
          
          ColorOcTreeNode* node = tree->search(it.getKey());
          if (node) {
            c = node->getColor();
            rgb_d[0] = c.r;
            rgb_d[1] = c.g;
            rgb_d[2] = c.b;
          } else {
            rgb_d[0] = 255;
            rgb_d[1] = 255;
            rgb_d[2] = 255;
          }
          count++;
          pts_d += 3;
          rgb_d += 3;
        }
      }
    THDoubleTensor_narrow(points,NULL,0,0,count);
    THFloatTensor_narrow(rgb,NULL,0,0,count);
    }
    return points;
  }


}
