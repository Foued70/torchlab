ctorch = util.ctorch

ffi  = require 'ffi'
-- This is a list of wrapper functions encapsulating the octomap
-- functions we want.  The wrapper functions are defined in octomap_ffi.cpp
-- in an extern "C" block to make the C++ callable from lua C.) These
-- functions call the needed C++ operators.

ffi.cdef [[
// ------------
//   functions on the OcTree
//   uses an opaque pointer (not visible from Lua interface)
// ------------
typedef struct PointXYZRGB
{
    float x;
    float y;
    float z;
    double r;
    double g;
    double b;
} PointXYZRGB;

typedef struct PCXYZRGB PCXYZRGB;
typedef struct PCN PCN;
typedef struct Descriptors Descriptors;
typedef struct PolygonMesh PolygonMesh;

void PointCloud_destroy(PCXYZRGB* cloud);
void PointCloudN_destroy(PCN* cloud);
void PointCloudDescriptors_destroy(Descriptors* cloud);
void PointCloudMesh_destroy(PolygonMesh* mesh);

PCN* PointCloud_normal_create();
PCXYZRGB* PointCloud_pc_create();
PCXYZRGB* PointCloud_fromFile(const char * fname);
PCXYZRGB* PointCloud_fromTensor(THDoubleTensor* xyz, THDoubleTensor* rgb);
PCN* PointCloud_getNormals_fromTensor(THDoubleTensor* norm);
int PointCloud_toFile(PCXYZRGB* test, const char * fname);
void PointCloud_toTensor(PCXYZRGB* test, THDoubleTensor* xyz, THDoubleTensor* rgb);
bool PointCloud_doICP(PCXYZRGB* cloud1, PCXYZRGB* cloud2, THDoubleTensor* transf, double epsilon, double maxIterations, double ransacIterations, double maxCorrespondDis);


void PointCloud_normalToTensor(PCN* cloud, THDoubleTensor* nxyz);
PCN* PointCloud_getNormals(PCXYZRGB* cloud, double ksearch);
PCXYZRGB* PointCloud_uniformSample(PCXYZRGB* cloud, double radius);
double PointCloud_getEuclideanValidationScore(PCXYZRGB* source, PCXYZRGB* target, double max_range);
PCXYZRGB* growRegions(PCXYZRGB* cloud, PCN* normals, THLongTensor* point_indices, THLongTensor* location_change, double minClusterSize, 
	int numNeighbors, double smoothnessThreshold, double curvatureThreshold);
PolygonMesh* greedy_proj_mesh(PCXYZRGB* cloud, PCN* normals, double mu,
  double radiusSearch, double maximumNearestNeighbors, double maxSurfaceAngle, double minAngle,
  double maxAngle);

Descriptors* PointCloud_computeDescriptors(PCXYZRGB* downsampled_cloud, PCXYZRGB* orig_cloud, PCN* normals, double descr_radius);

void PointCloud_computeTransformation(Descriptors* desc1,Descriptors* desc2, PCXYZRGB* cloud1_down, PCXYZRGB* cloud2_down, 
	 THDoubleTensor* transformations, THDoubleTensor* correspondences, double cg_size, double cg_thresh);
  

]]

return util.ffi.load("libpcl_ffi")

