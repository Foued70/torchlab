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
typedef struct PCXYZ PCXYZ;

void PointCloud_destroy(PCXYZ* cloud);

PCXYZ* PointCloud_pc_create();
PCXYZ* PointCloud_fromFile(const char * fname);
PCXYZ* PointCloud_fromTensor(THDoubleTensor* xyz);
int PointCloud_toFile(PCXYZ* test, const char * fname);
void PointCloud_toTensor(PCXYZ* test, THDoubleTensor* xyz);
bool PointCloud_doICP(PCXYZ* cloud1, PCXYZ* cloud2, THDoubleTensor* transf, double epsilon, double maxIterations, double ransacIterations, double maxCorrespondDis);

PCXYZ* PointCloud_uniformSample(PCXYZ* cloud, double radius);
double PointCloud_getEuclideanValidationScore(PCXYZ* source, PCXYZ* target, double max_range);
  

]]

return util.ffi.load("libpcl_ffi")

