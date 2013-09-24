ctorch = util.ctorch

ffi  = require 'ffi'
-- This is a list of wrapper functions encapsulating the octomap
-- functions we want.  The wrapper functions are defined in octomap_ffi.cpp
-- in an extern "C" block to make the C++ callable from lua C.) These
-- functions call the needed C++ operators.

ffi.cdef [[
// ------------
//   opaque pointer (not visible from Lua interface)
// ------------
typedef struct OcTree OcTree;
typedef struct ColorOcTree ColorOcTree;

// ------------
//   functions on the OcTree
// ------------
OcTree*         OcTree_new (double resolution);
void            OcTree_destroy(OcTree* tree);
bool            OcTree_write(OcTree* tree, const char* filename);
bool            OcTree_read(OcTree* tree, const char* filename);

void            OcTree_add_sweep(OcTree* tree, 
                                 THDoubleTensor* points, THDoubleTensor* origin, double max_range);

THDoubleTensor* OcTree_OccupiedCellstoTensor(OcTree* tree, THDoubleTensor* points);
THDoubleTensor* OcTree_EmptyCellstoTensor(OcTree* tree, THDoubleTensor* points);
THDoubleTensor* OcTree_getThresholds(OcTree* tree, THDoubleTensor* occupancy);

void            OcTree_outputStatistics(const OcTree* tree);
void            OcTree_getInfo(const OcTree* tree);

// ------------
//   functions on the ColorOcTree
// ------------
ColorOcTree* ColorOcTree_new (double resolution);
void         ColorOcTree_destroy(ColorOcTree* tree);
bool         ColorOcTree_write(ColorOcTree* tree, const char* filename);
bool         ColorOcTree_read(ColorOcTree* tree, const char* filename);

void         ColorOcTree_add_sweep(ColorOcTree* tree, 
                                   THDoubleTensor* points, THDoubleTensor* origin, double max_range,
                                   THByteTensor* color, 
                                   THDoubleTensor* cost);

// TODO make single version of these
THDoubleTensor* ColorOcTree_EmptyCellstoTensor(ColorOcTree* tree, THDoubleTensor* points);
THDoubleTensor* ColorOcTree_getThresholds(ColorOcTree* tree, THDoubleTensor* occupancy);

THDoubleTensor* ColorOcTree_OccupiedCellstoTensor(ColorOcTree* tree, THDoubleTensor* points, THByteTensor* rgb);

void         ColorOcTree_castRays(ColorOcTree* tree, 
                                  THDoubleTensor* origin, THDoubleTensor* directions, double max_range, 
                                  THByteTensor* rgb);

void         ColorOcTree_get_color_for_xyz(ColorOcTree* tree, 
                                           THDoubleTensor* points,
                                           THByteTensor* rgb);

void         ColorOcTree_outputStatistics(const ColorOcTree* tree);
void         ColorOcTree_getInfo(const ColorOcTree* tree);


]]

return util.ffi.load("liboctomap_ffi")

