ctorch = util.ctorch

ffi  = require 'ffi'
-- This is a list of wrapper functions encapsulating the octomap
-- functions we want.  The wrapper functions are defined in octomap.cpp
-- in an extern "C" block to make the C++ callable from lua C.) These
-- functions call the needed C++ operators. They accept and return the
-- opencv C structs defined above.

-- Mat.lua
ffi.cdef [[
// ------------
//   opaque pointer (not visible from Lua interface)
// ------------
typedef struct OcTree OcTree;
typedef struct ColorOcTree ColorOcTree;

// ------------
//   functions on the OcTree
// ------------
OcTree* OcTree_new (float resolution);
void    OcTree_destroy(OcTree* tree);

void    OcTree_add_sweep(OcTree* tree, THDoubleTensor* points, THDoubleTensor* origin, double max_range);
THDoubleTensor* OcTree_OccupiedCellstoTensor(OcTree* tree, THDoubleTensor* points);
THDoubleTensor* OcTree_EmptyCellstoTensor(OcTree* tree, THDoubleTensor* points);
void    OcTree_outputStatistics(const OcTree* tree);

// ------------
//   functions on the ColorOcTree
// ------------
ColorOcTree* ColorOcTree_new (float resolution);
void         ColorOcTree_destroy(ColorOcTree* tree);
void         ColorOcTree_add_sweep(ColorOcTree* tree, 
                                   THDoubleTensor* points, THDoubleTensor* origin, double max_range,
                                   THByteTensor* color);


]]

return util.ffi.load("liboctomap_ffi")

