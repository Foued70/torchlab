ctorch = util.ctorch

ffi  = require 'ffi'
-- This is a list of wrapper functions encapsulating the octomap
-- functions we want.  The wrapper functions are defined in octomap_ffi.cpp
-- in an extern "C" block to make the C++ callable from lua C.) These
-- functions call the needed C++ operators.

ffi.cdef [[
void get_graph_shortest_path(THDoubleTensor* points, int initializer, THDoubleTensor* result);
void get_mst(THDoubleTensor* points, THDoubleTensor* result);
]]

return util.ffi.load("libboostgraph_ffi")