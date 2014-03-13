ctorch = util.ctorch

ffi  = require 'ffi'
-- This is a list of wrapper functions encapsulating the octomap
-- functions we want.  The wrapper functions are defined in octomap_ffi.cpp
-- in an extern "C" block to make the C++ callable from lua C.) These
-- functions call the needed C++ operators.

ffi.cdef [[
	void calculate_boundary(THDoubleTensor* points, THDoubleTensor* center, THByteTensor* rgb, THIntTensor* result, THByteTensor* result_rgb);
]]

return util.ffi.load("libborder_boundary")