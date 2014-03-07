ctorch = util.ctorch

ffi  = require 'ffi'
-- This is a list of wrapper functions encapsulating the octomap
-- functions we want.  The wrapper functions are defined in octomap_ffi.cpp
-- in an extern "C" block to make the C++ callable from lua C.) These
-- functions call the needed C++ operators.

ffi.cdef [[

void map2xyz( THDoubleTensor* xyz_map, THDoubleTensor* result );
void classifyPoints( THDoubleTensor* xyz_map, int window, double dist_thresh, THDoubleTensor* invalid_mask, THDoubleTensor* th_eigenvalues,
					 THDoubleTensor* normals, THDoubleTensor* th_means, THDoubleTensor* th_second_moments );
void cullPoints( THDoubleTensor* th_cull_mask, THDoubleTensor* th_normals, THDoubleTensor* th_points, int window, 
				 double cosine_thresh, double residual_thresh );
void grow_plane_region( THLongTensor* th_start_indices, THDoubleTensor* th_normals, THDoubleTensor* th_means, 
                        THDoubleTensor* th_second_moments, THDoubleTensor* th_region_mask, THDoubleTensor* th_front_mask, 
                        THDoubleTensor* th_output_eigenvalues, THDoubleTensor* th_output_plane, THDoubleTensor* th_output_mean,
                        double cosine_thresh, double residual_thresh );
]]

return util.ffi.load("libcplane_ffi")

