local libflann  = require '../kdtree/libflann'
local ffi    = require 'ffi'
local ctorch = util.ctorch

local kdtree = Class()

local default_flann_params = 
	libflann.new_flann_params_pointer({algorithm=libflann.clib.FLANN_INDEX_AUTOTUNED,
									   target_precision=0.99,
									   log_level=libflann.clib.FLANN_LOG_INFO})

function kdtree:__init(data, flann_params_table)
	
	self.flann_params = default_flann_params
	if flann_params_table then
		self.flann_params = libflann.new_flann_params_pointer(flann_params_table)
	end
	
	if data then
		self:load_data(data, flann_params_table)
	end
end

function kdtree:load_data(data, flann_params_table)
	
	local flann_params = self.flann_params
	if flann_params_table then
		flann_params = libflann.new_flann_params_pointer(flann_params_table)
	end
	
	if data then
		self.index, self.speedup = libflann.build_index(data, flann_params)
	end	
end

function kdtree:find_n_nearest_neighbors(reference_points, number_of_neighbors, flann_params_table)
	
	local flann_params = self.flann_params
	if flann_params_table then
		flann_params = libflann.new_flann_params_pointer(flann_params_table)
	end
	
	local indices, dists
	if self.index then
		indices, dists = libflann.find_nearest_neighbors_index(self.index, reference_points, number_of_neighbors, flann_params)
	end
	
	return indices, dists
end

function kdtree:radius_search(reference_point, radius, max_num_neighbors, flann_params_table)
	
	local flann_params = self.flann_params
	if flann_params_table then
		flann_params = libflann.new_flann_params_pointer(flann_params_table)
	end
	
	local mnn = 10
	if max_num_neighbors then
		mnn = max_num_neighbors
	end
	
	local indices, dists, retsize
	if self.index then
		indices, dists = libflann.radius_search(self.index, reference_point, mnn, radius, flann_params)
		local size = 0
		print (indices)
		while (size < mnn) and (indices[size+1] > -1) do
			size = size + 1
		end
		
		retsize = size
		
		if size == 0 then
			size = 1	
		end
	
		indices = indices:sub(1,size):clone()
		dists = dists:sub(1,size):clone()
	end
	
	return indices, dists, retsize
end