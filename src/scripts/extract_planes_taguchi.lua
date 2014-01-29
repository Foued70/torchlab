--[[
	Extract Planes following the method from:
		Point-Plane SLAM for Hand-Held 3D Sensors, Taguchi, et al

	Method as described in II. System Overview
		1. Randomly select several reference points in the set points in the pointcloud
		2. For each reference point find the optimal plane using points in small local window 
		3. Find all inliers ( within some threshold ) that for a connected component with the 
		   reference point
		4. The best plane is the one with the most inliers that are above some sufficient number 
		   based upon the window size
		5. If optimal plane is found remove inliers and repeat
]]--

DEBUG = true

--[[
-- TODO: description and proper input handling!
function meshgrid( xdim, ydim ) 
    xgrid = torch.range(1,xdim):repeatTensor(ydim,1)
    ygrid = xgrid:clone():t()
    return xgrid, ygrid
end

-- TODO: description and proper input handling!
function meshlist( xdim, ydim )
    xgrid, ygrid = meshgrid( xdim, ydim )
    return xgrid:reshape(1,xdim*ydim), ygrid:reshape(1,xdim*ydim)
end
]]--

io = require 'io'
imgraph = require '../imgraph/init'  -- I feel like we shouldn't have to do this

--fit_plane = geom.linear_model.fit
fit_plane = Plane.util.fit_plane_to_points
residual_fast = geom.linear_model.residual_fast
cosine_distance = Plane.util.cosine_distance

meshlist = util.torch.meshlist
maskdim = util.torch.maskdim
select3d = util.torch.select3d

-- Filenames and whatnot
scan_fname = '/Users/uriah/Downloads/precise-transit-6548/source/po_scan/a/007/sweep.xyz' 

-- Load in pointcloud
pc = PointCloud.PointCloud.new( scan_fname )

points = pc.points
n_points = points:size(1)
-- Extract normals, etc ...
normal_map,dd,phi,theta,pc_mask = pc:get_normal_map()

-- Index and mask for remaps
pc_index, pc_mask = pc:get_index_and_mask()

-- Extract points mapped onto 2d coordinates ... maybe used the masked data?
points_map = pc:get_xyz_map()

-- Params
residual_thresh = 20
cosine_distance_thresh = 0.2
n_reference_points = 40
reference_window = 11
sample_bounds = torch.ceil(reference_window/2)
n_iter = 100

-- Create colormap for n_planes
--cmap = image.colormap(n_reference_points)
if n_iter > n_reference_points then 
	cmap = image.colormap(n_iter)
else
	cmap = image.colormap(n_reference_points)
end

-- Create indices list to sample from, as we remove points we will remove the corresponding indices
x_inds, y_inds = meshlist(sample_bounds, points_map:size(3)-sample_bounds, sample_bounds, points_map:size(2)-sample_bounds)

-- image.display( x_inds, points_map:size(3)-2*sample_bounds, points_map:size(2)-2*sample_bounds)
-- image.display( y_inds, points_map:size(3)-2*sample_bounds, points_map:size(2)-2*sample_bounds)
point_indices = x_inds:cat(y_inds,1)
point_indices_rng = torch.range(1,point_indices:size(2)):long()

-- DEBUG: for drawing patches, inliers and connected-components segmented inliers
if DEBUG then 
	patches_rgb = torch.Tensor(points_map:size())

	points_rgb = torch.Tensor(points:size()):transpose(2,1):contiguous()
	points_r = points_rgb[1]
	points_g = points_rgb[2]
	points_b = points_rgb[3]

	cc_regions_rgb = torch.Tensor(points_map:size())
	cc_regions_r = cc_regions_rgb[1]
	cc_regions_g = cc_regions_rgb[2]
	cc_regions_b = cc_regions_rgb[3]

	cc_regions_final_rgb = torch.Tensor(3, points_map:size(2)-reference_window, points_map:size(3)-reference_window)
	cc_regions_final_r = cc_regions_final_rgb[1]
	cc_regions_final_g = cc_regions_final_rgb[2]
	cc_regions_final_b = cc_regions_final_rgb[3]
end
--[[
	SAVING
	-- 1. Randomly sample n reference points by index, we want the refrence_window
	--    to be within the specified area so we offset the random values
	--   note: this would be a nice utility function
	--   TODO: we could also wrap-around since we do a full 360
	rand_x = torch.rand(n_reference_points,1):mul((points_map:size(2)-2*sample_bounds-1)):add(sample_bounds+1):int()
	rand_y = torch.rand(n_reference_points,1):mul((points_map:size(3)-2*sample_bounds-1)):add(sample_bounds+1):int()
]]--

function extract_planes( points_map, normal_map, point_inds, residual_thresh, n_reference_points, reference_window, sample_bounds, iter )
	-- TODO sample from point indices 

	-- 1. Randomly sample n-reference points. 
	-- Here we sample from point_indices, that have been generated with knowledge of the reference window 
	-- Then we convert those points into x and y coordinates

	local samples = torch.rand(n_reference_points):mul(point_inds:size(2)):long()+1
	local rand_x = point_inds:select(1,1)[samples]
	local rand_y = point_inds:select(1,2)[samples]

	--[[
	DEBUG:
	print("sample_bounds; ", sample_bounds)
	print("rand_x:lt(sample_bounds); ", rand_x:lt(sample_bounds))
	print("rand_y:lt(sample_bounds); ", rand_y:lt(sample_bounds))
	print("rand_x: ", rand_x)
	print("rand_y: ", rand_y)
	]]--


	-- 2. Fit optimal plane to local region ( I should probably wrap this in a nice RANSAC routine to get rid of outliers )
	local n_planes = {}
	local n_inliers = torch.IntTensor(n_reference_points)
	local n_inlier_mask = torch.ByteTensor(n_reference_points,n_points) -- consumes mad amounts of memory
	local n_inlier_map = torch.ByteTensor(n_reference_points,points_map:size(2), points_map:size(3)) -- so much memory

	torch.range(1,n_reference_points):apply( function( reference_idx )

		local reference_x = rand_x[reference_idx]
		local reference_y = rand_y[reference_idx]
		--[[ DEBUG 
		print("reference_x: ", reference_x)
		print("reference_y: ", reference_y)
		]]--

		local min_x = reference_x-torch.floor(reference_window/2)
		local max_x = reference_x+torch.floor(reference_window/2)
		local min_y = reference_y-torch.floor(reference_window/2)
		local max_y = reference_y+torch.floor(reference_window/2)

		-- Extract points in window
		local window_points = points_map:sub(1,3,min_y,max_y,min_x,max_x)
		-- print("window_points size: ", window_points:size())

		-- Reshape points into x,y,z rows 
		local pts = torch.reshape(window_points, 1,3,window_points:size(2)*window_points:size(3)):squeeze()
		-- Fit plane to points
		local current_plane = fit_plane( pts )
		-- Determine number of inliers
		local residuals = residual_fast(current_plane, points:t()):abs() 
	    local normal_dists = cosine_distance(current_plane,normal_map)
		local inlier_mask = residuals:lt(residual_thresh)
		-- local inliers = inlier_mask:sum()	

		-- 3. Find all inliers that for a connected component with the reference point
		-- 	  For a given inlier mask create a graph, run connected components on it, and see
		--    which group is a part of the reference point
		local inlier_map = util.addr.remap(inlier_mask:repeatTensor(3,1,1):double(), pc_index,pc_mask)

		-- Remove all normal outliers
		inlier_map:select(1,1)[normal_dists:gt(cosine_distance_thresh)] = 0
		inlier_map:select(1,2)[normal_dists:gt(cosine_distance_thresh)] = 0
		inlier_map:select(1,3)[normal_dists:gt(cosine_distance_thresh)] = 0

		local inlier_graph = imgraph.graph(inlier_map:eq(0):double(),8,'euclid')
		local inlier_cc = imgraph.connectcomponents(inlier_graph,0.999,false)
		-- Check if sampled point is inlier or not
		local is_inlier = inlier_map[{1,reference_y,reference_x}]
		local reference_cc = inlier_cc[{reference_y,reference_x}]

		local inliers = inlier_cc:eq(reference_cc):sum()

		--[[ DEBUG
		print("num inliers: ", inlier_cc:eq(reference_cc):sum())
		print("is_inlier: ", is_inlier) 
		]]--

		if DEBUG then 
			--[[
			patches_rgb:sub(1,1,min_x,max_x,min_y,max_y):fill(cmap[{reference_idx,1}])
			patches_rgb:sub(2,2,min_x,max_x,min_y,max_y):fill(cmap[{reference_idx,2}])
			patches_rgb:sub(3,3,min_x,max_x,min_y,max_y):fill(cmap[{reference_idx,3}])
			]]--
			patches_rgb:sub(1,1,min_y,max_y,min_x,max_x):fill(cmap[{iter,1}])
			patches_rgb:sub(2,2,min_y,max_y,min_x,max_x):fill(cmap[{iter,2}])
			patches_rgb:sub(3,3,min_y,max_y,min_x,max_x):fill(cmap[{iter,3}])

			points_r[inlier_mask] = cmap[{reference_idx,1}]
			points_g[inlier_mask] = cmap[{reference_idx,2}]
			points_b[inlier_mask] = cmap[{reference_idx,3}]

			if is_inlier==1 then 
				cc_regions_r[inlier_cc:eq(reference_cc)] = cmap[{reference_idx,1}]
				cc_regions_g[inlier_cc:eq(reference_cc)] = cmap[{reference_idx,2}]
				cc_regions_b[inlier_cc:eq(reference_cc)] = cmap[{reference_idx,3}]
			end		
		end

		if is_inlier==1 then
			n_inliers[reference_idx] = inliers
		else
			n_inliers[reference_idx] = 0
		end
		-- Store results from each sample
		n_planes[reference_idx] = current_plane
		-- Doing this is kinda dangerous for memory
		n_inlier_mask[reference_idx] = inlier_mask
		-- Store inliers maps

		n_inlier_map[reference_idx] = inlier_cc:eq(reference_cc)

		-- Gotta force that garbage collection
		collectgarbage()
	end
	)

	--	4. The best plane is the one with the most inliers that are above some sufficient number 
	--	   based upon the window size, it they are not inliers=0
	local sorted_inliers,sorted_i = torch.sort( n_inliers, true ) -- true for descending
	local inliers = sorted_inliers[1]
	local best_plane = n_planes[sorted_i[1]]	
	local inlier_mask = n_inlier_mask[sorted_i[1]]:squeeze()

	print("n_inliers: ", sorted_inliers)
	print("best inliers: ", inliers)

	-- inlier_map needs to map to our sampled indices
	local inlier_map = n_inlier_map[sorted_i[1]]
	local inlier_map = inlier_map:sub(sample_bounds, points_map:size(2)-sample_bounds, sample_bounds, points_map:size(3)-sample_bounds)
	local inlier_map = inlier_map:reshape(1,inlier_map:size(1)*inlier_map:size(2)):squeeze():byte()

	-- Return best plane, number of inliers and inlier mask
	return inliers, best_plane, inlier_mask, inlier_map
end

-- DEBUG: run this n_iter times, removing points each time
inlier_map = nil
for iter=1, n_iter do 
	-- Slightly frustrating logic 
	if inlier_map == nil then 
		indices = point_indices:index(2,point_indices_rng)
	else
		if inlier_map:eq(0):sum() == 0 then
			indices = point_indices:index(2,point_indices_rng)
		else		
			indices = point_indices:index(2,point_indices_rng[inlier_map:eq(0)])
		end
	end
	--image.display( point_indices[1]:reshape(points_map:size(2)-reference_window, points_map:size(3)-reference_window) )
	--image.display( point_indices[2]:reshape(points_map:size(2)-reference_window, points_map:size(3)-reference_window) )
	--image.display( point_indices[1]:reshape(869-11+1, 619-11+1) )

	print("indices:size(); ", indices:size())
	inliers, plane, inlier_mask, inlier_mp = extract_planes( points_map, normal_map, indices, residual_thresh, n_reference_points, reference_window, sample_bounds, iter)

	if inliers > reference_window*reference_window then
		ii = inlier_mp:reshape(points_map:size(2)-reference_window, points_map:size(3)-reference_window)
		cc_regions_final_r[ii] = cmap[{iter,1}]
		cc_regions_final_g[ii] = cmap[{iter,2}]
		cc_regions_final_b[ii] = cmap[{iter,3}]
	end

	if inlier_map == nil then
		inlier_map = inlier_mp
	else
		inlier_map = inlier_map:add(inlier_mp)
		inlier_map[inlier_map:gt(1)] = 1
		inlier_map[inlier_map:lt(1)] = 0
	end

	--[[
	print("points:size(); ", points:size())
	print("point_indices:size();  ", point_indices:size())
	print("inlier_map:size(); ", inlier_map:size())
	print("inlier_mask:size(); ", inlier_mask:size())
	print("points_map:size(); ", points_map:size())
	--points = maskdim(points, inlier_mask, 1)
	-- Since point_indices_rng
	print("point_indices_rng[inlier_map]:size(); ", point_indices_rng[inlier_map:eq(0)]:size())
	print("2*sample_bounds; ", 2*sample_bounds)
	]]--
	-- point_indices = maskdim(point_indices, inlier_map, 2)	

	-- point_indices = maskdim(point_indices, inlier_mask, 2)
end
-- DEBUG: point_indices arggg
-- image.display( point_indices[1]:reshape(619-11+1, 869-11+1) )
-- image.display( point_indices[2]:reshape(619-11+1, 869-11+1) )

if DEBUG then 
	image.display(normal_map)
	image.display(patches_rgb)

	-- Draw resulst
	points_rgb[1] = points_r
	points_rgb[2] = points_g
	points_rgb[3] = points_b
	pc_index, pc_mask = pc:get_index_and_mask()
	rgb_map = util.addr.remap(points_rgb, pc_index,pc_mask)
	image.display(rgb_map)

	cc_regions_rgb[1] = cc_regions_r
	cc_regions_rgb[2] = cc_regions_g
	cc_regions_rgb[3] = cc_regions_b
	image.display(cc_regions_rgb)

	cc_regions_final_rgb[1] = cc_regions_final_r
	cc_regions_final_rgb[2] = cc_regions_final_g
	cc_regions_final_rgb[3] = cc_regions_final_b
	image.display(cc_regions_final_rgb)
end
