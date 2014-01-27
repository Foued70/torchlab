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

io = require 'io'
imgraph = require '../imgraph/init'  -- I feel like we shouldn't have to do this

fit_plane = geom.linear_model.fit
residual_fast = geom.linear_model.residual_fast

-- Filenames and whatnot
scan_fname = '/Users/uriah/Downloads/precise-transit-6548/source/po_scan/a/001/sweep.xyz' 

-- Load in pointcloud
pc = PointCloud.PointCloud.new( scan_fname )

points = pc.points
n_points = points:size(1)
print("n_points: ", n_points)
-- Extract normals, etc ...
nmp,dd,phi,theta,pc_mask = pc:get_normal_map()

-- Extract points mapped onto 2d coordinates ... maybe used the masked data?
points_map = pc:get_xyz_map()
print("points_map size: ", points_map:size())

-- Params
residual_thresh = 10
n_reference_points = 50
reference_window = 51
sample_bounds = torch.ceil(reference_window/2)

-- 1. Randomly sample n reference points by index, we want the refrence_window
--    to be within the specified area so we offset the random values
--   note: this would be a nice utility function
--   TODO: we could also wrap-around since we do a full 360
rand_x = torch.rand(n_reference_points,1):mul((points_map:size(2)-2*sample_bounds-1)):add(sample_bounds+1):int()
rand_y = torch.rand(n_reference_points,1):mul((points_map:size(3)-2*sample_bounds-1)):add(sample_bounds+1):int()

-- 2. Fit optimal plane to local region ( I should probably wrap this in a nice RANSAC routine to get rid of outliers )
n_planes = {}
n_inliers = torch.IntTensor(n_reference_points)
n_inlier_mask = torch.ByteTensor(n_reference_points,n_points) -- consumes mad amounts of memory
torch.range(1,n_reference_points):apply( function( reference_idx )

	local reference_x = rand_x[reference_idx]
	local reference_y = rand_y[reference_idx]

	local min_x = (reference_x-torch.floor(reference_window/2))[1]
	local max_x = (reference_x+torch.floor(reference_window/2))[1]
	local min_y = (reference_y-torch.floor(reference_window/2))[1]
	local max_y = (reference_y+torch.floor(reference_window/2))[1]
	--[[
	print("reference_x: ", reference_x)
	print("reference_y: ", reference_y)
	print("min_x: ", min_x)
	print("max_x: ", max_x)
	print("min_y: ", min_y)
	print("max_y: ", max_y)
	]]--

	-- Extract points in window
	local window_points = points_map:sub(1,3,min_x,max_x,min_y,max_y)
	-- print("window_points size: ", window_points:size())

	-- Reshape points into x,y,z rows 
	local pts = torch.reshape(window_points, 1,3,window_points:size(2)*window_points:size(3)):squeeze()
	-- Fit plane to points
	local current_plane = fit_plane( pts )
	-- Determine number of inliers
	local residuals = residual_fast(current_plane, points:t()):abs() 
    -- local normal_dists = cosine_distance(plane,normals) TODO: add normals into this step 
	local inlier_mask = residuals:lt(residual_thresh)
	local inliers = inlier_mask:sum()	

	-- Store results from each sample
	n_planes[reference_idx] = current_plane
	n_inliers[reference_idx] = inliers

	-- Doing this is kinda dangerous for memory
	n_inlier_mask[reference_idx] = inlier_mask
end
)


-- Create colormap for n_planes
cmap = image.colormap(n_reference_points)

-- DEBUG: Draw patches
patches_rgb = torch.Tensor(points_map:size())
torch.range(1,n_reference_points):apply( function( reference_idx )
	local reference_x = rand_x[reference_idx]
	local reference_y = rand_y[reference_idx]

	local min_x = (reference_x-torch.floor(reference_window/2))[1]
	local max_x = (reference_x+torch.floor(reference_window/2))[1]
	local min_y = (reference_y-torch.floor(reference_window/2))[1]
	local max_y = (reference_y+torch.floor(reference_window/2))[1]
	patches_rgb:sub(1,1,min_x,max_x,min_y,max_y):fill(cmap[{reference_idx,1}])
	patches_rgb:sub(2,2,min_x,max_x,min_y,max_y):fill(cmap[{reference_idx,2}])
	patches_rgb:sub(3,3,min_x,max_x,min_y,max_y):fill(cmap[{reference_idx,3}])
end
)
image.display(patches_rgb)

-- DEBUG: Color and draw inlier patches on top of depth map
points_rgb = torch.Tensor(points:size()):transpose(2,1):contiguous()
points_r = points_rgb[1]
points_g = points_rgb[2]
points_b = points_rgb[3]
torch.range(1,n_reference_points):apply( function( reference_idx )
	points_r[n_inlier_mask[reference_idx]] = cmap[{reference_idx,1}]
	points_g[n_inlier_mask[reference_idx]] = cmap[{reference_idx,2}]
	points_b[n_inlier_mask[reference_idx]] = cmap[{reference_idx,3}]
end
)

points_rgb[1] = points_r
points_rgb[2] = points_g
points_rgb[3] = points_b
pc_index, pc_mask = pc:get_index_and_mask()
rgb_map = util.addr.remap(points_rgb, pc_index,pc_mask)
image.display( rgb_map )


-- 3. Find all inliers that for a connected component with the reference point
-- 	  For a given inlier mask create a graph, run connected components on it, and see
--    which group is a part of the reference point

-- g = imgraph.graph(mask:eq(0),8,'euclid')
-- image.display(imgraph.connectcomponents(g,0.999,true):mul(255))
cc_regions_rgb = torch.Tensor(points_map:size())
cc_regions_r = cc_regions_rgb[1]
cc_regions_g = cc_regions_rgb[2]
cc_regions_b = cc_regions_rgb[3]

torch.range(1,n_reference_points):apply( function( reference_idx )
	inlier_map = util.addr.remap(n_inlier_mask[reference_idx]:repeatTensor(3,1,1):double(), pc_index,pc_mask)
	-- image.display( inlier_map:mul(255) )
	inlier_graph = imgraph.graph(inlier_map:eq(0):double(),8,'euclid')
	inlier_cc = imgraph.connectcomponents(inlier_graph,0.999,false)
	-- image.display(imgraph.connectcomponents(inlier_graph,0.999,true):mul(255))

	is_inlier = inlier_map[{1,rand_x[reference_idx][1],rand_y[reference_idx][1]}]

	reference_cc = inlier_cc[{rand_x[reference_idx][1],rand_y[reference_idx][1]}]
	--[[
	print("rand_x: ", rand_x[reference_idx][1])
	print("rand_y: ", rand_y[reference_idx][1])
	print("inlier_cc size: ", inlier_cc:size())
	print("inlier_cc: ", reference_cc)
	]]--
	print("num inliers: ", inlier_cc:eq(reference_cc):sum())
	print("is_inlier: ", is_inlier) 

	if is_inlier==1 then 
		cc_regions_r[inlier_cc:eq(reference_cc)] = cmap[{reference_idx,1}]
		cc_regions_g[inlier_cc:eq(reference_cc)] = cmap[{reference_idx,2}]
		cc_regions_b[inlier_cc:eq(reference_cc)] = cmap[{reference_idx,3}]
	end

	-- image.display(inlier_cc:eq(reference_cc):mul(255))
end
)

cc_regions_rgb[1] = cc_regions_r
cc_regions_rgb[2] = cc_regions_g
cc_regions_rgb[3] = cc_regions_b
image.display(cc_regions_rgb)


--	4. The best plane is the one with the most inliers that are above some sufficient number 
--	   based upon the window size






-- DEBUG: Color and draw connected component inliers 











