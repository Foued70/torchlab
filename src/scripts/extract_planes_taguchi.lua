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

fit_plane = geom.linear_model.fit
residual_fast = geom.linear_model.residual_fast
meshlist = torch.util.meshlist

-- Filenames and whatnot
scan_fname = '/Users/uriah/Downloads/precise-transit-6548/source/po_scan/a/001/sweep.xyz' 

-- Load in pointcloud
pc = PointCloud.PointCloud.new( scan_fname )

points = pc.points
n_points = points:size(1)
print("n_points: ", n_points)
-- Extract normals, etc ...
nmp,dd,phi,theta,pc_mask = pc:get_normal_map()

-- Index and mask for remaps
pc_index, pc_mask = pc:get_index_and_mask()

-- Extract points mapped onto 2d coordinates ... maybe used the masked data?
points_map = pc:get_xyz_map()
print("points_map size: ", points_map:size())

-- Params
residual_thresh = 20
n_reference_points = 20
reference_window = 21
sample_bounds = torch.ceil(reference_window/2)

-- Create colormap for n_planes
cmap = image.colormap(n_reference_points)

-- Create indices list to sample from, as we remove points we will remove the corresponding indices
x_inds, y_inds = meshlist(sample_bounds, points_map:size(2)-sample_bounds, sample_bounds, points_map:size(3)-sample_bounds)
point_indices = x_inds:cat(y_inds,1)

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

function extract_planes( points_map, point_indices, residual_thresh, n_reference_points, reference_window, sample_bounds )


	-- TODO sample from point indices 

	-- 1. Randomly sample n-reference points. 
	-- Here we sample from point_indices, that have been generated with knowledge of the reference window 
	-- Then we convert those points into x and y coordinates

	samples = torch.rand(n_reference_points):mul(point_indices:size(2)):long()+1
	rand_x = point_indices:select(1,1)[samples]
	rand_y = point_indices:select(1,2)[samples]

	bounded_x = points_map:size(2)-2*sample_bounds
	bounded_y = points_map:size(3)-2*sample_bounds

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

		-- 3. Find all inliers that for a connected component with the reference point
		-- 	  For a given inlier mask create a graph, run connected components on it, and see
		--    which group is a part of the reference point
		local inlier_map = util.addr.remap(inlier_mask:repeatTensor(3,1,1):double(), pc_index,pc_mask)
		local inlier_graph = imgraph.graph(inlier_map:eq(0):double(),8,'euclid')
		local inlier_cc = imgraph.connectcomponents(inlier_graph,0.999,false)
		-- Check if sampled point is inlier or not
		local is_inlier = inlier_map[{1,reference_x[1],reference_y[1]}]

		local reference_cc = inlier_cc[{reference_x[1],reference_y[1]}]

		print("num inliers: ", inlier_cc:eq(reference_cc):sum())
		print("is_inlier: ", is_inlier) 

		if DEBUG then 
			patches_rgb:sub(1,1,min_x,max_x,min_y,max_y):fill(cmap[{reference_idx,1}])
			patches_rgb:sub(2,2,min_x,max_x,min_y,max_y):fill(cmap[{reference_idx,2}])
			patches_rgb:sub(3,3,min_x,max_x,min_y,max_y):fill(cmap[{reference_idx,3}])

			points_r[inlier_mask] = cmap[{reference_idx,1}]
			points_g[inlier_mask] = cmap[{reference_idx,2}]
			points_b[inlier_mask] = cmap[{reference_idx,3}]

			if is_inlier==1 then 
				cc_regions_r[inlier_cc:eq(reference_cc)] = cmap[{reference_idx,1}]
				cc_regions_g[inlier_cc:eq(reference_cc)] = cmap[{reference_idx,2}]
				cc_regions_b[inlier_cc:eq(reference_cc)] = cmap[{reference_idx,3}]
			end		
		end

		-- Store results from each sample
		n_planes[reference_idx] = current_plane
		n_inliers[reference_idx] = inliers

		-- Doing this is kinda dangerous for memory
		n_inlier_mask[reference_idx] = inlier_mask

		-- Gotta force that garbage collection
		collectgarbage()
	end
	)

	--	4. The best plane is the one with the most inliers that are above some sufficient number 
	--	   based upon the window size, it they are not inliers=0
	_,sorted_i = torch.sort( n_inliers, true ) -- true for descending
	local inliers = n_inlier_mask[sorted_i[1]]
	local best_plane = n_planes[sorted_i[1]]	
	local inlier_mask = n_inlier_mask[sorted_i[1]]:squeeze()

	-- Return best plane, number of inliers and inlier mask
	return inliers, best_plane, inlier_mask
end

inliers, plane, inlier_mask = extract_planes( points_map, residual_thresh, n_reference_points, reference_window, sample_bounds)

if DEBUG then 
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
end

