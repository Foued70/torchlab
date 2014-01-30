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

-- Setup --- 

-- Params
params = {}
params.residual_threshold = 20
params.normal_threshold = 0.4
params.n_reference_points = 15
params.reference_window = 12
params.min_points_for_seed = math.pow(params.reference_window,2)
params.min_points_for_plane = params.min_points_for_seed
params.sample_bounds = torch.ceil(params.reference_window/2)
params.n_iter = 30

print("sample_bounds: ", params.sample_bounds)

-- Filenames and whatnot
scan_fname = '/Users/uriah/Downloads/precise-transit-6548/source/po_scan/a/007/sweep.xyz' 
output_dir = '/Users/uriah/Downloads/precise-transit-6548/work/output'

-- Configure plane validator and iterative fitting 
validate = Plane.Validate.new{
   residual_threshold   = params.residual_threshold,
   normal_threshold     = params.normal_threshold,
   min_points_for_seed  = params.min_points_for_seed,
   min_points_for_plane = params.min_points_for_plane
}

itrw = Plane.FitIterativeReweighted.new{
   residual_threshold = params.residual_threshold,
   residual_decr      = 0.7,
   residual_stop      = 1,
   normal_threshold   = params.normal_threshold,
   normal_decr        = 0.7,
   normal_stop        = math.pi/360,
   min_points_for_plane = params.min_points_for_plane
}

--itrw.save_images = true
itrw.verbose     = true

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

-- Create colormap for n_reference points or n_planes 
if params.n_iter > params.n_reference_points then 
	cmap = image.colormap(params.n_iter)
else
	cmap = image.colormap(params.n_reference_points)
end

-- Create indices list to sample from, as we remove points we will remove the corresponding indices
x_inds, y_inds = meshlist(params.sample_bounds, points_map:size(3)-params.sample_bounds, params.sample_bounds, points_map:size(2)-params.sample_bounds)

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

	cc_regions_final_rgb = torch.Tensor(3, points_map:size(2)-2*params.sample_bounds+1, points_map:size(3)-2*params.sample_bounds+1)
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

function extract_planes( points, points_map, normals, point_inds, iter, params )
	-- Extract useful params
	local residual_threshold = params.residual_threshold
	local normal_threshold = params.normal_threshold
	local n_reference_points = params.n_reference_points
	local reference_window = params.reference_window
	local sample_bounds = params.sample_bounds

	-- 1. Randomly sample n-reference points. 
	-- Here we sample from point_indices, that have been generated with knowledge of the reference window 
	-- Then we convert those points into x and y coordinates

	local samples = torch.rand(n_reference_points):mul(point_inds:size(2)):long()+1
	local rand_x = point_inds:select(1,1)[samples]
	local rand_y = point_inds:select(1,2)[samples]

	-- 2. Fit optimal plane to local region ( I should probably wrap this in a nice RANSAC routine to get rid of outliers )
	local n_planes = {}
	local n_inliers = torch.IntTensor(n_reference_points):zero()
	local n_inlier_mask = torch.ByteTensor(n_reference_points,n_points):zero() -- consumes mad amounts of memory
	local n_inlier_map = torch.ByteTensor(n_reference_points,points_map:size(2), points_map:size(3)):zero() -- so much memory

	local imgh = normals:size(2)
	local imgw = normals:size(3)

	local idx = torch.IntTensor(2)
    local patch_mask = torch.ByteTensor(points_map:size(2), points_map:size(3)):zero()

	torch.range(1,n_reference_points):apply( function( reference_idx )

		local reference_x = rand_x[reference_idx]
		local reference_y = rand_y[reference_idx]
		-- Set idx 
		idx[2] = reference_x
		idx[1] = reference_y

        local bbx = Plane.finder_utils.compute_bbx(idx,reference_window,reference_window,imgh,imgw)
		patch_mask:fill(0)
        patch_mask[{{bbx[1],bbx[2]},{bbx[3],bbx[4]}}] = 1

        local min_y = bbx[1]
        local max_y = bbx[2]
        local min_x = bbx[3]
        local max_x = bbx[4]

       	--[[
		local min_x = reference_x-torch.floor(reference_window/2)
		local max_x = reference_x+torch.floor(reference_window/2)
		local min_y = reference_y-torch.floor(reference_window/2)
		local max_y = reference_y+torch.floor(reference_window/2)

		-- Extract points in window
		local window_points = points_map:sub(1,3,min_y,max_y,min_x,max_x)

		-- Reshape points into x,y,z rows 
		local pts = torch.reshape(window_points, 1,3,window_points:size(2)*window_points:size(3)):squeeze()
		-- Fit plane to points
		--local current_plane = fit_plane( pts )
		]]--

		local current_plane, error_string =
	            validate:seed(points_map, normals, patch_mask, debug_info)	

		-- Determine number of inliers
		--[[
		local residuals = residual_fast(current_plane, points:t()):abs() 
	    local normal_dists = cosine_distance(current_plane,normal_map)
		local inlier_mask = residuals:lt(residual_threshold)
		]]--

		-- If current plane is nil then we are not an inlier by the cc test
		local is_inlier = false
		local inlier_cc = nil
		local reference_cc = nil
		local inliers = 0
		if current_plane then

			-- check filtering (not removing enough points) ... hopefully using connected components helps 
	        local inlier_pts, inlier_n_pts, inlier_mask =
		           current_plane:filter_points(points_map, normals)

		    local inlier_map = inlier_mask:reshape(imgh, imgw):repeatTensor(3,1,1)


			-- 3. Find all inliers that for a connected component with the reference point
			-- 	  For a given inlier mask create a graph, run connected components on it, and see
			--    which group is a part of the reference point
			-- local inlier_map = util.addr.remap(inlier_mask:repeatTensor(3,1,1):double(), pc_index,pc_mask)

			--[[
			-- Remove all normal outliers
			inlier_map:select(1,1)[normal_dists:gt(normal_threshold)] = 0
			inlier_map:select(1,2)[normal_dists:gt(normal_threshold)] = 0
			inlier_map:select(1,3)[normal_dists:gt(normal_threshold)] = 0
			]]--

			local inlier_graph = imgraph.graph(inlier_map:eq(0):double(),8,'euclid')
			inlier_cc = imgraph.connectcomponents(inlier_graph,0.999,false)
			-- Check if sampled point is inlier or not
			is_inlier = inlier_map[{1,reference_y,reference_x}]
			reference_cc = inlier_cc[{reference_y,reference_x}]

			inliers = inlier_cc:eq(reference_cc):sum()
			if inliers < params.min_points_for_seed then 
				inliers = 0
			end
		end

		if DEBUG then 
			patches_rgb:sub(1,1,min_y,max_y,min_x,max_x):fill(cmap[{iter,1}])
			patches_rgb:sub(2,2,min_y,max_y,min_x,max_x):fill(cmap[{iter,2}])
			patches_rgb:sub(3,3,min_y,max_y,min_x,max_x):fill(cmap[{iter,3}])

			--[[
			points_r[inlier_mask] = cmap[{reference_idx,1}]
			points_g[inlier_mask] = cmap[{reference_idx,2}]
			points_b[inlier_mask] = cmap[{reference_idx,3}]
			]]--

			if is_inlier==1 then 
				cc_regions_r[inlier_cc:eq(reference_cc)] = cmap[{reference_idx,1}]
				cc_regions_g[inlier_cc:eq(reference_cc)] = cmap[{reference_idx,2}]
				cc_regions_b[inlier_cc:eq(reference_cc)] = cmap[{reference_idx,3}]
			end		
		end

		if is_inlier==1 then
			n_inlier_map[reference_idx] = inlier_cc:eq(reference_cc)
		end
		-- Store results from each sample
		n_inliers[reference_idx] = inliers
		n_planes[reference_idx] = current_plane
		-- Doing this is kinda dangerous for memory
		-- n_inlier_mask[reference_idx] = inlier_mask
		-- Store inliers maps


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

	print("sorted_inliers: ", sorted_inliers)
	print("best inliers: ", inliers)

	-- inlier_map needs to map to our sampled indices
	local inlier_map = n_inlier_map[sorted_i[1]]
	local inlier_map = inlier_map:sub(sample_bounds, points_map:size(2)-sample_bounds, sample_bounds, points_map:size(3)-sample_bounds)
	local inlier_map = inlier_map:reshape(1,inlier_map:size(1)*inlier_map:size(2)):squeeze():byte()

	-- Return best plane, number of inliers and inlier mask
	return inliers, best_plane, inlier_mask, inlier_map
end

-- DEBUG: run this n_iter times, removing sample points each time
inlier_map = nil
best_planes = {}

iter = 1
--for iter=1, params.n_iter do 
while iter < params.n_iter do 
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

	inliers, plane, inlier_mask, inlier_mp = extract_planes( points, points_map, normal_map, indices, iter, params)

	if inlier_mp:sum() > params.reference_window*params.reference_window then
		imap = inlier_mp:reshape(points_map:size(2)-2*params.sample_bounds+1, points_map:size(3)-2*params.sample_bounds+1)
		print(inlier_mp:max())
		print(inlier_mp:min())
		cc_regions_final_r[imap] = cmap[{iter,1}]
		cc_regions_final_g[imap] = cmap[{iter,2}]
		cc_regions_final_b[imap] = cmap[{iter,3}]

		plane.inlier_map = torch.ByteTensor(points_map:size(2), points_map:size(3)):zero()
		plane.inlier_map[{{params.sample_bounds, points_map:size(2)-params.sample_bounds},{params.sample_bounds, points_map:size(3)-params.sample_bounds}}] = imap
		table.insert( best_planes, plane )
		iter = iter + 1
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

if DEBUG then 
	image.display(normal_map)
	image.display(patches_rgb)

	-- Draw resulst
	--[[
	points_rgb[1] = points_r
	points_rgb[2] = points_g
	points_rgb[3] = points_b
	pc_index, pc_mask = pc:get_index_and_mask()
	rgb_map = util.addr.remap(points_rgb, pc_index,pc_mask)
	image.display(rgb_map)
	]]--

	cc_regions_rgb[1] = cc_regions_r
	cc_regions_rgb[2] = cc_regions_g
	cc_regions_rgb[3] = cc_regions_b
	image.display(cc_regions_rgb)

	cc_regions_final_rgb[1] = cc_regions_final_r
	cc_regions_final_rgb[2] = cc_regions_final_g
	cc_regions_final_rgb[3] = cc_regions_final_b
	image.display(cc_regions_final_rgb)
end

-- Write output data, planes, etc ...
plane_fname = string.format("%s/planes.t7",output_dir)
print("saving "..plane_fname)
torch.save(plane_fname, best_planes)