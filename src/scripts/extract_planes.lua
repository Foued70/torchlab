--[[
	Extract Planes: 
		The method behind this implementation is as follows, 
		--  Use gradient magnitude to eliminate non-smooth regions in the the normal map ( edges, lines )
		--  Bin smooth regions into (N by M)-bins of phi,theta
		--  Within each bin run RANSAC based 3-point plane extraction to extract planes 
			until no more "good" planes can be extracted, where a good plane ( for now )
			is one that represents at least n-points
		--  Finally ( maybe ) within each set of points for a plane identify convex regions 
			using some measure of connectivity

	Notes:
		-- When removing non-smooth regions it might make sense to have a depth varying guassian kernel to that 
		   we can determine non-smooth features in a depth invariant way 
]]--

-- Initialize gnuplot with x11
--[[
require 'gnuplot'
gnuplot.setgnuplotexe("/usr/local/bin/gnuplot")
gnuplot.setterm('x11')
]]--

io = require 'io'
fit_plane = geom.linear_model.fit
residual_fast = geom.linear_model.residual_fast

-- Simple helper function 
function gradient_magnitude( im ) 
	grad_kern = torch.Tensor({{-1,1},{-1,1}})
	dx = torch.conv2( im, grad_kern, 'V')
	dy = torch.conv2( im, grad_kern:t(), 'V')
	return dx:pow(2) + dy:pow(2)
end

-- Simple ransac_fit 
function ransac_fit( points, n_samples, residual_thresh ) 
	-- Generate n_samples random sets of 3 points ... rounding might be a bad thing to do
	local n_points = points:size(1)
	local sample_indices = torch.rand(3,n_samples):mul(n_points):int()
	print("n_points: ", n_points)

	-- Store points and votes from all samples
	local n_pts = torch.Tensor(n_samples,3,3)
	local n_inliers = torch.IntTensor(n_samples)
	local n_planes = {}
	local inlier_mask = torch.ByteTensor(n_points):zero()
	-- For all samples, fit a plane and evaluate how many points vote for that plane
	-- for sample_ind=1, n_samples do 
	torch.range(1,n_samples):apply( function(sample_ind)
		local pts = torch.Tensor(3,3)
		local inds = sample_indices[{{},sample_ind}]

		pts[1] = points[inds[1]]
		pts[2] = points[inds[2]]
		pts[3] = points[inds[3]]

		-- print("pts: ", pts:t())

		-- Fit plane to 3 points
		local current_plane = fit_plane( pts:t() )
		-- print("current_plane: ", current_plane)

		-- Determine number of inliers
		local residuals = residual_fast(current_plane, points:t()):abs() -- DEBUG: start with pts, residuals better be small
        -- local normal_dists = cosine_distance(plane,normals) TODO: add normals into this step 
		local inlier_mask = residuals:lt(residual_thresh)
		local inliers = inlier_mask:sum()

		-- Save results
		n_pts[sample_ind] = pts
		n_inliers[sample_ind] = inliers
		n_planes[sample_ind] = current_plane

		-- print("residuals max: ", residuals:max())
		-- print("residuals min: ", residuals:min())
		-- print("num_inliers: ", residuals:lt(residual_thresh):sum())
		-- print("residuals: ", residuals)	
	end
	)

	-- Return best
	_,sorted_i = torch.sort( n_inliers, true ) -- true for descending
	local current_plane = n_planes[sorted_i[1]]	
	local inliers = n_inliersk[sorted_i[1]] 	
	-- Recompute inlier mask
	local residuals = residual_fast(current_plane, points:t()):abs() -- DEBUG: start with pts, residuals better be small
	local inlier_mask = residuals:lt(residual_thresh)
	-- Recompute plane given inlier mask
	local pts = torch.Tensor(3,inliers)
	pts[1] = points:select(2,1)[inlier_mask]
	pts[2] = points:select(2,2)[inlier_mask]
	pts[3] = points:select(2,3)[inlier_mask]

	-- Fit plane to all points
	local current_plane = fit_plane( pts )

	-- TODO: refit plane given inliers and produce new plane,n_inliers, and inlier_mask
	return current_plane, n_inliers, inlier_mask:squeeze()
end

-- Filenames and whatnot
scan_fname = '/Users/uriah/Downloads/precise-transit-6548/source/po_scan/a/001/sweep.xyz' 
output_fname = 'normals'

-- Params
smoothness_thresh = 0.3

-- Load in pointcloud
pc = PointCloud.PointCloud.new( scan_fname )

points = pc.points
n_points = points:size(1)
print("n_points: ", n_points)
-- Extract normals, etc ...
nmp,dd,phi,theta,pc_mask = pc:get_normal_map()

-- First step identify non-smooth regions
nx = nmp:select(1,1)
ny = nmp:select(1,2)
nz = nmp:select(1,3)

magx = gradient_magnitude( nx )
magy = gradient_magnitude( ny )
magz = gradient_magnitude( nz )

mag = magx + magy + magz 
smooth_mask = mag:lt(smoothness_thresh)

print("size test: ", smooth_mask:eq(0):repeatTensor(3,1,1):size() )

print(smooth_mask:size())
print(pc_mask:size())

mask = smooth_mask + pc_mask:sub(1,smooth_mask:size(1),1,smooth_mask:size(2)):eq(0)
mask = mask:eq(2)

-- DEBUG: display output 

-- TEST: Extract the 'best' plane using RANSAC
n_samples = 1000
residual_thresh = 25.0

n_planes = 10

-- Create colormap for n_planes
cmap = image.colormap(n_planes)

points_rgb = torch.Tensor(points:size()):transpose(2,1):contiguous()
points_r = points_rgb[1]
points_g = points_rgb[2]
points_b = points_rgb[3]

point_inds = torch.range(1,points:size(1)):long()

for plane_ind=1, n_planes do 
	plane,n_inliers,inlier_mask = ransac_fit( points, n_samples, residual_thresh )

	print("n_inliers: ", n_inliers)
	print("inlier_mask:size(): " , inlier_mask:size())
	print("nx:size(): ", nx:size())

	inlier_im = torch.Tensor(nx:size())
	print("pc_mask: ", inlier_im[pc_mask:eq(0)]:size())

	points_r[point_inds[inlier_mask]] = cmap[{plane_ind,1}]
	points_g[point_inds[inlier_mask]] = cmap[{plane_ind,2}]
	points_b[point_inds[inlier_mask]] = cmap[{plane_ind,3}]

	keep_mask = inlier_mask:eq(0)

	-- This must be slow and is gross
	points_old = points:clone()

	-- Garbage collect
	points = nil
	collectgarbage()

	points = torch.Tensor(3,keep_mask:sum())

	points[1] = points_old:t():select(1,1)[keep_mask]
	points[2] = points_old:t():select(1,2)[keep_mask]
	points[3] = points_old:t():select(1,3)[keep_mask]
	-- Transpose back, .. so much transposing
	points = points:t()

	point_inds = point_inds[keep_mask]

end

points_rgb[1] = points_r
points_rgb[2] = points_g
points_rgb[3] = points_b

pc_index, pc_mask = pc:get_index_and_mask()
-- rgb_map = util.addr.remap(pc:get_points(),pc_index,pc_mask)
rgb_map = util.addr.remap(points_rgb, pc_index,pc_mask)

rgb_map_sub = rgb_map:sub(1,3,1,smooth_mask:size(1), 1,smooth_mask:size(2))

disp_map = torch.add(rgb_map_sub, smooth_mask:eq(0):repeatTensor(3,1,1):double())

-- Clamp max output value
disp_map[disp_map:gt(1.0)] = 1.0

image.display( nmp )
image.display( mag )
image.display( smooth_mask )
image.display( mask )
image.display(disp_map)






--[[
	Taking a break from clustering normals ... its probably better to just move forward from here
-- Now cluster normals 

-- Debug show normals ... hopefully there are nice clusters ( there are very nice clusters )

-- create 1-d vectors
n_elements = nx:nElement()
x = nx:resize(1,n_elements):squeeze()
y = ny:resize(1,n_elements):squeeze()
z = nz:resize(1,n_elements):squeeze()
pts_rng = torch.range(1,n_elements)

-- gnuplot.plot(x,y,'.') -- to many results to unpack, lamesauce

-- Create pts file of normals, also colored by normal coordinates for ease of visualization

r = x:add(1):mul(255/2):int()
g = y:add(1):mul(255/2):int()
b = z:add(1):mul(255/2):int()
]]--

--[[
output_fh = io.open( output_fname .. '.pts', "w" )
pts_rng[mask:resize(1,n_elements)]:apply( function(idx) 
		output_fh:write( x[idx] ..' '.. y[idx] ..' '.. z[idx] ..' '.. r[idx] ..' '.. g[idx] ..' '.. b[idx] .. '\n' )
	end
)
output_fh:close()
]]--

--[[
-- K-means clustering ( eventually on a unit sphere )
-- TODO: come up with some good way to identify the number of centroids
n_centroids = 10
n_iters = 10

-- Choose K centroids randomly on a unit sphere 
centroids_angles = torch.rand(2,n_centroids):mul(2.0*math.pi):add(-math.pi)
centroids = geom.util.spherical_angles_to_unit_cartesian(centroids_angles)

points = nmp
dists = torch.Tensor(n_centroids, points:size(2))

-- For now we just iterate some number of times and have no stopping criteria
for iters=1, n_iters do 
	-- For each centroid compute the distances from all points
	for c_idx=1,n_centroids do		
		-- do x, then y, then z just to keep things simple
		dists[c_idx] = points[1]:add(-centroids[{1,c_idx}]):pow(2)
		dists[c_idx] = dists[c_idx] + points[2]:add(-centroids[{2,c_idx}]):pow(2)
		dists[c_idx] = dists[c_idx] + points[3]:add(-centroids[{3,c_idx}]):pow(2)
	end


	-- Assign points to centroids and compute new centroid

	-- Repeat
end
]]--








