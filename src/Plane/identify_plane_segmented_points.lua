--[[
	A small script for generating segmented sets of points with different colors (eventually) based upon
	the set of input planes created with /src/scripts/find_planes.lua
]]--

-- require 'gnuplot'
io = require 'io'

-- gnuplot.setgnuplotexe("/usr/local/bin/gnuplot")
-- gnuplot.setterm('x11')

-- User specified planes and points
--[[
planes_fname = 'planes.t7'
points_fname = 'sweep.xyz' -- note there is an accompanying sweep.meta file
output_fname = 'segmented_points'
]]--

-- TODO: Use Torch:CmdLine to parse user inputs ... or develop generic pointcloud data loader
planes_fname = '/Users/uriah/Downloads/precise-transit-6548/work/output/_Users_uriah_Downloads_precise-transit-6548_source_po_scan_a_001_sweep_xyz/iterative_saliency_base_9_scale_1.8_n_scale_5_thres_100_normthres_1.0472_minseed_81_minplane_900_normal_var/planes.t7'
points_fname = '/Users/uriah/Downloads/precise-transit-6548/source/po_scan/a/001/sweep.xyz' -- note there is an accompanying sweep.meta file
output_fname = '/Users/uriah/Downloads/precise-transit-6548/work/output/001/segmented_points'

-- Params:
-- Score Threshold for planes
plane_score_threshold = 0.7
point_score_threshold = 0.7

-- Load planes and point cloud
planes = torch.load(planes_fname)
point_cloud = PointCloud.PointCloud.new(points_fname)

-- Extract plane scores
plane_scores = {}
for _,plane in pairs(planes) do
	table.insert(plane_scores, plane.score)
end
plane_scores = torch.Tensor( plane_scores )

-- Initialize matcher, used to match points to planes
matcher = Plane.Matcher.new()

-- Get points
points = point_cloud.points

-- Extract normals from point cloud
normal_map = point_cloud:get_normal_map()
-- Get accompanying indices and mask for point_cloud since not all points have valid normals
index, mask = point_cloud:get_index_and_mask()
normals = util.torch.select3d(normal_map, mask*-1+1)

-- Match points to planes, returning plane index for corresponding points
scores, plane_indices = matcher:match( planes, points:t(), normals:t() )

-- Take only those plane indices corresponding to scores above plane_score_threshold
valid_planes = torch.range(1,#planes)[ plane_scores:gt(plane_score_threshold) ]

-- Squeeze plane_indices  and scores since I don't care about the 1-dim
scores = scores:squeeze()
plane_indices = plane_indices:squeeze()

print("Number of planes", #planes)
print("Number of pointsl", points:size(1))

-- Colormap for different coloring
colormap = image.colormap(#planes):mul(255.0):int()

-- index_rng = torch.range(1, points:size(1))
pts_rng = torch.range(1,points:size(1))

-- Only save data from valid plane set 
valid_planes:apply( function(plane_idx)
	idx_msk = plane_indices:eq(plane_idx)
	points_on_plane = plane_indices[idx_msk]

	print('output_fname' .. output_fname .. tostring(plane_idx) .. '.pts' )
	print("number of points for plane ", plane_idx, " ",  points_on_plane:size(1))

	output_fh = io.open( output_fname .. tostring(plane_idx) .. '.pts', "w" )
	pts_rng[idx_msk]:apply( function(idx) 
		-- Remove points with score below threshold. TODO: vectorize by filtering out these points beforehand
		-- local score = scores[idx]
		if ( scores[idx] > point_score_threshold ) then
			local point = points[idx]
			local rgb = colormap[plane_idx]
			output_fh:write( point[1] ..' '.. point[2] ..' '.. point[3] ..' '.. rgb[1] ..' '..
				     		 rgb[2] ..' '.. rgb[3] .. '\n')	 
		end
	end
	)
	output_fh:close()
end
)




