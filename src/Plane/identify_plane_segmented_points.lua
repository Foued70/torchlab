--[[
	A small script for generating segmented sets of points with different colors (eventually) based upon
	the set of input planes created with /src/scripts/find_planes.lua
]]--

require 'gnuplot'
io = require 'io'

gnuplot.setgnuplotexe("/usr/local/bin/gnuplot")
gnuplot.setterm('x11')

-- User specified planes and points
planes_fname = 'planes.t7'
points_fname = 'sweep.xyz' -- note there is an accompanying sweep.meta file

output_fname = 'segmented_points'

-- Load planes and point cloud
planes = torch.load(planes_fname)
point_cloud = PointCloud.PointCloud.new(points_fname)

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

-- Squeeze plane_indices since I don't care about the 1-dim
plane_indices = plane_indices:squeeze()

print("Number of planes", #planes)
print("Number of pointsl", points:size(1))

-- Colormap for different coloring
colormap = image.colormap(#planes):mul(255.0)

-- index_rng = torch.range(1, points:size(1))
pts_rng = torch.range(1,points:size(1))

for plane_idx=1, #planes do
	idx_msk = plane_indices:eq(plane_idx)
	points_on_plane = plane_indices[idx_msk]

	print('output_fname' .. output_fname .. tostring(plane_idx) .. '.pts' )
	print("number of points for plane ", plane_idx, " ",  points_on_plane:size(1))

	output_fh = io.open( output_fname .. tostring(plane_idx) .. '.pts', "w" )
	pts_rng[idx_msk]:apply( 
		function(idx) 
			local point = points[idx]
			local rgb = colormap[plane_idx]
			output_fh:write( point[1] ..' '.. point[2] ..' '.. point[3] ..' '.. rgb[1] ..' '..
				     		 rgb[2] ..' '.. rgb[3] .. '\n')	 
		end
		)
	output_fh:close()
end




