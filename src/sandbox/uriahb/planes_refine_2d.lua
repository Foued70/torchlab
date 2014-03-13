--[[
	Refine 2d alignment using planes

	- First step: identify which planes are inliers between the two transforms
	- Second step: using inliers 


	- TODO: clean this up, make a class and make more robust 
	       	Note: plane overlap is a critical part of matching  
]]--
io = require('io')

ArcIO = data.ArcIO

estimate_transform_from_planes = Plane.util.estimate_transform_from_planes

transform_plane = Plane.util.transform_plane
angle_between = geom.util.angle_between

job_id = 'precise-transit-6548'  

planes_arc_io = ArcIO.new( job_id, 'region-growing-random-002' )

tf_arc_io = ArcIO.new( job_id, "tf_from_planes")
tf2d = tf_arc_io:loadTorch("transformations2d", "transformations2d")


scan_ids = {1,2,3,4,6,7,8,9,10,11,12,13,14,15,16,17,18,19,21,22,23,24,25,26,27}

-- Load in planes
scan0 = 1
scan1 = 2

--scan_ids = {9,10,11,12,13,14,15,16,17,18,19,21,22,23,24,25,26,27}

scan_data0 = planes_arc_io:loadTorch("refitted_planes", string.format("scan%.3d", scan0))
scan_data1 = planes_arc_io:loadTorch("refitted_planes", string.format("scan%.3d", scan1))

planes0 = scan_data0.planes
planes1 = scan_data1.planes

masks_data0 = planes_arc_io:loadTorch( "merged_planes", string.format("%.3d", scan0))
masks_data1 = planes_arc_io:loadTorch( "merged_planes", string.format("%.3d", scan1))
masks0 = masks_data0.plane_masks
masks1 = masks_data1.plane_masks

tf2d = tf_arc_io:loadTorch("transformations2d", "transformations2d") -- hmm bad naming?
tf = tf2d[scan0].rough_transformation


t0 = torch.tic()

tf = tf2d[scan0].rough_transformation

-- Identify potential pairs of planes between scans given hypothesis transformation
-- Approach: transform planes from scan0 into scan1 then for each plane find the closest
-- 		     plane that is within some residual and angle threshold
tfd_planes = {}
for i = 1,#planes1 do 	
	table.insert( tfd_planes, transform_plane(planes1[i], tf) )	
end

residual_thresh = 50
normal_thresh = math.pi/8

matches = {}
for i=1,#planes0 do
	closest_residual = nil
	closest = nil	 
	for j = 1,#tfd_planes do 
		local angle = angle_between( planes0[i].eqn:sub(1,3) , tfd_planes[j].eqn:sub(1,3) )		
		local residual = math.abs(torch.dot( planes0[i].eqn:sub(1,3), torch.add(tfd_planes[j].centroid, -planes0[i].centroid) ))
		--print(string.format("angle between [%d,%d]: %f", i,j,angle))
		if angle < normal_thresh and residual < residual_thresh then
			table.insert( matches, {i,j} )
			print(string.format("[%d,%d]: %f, %f", i,j,angle, residual))
			if closest == nil or closest_residual > residual then 
				closest = j 
				closest_residual = residual
			end
		end
	end
	--table.insert( matches, {i,closest} )
end

for i=1,#matches do
	print(matches[i][1], matches[i][2])
end

-- Reduced match set
matches_reduced = {}
for i=1,#matches do 
	if matches[i][2] ~= nil then
		table.insert( matches_reduced, matches[i] ) 
	end
end
print("matches_reduced")
for i=1,#matches_reduced do
	print(matches_reduced[i][1], matches_reduced[i][2])
end

-- Acquire weights for each plane 
match_weights = {}
for i=1,#matches_reduced do 
	msk0 = masks0[{matches_reduced[i][1], {}, {}}]
	msk1 = masks1[{matches_reduced[i][2], {}, {}}]
	--table.insert( match_weights, msk1:sum() + msk0:sum() )
	table.insert( match_weights, 1 )
end


-- Given matches run RANSAC to find a good trio of plane matches ... because they aren't all good

-- Fix validate_tf
function validate_tf( est_tf, planes0, planes1, normal_thresh, residual_thresh )
	local tfd_planes = {}
	for i = 1,#planes1 do 		
		table.insert( tfd_planes, transform_plane(planes1[i], est_tf) )	
	end	
	-- Next count the number of planes that are within the angle/residual dist
	local match_matrix = torch.Tensor(#planes0,#tfd_planes):zero()
	for i=1,#planes0 do 
		for j = 1,#tfd_planes do 
			local angle = angle_between( planes0[i].eqn:sub(1,3) , tfd_planes[j].eqn:sub(1,3) )		
			local residual = math.abs(torch.dot( planes0[i].eqn:sub(1,3), torch.add(tfd_planes[j].centroid, -planes0[i].centroid) ))
			--print(string.format("angle between [%d,%d]: %f", i,j,angle))
			if angle < normal_thresh and residual < residual_thresh then
				match_matrix[{i,j}] = 1
			end
		end
	end
	return match_matrix:sum()
end

function validate_matched_tf( est_tf, planes0, planes1, match_inds, match_weights, normal_thresh, residual_thresh )
	local tfd_planes = {}
	local inlier_cnt = 0
	local inliers = {}
	for i = 1,#match_inds do 			
		local plane = planes0[match_inds[i][1] ] 
		local tfd_plane = transform_plane(planes1[match_inds[i][2] ], est_tf)
		local angle = angle_between( plane.eqn:sub(1,3) , tfd_plane.eqn:sub(1,3) )		
		local residual = math.abs(torch.dot( plane.eqn:sub(1,3), torch.add(tfd_plane.centroid, -plane.centroid) ))
		--print(string.format("angle between [%d,%d]: %f", i,j,angle))
		if angle < normal_thresh and residual < residual_thresh then
			inlier_cnt = inlier_cnt + match_weights[i]
			table.insert( inliers, i )
		end		 
	end	
	return inlier_cnt, inliers
end


sample0 = {nil, nil, nil}
sample1 = {nil, nil, nil}

best_tf = nil 
best_inlier_cnt = nil
best_inliers = nil 
best_inds = nil

angle_thresh = math.pi/8

residual_thresh = 15
normal_thresh = math.pi/8

n_samples = 400
for i = 1,n_samples do 
	-- Randomly select 3 indices until we find a "good" pair 
	local inds
	local i1
	local i2
	local i3
	while true do 
		inds = torch.rand(3):mul(#matches_reduced):long()+1		
		i1 = matches_reduced[inds[1] ][1]
		i2 = matches_reduced[inds[2] ][1]
		i3 = matches_reduced[inds[3] ][1]		
		local normals1 = planes0[i1].eqn:sub(1,3)
		local normals2 = planes0[i2].eqn:sub(1,3)
		local normals3 = planes0[i3].eqn:sub(1,3)
		local angle12 = angle_between( normals1, normals2 )
		local angle13 = angle_between( normals1, normals3 )
		local angle23 = angle_between( normals2, normals3 )
		if angle12 > angle_thresh and math.pi-angle12 > angle_thresh and
		   angle13 > angle_thresh and math.pi-angle13 > angle_thresh and	
		   angle23 > angle_thresh and math.pi-angle23 > angle_thresh then
		   break
		end
	end

	local j1 = matches_reduced[inds[1] ][2]
	local j2 = matches_reduced[inds[2] ][2]
	local j3 = matches_reduced[inds[3] ][2]		

	sample0[1] = planes0[i1]
	sample0[2] = planes0[i2]
	sample0[3] = planes0[i3]

	sample1[1] = planes1[j1]
	sample1[2] = planes1[j2]
	sample1[3] = planes1[j3]

	est_tf = estimate_transform_from_planes( sample0, sample1 )
	inlier_cnt, inliers = validate_matched_tf( est_tf, planes0, planes1, matches_reduced, match_weights, normal_thresh, residual_thresh )

	if best_tf == nil or inlier_cnt > best_inlier_cnt then
		best_tf = est_tf
		best_inlier_cnt = inlier_cnt
		best_inliers = inliers
		best_inds = inds
	end

	print("inlier_cnt: ", inlier_cnt )
end

-- TODO: refit using all matching inliers ... can't do this in torch since it doesn't have pseudo-inverse 
-- 											  will do in Eigen or something ... or Julia!
--[[
sample0 = {}
sample1 = {}
for i = 1,#inliers do 
	table.insert( sample0, planes0[matches_reduced[inliers[i] ][1] ] )
	table.insert( sample1, planes0[matches_reduced[inliers[i] ][2] ] )
end
best_tf = estimate_transform_from_planes( sample0, sample1 )
]]-- THIS THING!

print("dt: ", torch.toc(t0))
		
--best_tf[{{1,3},{1,3}}] = torch.inverse(best_tf[{{1,3},{1,3}}])
best_tf[{{1,3},{4}}] = -best_tf[{{1,3},{4}}]
print("best_inliers: ", best_inlier_cnt)

cmap = image.colormap(#matches):mul(255.0):int()

-- Output match inds 

scan_num = scan0
pc = planes_arc_io:getScan( scan_num )
points_map = pc:get_xyz_map()

-- Load in merged planes 
input_data = planes_arc_io:loadTorch( "merged_planes", string.format("%.3d", scan_num))

-- Load in transform ... TODO, need a way to access things outside of work_id 	
--transform = tf
transform = torch.eye(4)

-- Transform points map 
pts = torch.reshape(points_map, 3,points_map:size(2)*points_map:size(3))
pts = torch.cat( pts, torch.Tensor(1,points_map:size(2)*points_map:size(3)):fill(1), 1)	
pts = transform*pts
points_map = torch.reshape(pts:sub(1,3), 3, points_map:size(2), points_map:size(3))	

plane_masks = input_data.plane_masks

-- For each plane mask convert to x,y,z,r,g,b pointcloud and write to .pts file 
num_planes = plane_masks:size(1)
print("Number of planes: ", num_planes)
local output_fname = tf_arc_io:workStr("match_hypotheses", string.format("%.3dorig.pts",scan0))										
print("Writing segmented region: ", output_fname)
local output_fh = io.open( output_fname, "w" )

for i = 1,3 do 
--torch.range(1,num_planes):apply( function( plane_idx )
	plane_idx = matches_reduced[best_inds[i]][1]	
	local mask = plane_masks[{plane_idx, {}, {}}]
	local rgb = cmap[i]
	--local rgb = torch.Tensor({255,255,255})
	--if matches[plane_idx] ~= nil then 
		--rgb = cmap[plane_idx]
		--print(plane_idx)
	--end
	local points_x = points_map:select(1,1)[mask]
	local points_y = points_map:select(1,2)[mask]
	local points_z = points_map:select(1,3)[mask]
	-- Output point data for visualization 
	torch.range(1,points_x:nElement()):apply( function( idx )
		
		output_fh:write( points_x[idx] ..' '.. points_y[idx] ..' '.. points_z[idx] ..' '.. 
						 rgb[1] ..' '..   rgb[2] ..' '.. rgb[3] .. '\n')	 
	end
	)		
end
--)
output_fh:close()

-- Output match inds 

scan_num = scan1
pc = planes_arc_io:getScan( scan_num )
points_map = pc:get_xyz_map()

-- Load in merged planes 
input_data = planes_arc_io:loadTorch( "merged_planes", string.format("%.3d", scan_num))

-- Load in transform ... TODO, need a way to access things outside of work_id 	
transform = tf
--transform = torch.eye(4)

-- Transform points map 
pts = torch.reshape(points_map, 3,points_map:size(2)*points_map:size(3))
pts = torch.cat( pts, torch.Tensor(1,points_map:size(2)*points_map:size(3)):fill(1), 1)	
pts = transform*pts
points_map = torch.reshape(pts:sub(1,3), 3, points_map:size(2), points_map:size(3))	

plane_masks = input_data.plane_masks

-- For each plane mask convert to x,y,z,r,g,b pointcloud and write to .pts file 
num_planes = plane_masks:size(1)
print("Number of planes: ", num_planes)
local output_fname = tf_arc_io:workStr("match_hypotheses", string.format("%.3dnew.pts",scan0))										
print("Writing segmented region: ", output_fname)
local output_fh = io.open( output_fname, "w" )

for i = 1,3 do 
--torch.range(1,num_planes):apply( function( plane_idx )
	plane_idx = matches_reduced[best_inds[i]][2]	
	local mask = plane_masks[{plane_idx, {}, {}}]
	local rgb = cmap[i]
	--local rgb = torch.Tensor({255,255,255})
	--if matches[plane_idx] ~= nil then 
		--rgb = cmap[plane_idx]
		--print(plane_idx)
	--end
	local points_x = points_map:select(1,1)[mask]
	local points_y = points_map:select(1,2)[mask]
	local points_z = points_map:select(1,3)[mask]
	-- Output point data for visualization 
	torch.range(1,points_x:nElement()):apply( function( idx )
		
		output_fh:write( points_x[idx] ..' '.. points_y[idx] ..' '.. points_z[idx] ..' '.. 
						 rgb[1] ..' '..   rgb[2] ..' '.. rgb[3] .. '\n')	 
	end
	)		
end
--)
output_fh:close()


-- Output match hypotheses

scan_num = scan0
pc = planes_arc_io:getScan( scan_num )
points_map = pc:get_xyz_map()

-- Load in merged planes 
input_data = planes_arc_io:loadTorch( "merged_planes", string.format("%.3d", scan_num))

-- Load in transform ... TODO, need a way to access things outside of work_id 	
--transform = tf
transform = torch.eye(4)

-- Transform points map 
pts = torch.reshape(points_map, 3,points_map:size(2)*points_map:size(3))
pts = torch.cat( pts, torch.Tensor(1,points_map:size(2)*points_map:size(3)):fill(1), 1)	
pts = transform*pts
points_map = torch.reshape(pts:sub(1,3), 3, points_map:size(2), points_map:size(3))	

plane_masks = input_data.plane_masks

-- For each plane mask convert to x,y,z,r,g,b pointcloud and write to .pts file 
num_planes = plane_masks:size(1)
print("Number of planes: ", num_planes)
local output_fname = tf_arc_io:workStr("matches", string.format("%.3dorig.pts",scan0))										
print("Writing segmented region: ", output_fname)
local output_fh = io.open( output_fname, "w" )

for i = 1,#matches do 
--torch.range(1,num_planes):apply( function( plane_idx )
	plane_idx = matches[i][1]
	if matches[i][2] ~= nil then
		local mask = plane_masks[{plane_idx, {}, {}}]
		local rgb = cmap[i]
		--local rgb = torch.Tensor({255,255,255})
		--if matches[plane_idx] ~= nil then 
			--rgb = cmap[plane_idx]
			--print(plane_idx)
		--end
		local points_x = points_map:select(1,1)[mask]
		local points_y = points_map:select(1,2)[mask]
		local points_z = points_map:select(1,3)[mask]
		-- Output point data for visualization 
		torch.range(1,points_x:nElement()):apply( function( idx )
			
			output_fh:write( points_x[idx] ..' '.. points_y[idx] ..' '.. points_z[idx] ..' '.. 
							 rgb[1] ..' '..   rgb[2] ..' '.. rgb[3] .. '\n')	 
		end
		)	
	end
end
--)
output_fh:close()


scan_num = scan1
pc = planes_arc_io:getScan( scan_num )
points_map = pc:get_xyz_map()

-- Load in merged planes 
input_data = planes_arc_io:loadTorch( "merged_planes", string.format("%.3d", scan_num))

-- Load in transform ... TODO, need a way to access things outside of work_id 	
--transform = torch.eye(4)
transform = best_tf
print("best_tf: ", best_tf)


-- Transform points map 
pts = torch.reshape(points_map, 3,points_map:size(2)*points_map:size(3))
pts = torch.cat( pts, torch.Tensor(1,points_map:size(2)*points_map:size(3)):fill(1), 1)	
pts = transform*pts
points_map = torch.reshape(pts:sub(1,3), 3, points_map:size(2), points_map:size(3))	

plane_masks = input_data.plane_masks

-- For each plane mask convert to x,y,z,r,g,b pointcloud and write to .pts file 
--num_planes = plane_masks:size(1)
--print("Number of planes: ", num_planes)
local output_fname = tf_arc_io:workStr("matches", string.format("%.3dplanes.pts",scan0))										
print("Writing segmented region: ", output_fname)
local output_fh = io.open( output_fname, "w" )

--torch.range(1,num_planes):apply( function( ix )
for i = 1,#matches do
	plane_idx = matches[i][2]
	if plane_idx ~= nil then 		
		--print(plane_idx)		
		local mask = plane_masks[{plane_idx, {}, {}}]
		--local rgb = torch.Tensor({255,255,255})
		
		rgb = cmap[i]
		
		local points_x = points_map:select(1,1)[mask]
		local points_y = points_map:select(1,2)[mask]
		local points_z = points_map:select(1,3)[mask]
		-- Output point data for visualization 
		torch.range(1,points_x:nElement()):apply( function( idx )
			
			output_fh:write( points_x[idx] ..' '.. points_y[idx] ..' '.. points_z[idx] ..' '.. 
							 rgb[1] ..' '..   rgb[2] ..' '.. rgb[3] .. '\n')	 
		end
		)
	end	
end
--)
output_fh:close()

scan_num = scan1
pc = planes_arc_io:getScan( scan_num )
points_map = pc:get_xyz_map()

-- Load in merged planes 
input_data = planes_arc_io:loadTorch( "merged_planes", string.format("%.3d", scan_num))

-- Load in transform ... TODO, need a way to access things outside of work_id 	
--transform = torch.eye(4)
transform = tf
print("tf: ", tf)

-- Transform points map 
pts = torch.reshape(points_map, 3,points_map:size(2)*points_map:size(3))
pts = torch.cat( pts, torch.Tensor(1,points_map:size(2)*points_map:size(3)):fill(1), 1)	
pts = transform*pts
points_map = torch.reshape(pts:sub(1,3), 3, points_map:size(2), points_map:size(3))	

plane_masks = input_data.plane_masks

-- For each plane mask convert to x,y,z,r,g,b pointcloud and write to .pts file 
--num_planes = plane_masks:size(1)
--print("Number of planes: ", num_planes)
local output_fname = tf_arc_io:workStr("matches", string.format("%.3dprev.pts",scan0))										
print("Writing segmented region: ", output_fname)
local output_fh = io.open( output_fname, "w" )

--torch.range(1,num_planes):apply( function( ix )
for i = 1,#matches do
	plane_idx = matches[i][2]
	if plane_idx ~= nil then 		
		--print(plane_idx)		
		local mask = plane_masks[{plane_idx, {}, {}}]
		--local rgb = torch.Tensor({255,255,255})
		
		rgb = cmap[i]
		
		local points_x = points_map:select(1,1)[mask]
		local points_y = points_map:select(1,2)[mask]
		local points_z = points_map:select(1,3)[mask]
		-- Output point data for visualization 
		torch.range(1,points_x:nElement()):apply( function( idx )
			
			output_fh:write( points_x[idx] ..' '.. points_y[idx] ..' '.. points_z[idx] ..' '.. 
							 rgb[1] ..' '..   rgb[2] ..' '.. rgb[3] .. '\n')	 
		end
		)
	end	
end
--)
output_fh:close()

--[[


-- Output transformed plane points ... raw copy paste 
scan_num = scan0
pc = planes_arc_io:getScan( scan_num )
points_map = pc:get_xyz_map()

-- Load in merged planes 
input_data = planes_arc_io:loadTorch( "merged_planes", string.format("%.3d", scan_num))

-- Load in transform ... TODO, need a way to access things outside of work_id 	
transform = torch.eye(4)

-- Transform points map 
pts = torch.reshape(points_map, 3,points_map:size(2)*points_map:size(3))
pts = torch.cat( pts, torch.Tensor(1,points_map:size(2)*points_map:size(3)):fill(1), 1)	
pts = transform*pts
points_map = torch.reshape(pts:sub(1,3), 3, points_map:size(2), points_map:size(3))	

cmap = input_data.colormap:mul(255.0):int()
plane_masks = input_data.plane_masks

-- For each plane mask convert to x,y,z,r,g,b pointcloud and write to .pts file 
num_planes = plane_masks:size(1)
print("Number of planes: ", num_planes)
local output_fname = tf_arc_io:workStr("segmented_points_cull", string.format("%.3dchild.pts",scan0))										
print("Writing segmented region: ", output_fname)
local output_fh = io.open( output_fname, "w" )

torch.range(1,num_planes):apply( function( plane_idx ) 
	local mask = plane_masks[{plane_idx, {}, {}}]
	local points_x = points_map:select(1,1)[mask]
	local points_y = points_map:select(1,2)[mask]
	local points_z = points_map:select(1,3)[mask]
	-- Output point data for visualization 
	torch.range(1,points_x:nElement()):apply( function( idx )
		local rgb = cmap[plane_idx]
		output_fh:write( points_x[idx] ..' '.. points_y[idx] ..' '.. points_z[idx] ..' '.. 
						 rgb[1] ..' '..   rgb[2] ..' '.. rgb[3] .. '\n')	 
	end
	)
end
)
output_fh:close()

scan_num = scan1
pc = planes_arc_io:getScan( scan_num )
points_map = pc:get_xyz_map()

-- Load in merged planes 
input_data = planes_arc_io:loadTorch( "merged_planes", string.format("%.3d", scan_num))

-- Load in transform ... TODO, need a way to access things outside of work_id 	
transform = tf

-- Transform points map 
pts = torch.reshape(points_map, 3,points_map:size(2)*points_map:size(3))
pts = torch.cat( pts, torch.Tensor(1,points_map:size(2)*points_map:size(3)):fill(1), 1)	
pts = transform*pts
points_map = torch.reshape(pts:sub(1,3), 3, points_map:size(2), points_map:size(3))	

cmap = input_data.colormap:mul(255.0):int()
plane_masks = input_data.plane_masks

-- For each plane mask convert to x,y,z,r,g,b pointcloud and write to .pts file 
num_planes = plane_masks:size(1)
print("Number of planes: ", num_planes)
local output_fname = tf_arc_io:workStr("segmented_points_cull", string.format("%.3dparent.pts",scan0))										
print("Writing segmented region: ", output_fname)
local output_fh = io.open( output_fname, "w" )

torch.range(1,num_planes):apply( function( plane_idx ) 
	local mask = plane_masks[{plane_idx, {}, {}}]
	local points_x = points_map:select(1,1)[mask]
	local points_y = points_map:select(1,2)[mask]
	local points_z = points_map:select(1,3)[mask]
	-- Output point data for visualization 
	torch.range(1,points_x:nElement()):apply( function( idx )
		local rgb = cmap[plane_idx]
		output_fh:write( points_x[idx] ..' '.. points_y[idx] ..' '.. points_z[idx] ..' '.. 
						 rgb[1] ..' '..   rgb[2] ..' '.. rgb[3] .. '\n')	 
	end
	)
end
)
output_fh:close()


scan_num = scan1
pc = planes_arc_io:getScan( scan_num )
points_map = pc:get_xyz_map()

-- Load in merged planes 
input_data = planes_arc_io:loadTorch( "merged_planes", string.format("%.3d", scan_num))

-- Load in transform ... TODO, need a way to access things outside of work_id 	
transform = best_tf

-- Transform points map 
pts = torch.reshape(points_map, 3,points_map:size(2)*points_map:size(3))
pts = torch.cat( pts, torch.Tensor(1,points_map:size(2)*points_map:size(3)):fill(1), 1)	
pts = transform*pts
points_map = torch.reshape(pts:sub(1,3), 3, points_map:size(2), points_map:size(3))	

cmap = input_data.colormap:mul(255.0):int()
plane_masks = input_data.plane_masks

-- For each plane mask convert to x,y,z,r,g,b pointcloud and write to .pts file 
num_planes = plane_masks:size(1)
print("Number of planes: ", num_planes)
local output_fname = tf_arc_io:workStr("segmented_points_cull", string.format("%.3dparent_refit.pts",scan0))										
print("Writing segmented region: ", output_fname)
local output_fh = io.open( output_fname, "w" )

torch.range(1,num_planes):apply( function( plane_idx ) 
	local mask = plane_masks[{plane_idx, {}, {}}]
	local points_x = points_map:select(1,1)[mask]
	local points_y = points_map:select(1,2)[mask]
	local points_z = points_map:select(1,3)[mask]
	-- Output point data for visualization 
	torch.range(1,points_x:nElement()):apply( function( idx )
		local rgb = cmap[plane_idx]
		output_fh:write( points_x[idx] ..' '.. points_y[idx] ..' '.. points_z[idx] ..' '.. 
						 rgb[1] ..' '..   rgb[2] ..' '.. rgb[3] .. '\n')	 
	end
	)
end
)
output_fh:close()


]]--

