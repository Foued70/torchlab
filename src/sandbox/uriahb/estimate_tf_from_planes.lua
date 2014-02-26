--[[
	Given a to sets of planes with correspondences and a homogenous tranformation identify the 
	tranformation by solving the least-squares system

	-- So far my thoughts are that without a good plane similarity measure there are too 
	   many possible match combinations to make this an effective method. Additionally 
	   I need to refine my validation step.

]]-- 

-- torch doesn't have a determinant operation, how dumb is that
-- Input 3x3 matrix 
function det3x3( matrix )
	return matrix[{{1},{1}}]*matrix[{{2},{2}}]*matrix[{{3},{3}}] + 
	       matrix[{{1},{2}}]*matrix[{{2},{3}}]*matrix[{{3},{1}}] + 
	       matrix[{{1},{3}}]*matrix[{{2},{1}}]*matrix[{{3},{2}}] - 
	       matrix[{{1},{3}}]*matrix[{{2},{2}}]*matrix[{{3},{1}}] - 
	       matrix[{{1},{2}}]*matrix[{{2},{1}}]*matrix[{{3},{3}}] - 
	       matrix[{{1},{1}}]*matrix[{{2},{3}}]*matrix[{{3},{2}}]
end

io = require('io')

quaternion = geom.quaternion
transform_plane = Plane.util.transform_plane
angle_between = geom.util.angle_between

estimate_transform_from_planes = Plane.util.estimate_transform_from_planes

ArcIO = data.ArcIO

selectdim = util.torch.selectdim


scan0 = 17
scan1 = 18

job_id = 'precise-transit-6548'  
work_id = 'region-growing-random-002'
in_arc_io = ArcIO.new( job_id, work_id )

out_arc_io = ArcIO.new( job_id, "tf_from_planes")
out_arc_io.scan_num = scan1


scan_data0 = in_arc_io:loadTorch("refitted_planes", string.format("scan%.3d", scan0))
scan_data1 = in_arc_io:loadTorch("refitted_planes", string.format("scan%.3d", scan1))

planes0 = scan_data0.planes
planes1 = scan_data1.planes

--[[
-- Stack useful information 
normals0 = torch.Tensor(3,#planes0)
centroids0 = torch.Tensor(3,#planes0)
ds0 = torch.Tensor(#planes0)
for i = 1,#planes0 do 
	normals0[{{},{i}}] = planes0[i].eqn:sub(1,3)
	ds0[{i}] = planes0[i].eqn[{4}]
	centroids0[{{},{i}}] = planes0[i].centroid
end

normals1 = torch.Tensor(3,#planes1)
centroids1 = torch.Tensor(3,#planes1)
ds1 = torch.Tensor(#planes1)
for i = 1,#planes1 do 
	normals1[{{},{i}}] = planes1[i].eqn:sub(1,3)
	ds1[{i}] = planes1[i].eqn[{4}]
	centroids1[{{},{i}}] = planes1[i].centroid
end
]]--

print("#planes0: ", #planes0)
print("#planes1: ", #planes1)

-- Choose top 20 largest planes from each set 

-- Test set of planes, these ones are nice and orthogonal, basically give us rotations already
--[[
planes = {}

plane = {}
plane.eqn = torch.Tensor({1,0,0,0})
plane.centroid = torch.Tensor({3,0,0})
plane.eqn[{4}] = torch.dot(plane.eqn:sub(1,3), plane.centroid)
--plane.centroid = torch.rand(3)
table.insert(planes, plane)

plane = {}
plane.eqn = torch.Tensor({0,1,0,0})
plane.centroid = torch.Tensor({0,2,0.1})
plane.eqn[{4}] = torch.dot(plane.eqn:sub(1,3), plane.centroid)
--plane.centroid = torch.rand(3)
table.insert(planes, plane)

plane = {}
plane.eqn = torch.Tensor({0,0,1,0})
plane.centroid = torch.Tensor({0,1,2})
plane.eqn[{4}] = torch.dot(plane.eqn:sub(1,3), plane.centroid)
--plane.centroid = torch.rand(3)
table.insert(planes, plane)

print("planes")
for i = 1,#planes do 
	print(planes[i].eqn)
end

-- Transform planes by a given transformation, currently no translation
axis = torch.Tensor({1,0,0})
axis = axis/axis:norm()
angle = (torch.rand(1)*math.pi):squeeze()
q = quaternion.from_axis_angle( axis, angle )
rotation = quaternion.to_rotation_matrix(q)

print("rotation: ", rotation)

tf = torch.eye(4)
tf[{{1,3},{1,3}}] = rotation

translation = torch.rand(3)

tf[{{1,3},{4}}] = translation

print("tf: ", tf)


tfd_planes = {}

for i = 1,#planes do 
	table.insert( tfd_planes, transform_plane(planes[i], tf) )	
end
print("tfd_planes")
for i = 1,#tfd_planes do 
	print(tfd_planes[i].eqn)
end

tf_est = estimate_transform_from_planes( tfd_planes, planes )

print("tf: ", tf )
print("tf_est: ", tf_est)
print("tf - tf_est: ", tf-tf_est)
]]--
--[[
	RANSAC test:
		- randomly choose 3 planes from each scan
		- check that all normal differences are great enough between 
		  planes so that we only try good possibilities
]]--


-- Validate a set of samples by checking that all pairs angle_between is greater than thresh
function validate_samples( normals, centroids, angle_thresh )

	--[[
	local is_valid = true
	for i = 1,normals:size(2) do 
		for j = 1,normals:size(2) do
			if i ~= j then 			
				-- TODO, wrap around pi/2 since its just as bad to have two planes with opposite normals	
				angle = angle_between( normals[{{}, {i}}], normals[{{},{j}}] )
				--print(string.format("angle[%d,%d]: %f", i,j,angle))
				if angle < angle_thresh or math.pi-angle < angle_thresh then
					is_valid = false
					break
				end	
			end
		end
	end

	]]--
	

	local is_valid = true
	local descriptor = torch.Tensor(3)
	local angle12 = angle_between( normals[{{}, {1}}], normals[{{},{2}}] )
	local angle13 = angle_between( normals[{{}, {1}}], normals[{{},{3}}] )
	local angle23 = angle_between( normals[{{}, {2}}], normals[{{},{3}}] )
	descriptor[1] = angle12
	descriptor[2] = angle13
	descriptor[3] = angle23
	
	for i = 1,3 do 
		if descriptor[i] < angle_thresh or math.pi-descriptor[i] < angle_thresh then
			is_valid = false
			break
		end	
	end
	if is_valid == true then 
		--print(string.format("[%f,%f,%f]", angle12, angle13, angle23))
	end
	return is_valid, descriptor
end

function match_samples( samples0, samples1 )
	match_matrix = torch.Tensor(#planes0,#tfd_planes):zero()

	corr_list = torch.Tensor( #samples0*#samples1 )	
	inds = torch.Tensor(2,corr_list:nElement())
	--print(inds)
	cnt = 1 
	for i = 1,#samples0 do 
		for j = 1,#samples1 do 
			--ind = torch.LongTensor({(i-1)*#samples0 + j})						
			err = (samples0[i].descriptor:add(-samples1[j].descriptor)):pow(2):sum()
			corr_list[cnt] = err 
			inds[{{1},{cnt}}] = i
			inds[{{2},{cnt}}] = j
			--print(corr_list[cnt])
			--print(inds[{{},{cnt}}])
			cnt = cnt + 1
		end
	end	
	sorted_vals, s_inds = torch.sort(corr_list)

	sorted_inds = inds:index(2,s_inds)

	return sorted_vals, sorted_inds
end

function extract_sample_set( inds, normals, centroids, ds, angle_thresh )
	sample_normals = normals:index(2,inds)
	sample_centroids = centroids:index(2,inds)
	is_valid, descriptor = validate_samples( sample_normals, sample_centroids, angle_thresh )			
	sample_ds = ds[inds]

	sample_set = {}			
	for i = 1,sample_normals:size(2) do 
		plane = {}				
		plane.eqn = torch.cat(sample_normals[{{}, {i}}]:squeeze(), torch.Tensor({sample_ds[{i}]}))
		plane.centroid = sample_centroids[{{}, {i}}]
		table.insert( sample_set, plane )				
	end	
	sample_set.descriptor = descriptor	
	sample_set.inds = inds									
	return is_valid, sample_set
end

function generate_samples( normals, centroids, ds, angle_thresh, n_samples ) 
	-- Generate n_samples valid samples from each plane set
	attempt_cnt = 0
	sample_cnt = 0
	sample_sets = {}
	while sample_cnt < n_samples do 
		attempt_cnt = attempt_cnt + 1
		inds = torch.rand(3):mul(normals:size(2)):long()+1		
		is_valid, sample_set = extract_sample_set( inds, normals, centroids, ds, angle_thresh )
		if is_valid then 
			table.insert(sample_sets, sample_set)
			sample_cnt  = sample_cnt + 1	
		end
		--[[
		sample_normals = normals:index(2,inds)
		is_valid = validate_samples( sample_normals, angle_thresh )		
		if is_valid then 
			sample_centroids = centroids:index(2,inds)
			sample_ds = ds[inds]

			sample_set = {}			
			for i = 1,sample_normals:size(2) do 
				plane = {}				
				plane.eqn = torch.cat(sample_normals[{{}, {i}}]:squeeze(), torch.Tensor({sample_ds[{i}]}))
				plane.centroid = sample_centroids[{{}, {i}}]
				table.insert( sample_set, plane )				
			end											
			sample_set.inds = inds
			table.insert(sample_sets, sample_set)
			sample_cnt  = sample_cnt + 1			
		end
		]]--
	end
	print("attempt_cnt: ", attempt_cnt)
	return sample_sets
end

function validate_tf( est_tf, planes0, planes1, normal_thresh, residual_thresh )
	local tfd_planes = {}
	for i = 1,#planes1 do 		
		table.insert( tfd_planes, transform_plane(planes0[i], est_tf) )	
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

function extract_ground( planes )
	-- Find ground plane
	local min_i = nil
	local min_z = nil
	for i = 1,#planes do 
		angle = angle_between( planes[i].eqn:sub(1,3), torch.Tensor({0,0,-1}) )		
		if angle < math.pi/4 then
			if min_z == nil then
				min_i = i
				min_z = planes[i].centroid[3]
			end
			if planes[i].centroid[3] < min_z then
				min_i = i
				min_z = planes[i].centroid[3]
			end		
		end
	end
	print("normal: ", planes[min_i].eqn)
	print("centroid: ", planes[min_i].centroid)
	return planes[min_i], min_i
end

-- Return all planes that are neither ceiling or floor
function extract_walls( planes )
	local walls = {}
	local wall_inds = {}
	for i = 1,#planes do 
		local angle_f = angle_between( planes[i].eqn:sub(1,3), torch.Tensor({0,0,-1}) )
		local angle_c = angle_between( planes[i].eqn:sub(1,3), torch.Tensor({0,0,1}) )		
		if angle_f > math.pi/4 and angle_c > math.pi/4 then
			table.insert( walls, planes[i] )
			table.insert( wall_inds, i )
		end
	end
	return walls, torch.LongTensor(wall_inds)
end

function  output_walls( scan_num, inds )	
	pc = in_arc_io:getScan( scan_num )
	points_map = pc:get_xyz_map()
	-- Load in merged planes 
	input_data = in_arc_io:loadTorch( "merged_planes", string.format("%.3d", scan_num))

	cmap = input_data.colormap:mul(255.0):int()
	plane_masks = input_data.plane_masks

	-- Load in transform ... TODO, need a way to access things outside of work_id 	
		transform = in_arc_io:loadTorch("transformations", string.format("sweep%.3d", scan_num))

		-- Transform points map 
		pts = torch.reshape(points_map, 3,points_map:size(2)*points_map:size(3))
		pts = torch.cat( pts, torch.Tensor(1,points_map:size(2)*points_map:size(3)):fill(1), 1)	
		pts = transform*pts
		points_map = torch.reshape(pts:sub(1,3), 3, points_map:size(2), points_map:size(3))	

	-- For each plane mask convert to x,y,z,r,g,b pointcloud and write to .pts file 
	num_planes = plane_masks:size(1)
	print("Number of planes: ", num_planes)
	local output_fname = out_arc_io:workStr("wall_planes", string.format("%.3d.pts",scan_num))										
	print("Writing segmented region: ", output_fname)
	local output_fh = io.open( output_fname, "w" )
	
	inds:apply( function( plane_idx ) 
		local mask = plane_masks[{{plane_idx}, {}, {}}]		
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
end

function  output_plane( scan_num, ind )	
	pc = in_arc_io:getScan( scan_num )
	points_map = pc:get_xyz_map()
	-- Load in merged planes 
	input_data = in_arc_io:loadTorch( "merged_planes", string.format("%.3d", scan_num))

	cmap = input_data.colormap:mul(255.0):int()
	plane_masks = input_data.plane_masks
		transform = in_arc_io:loadTorch("transformations", string.format("sweep%.3d", scan_num))

		-- Transform points map 
		pts = torch.reshape(points_map, 3,points_map:size(2)*points_map:size(3))
		pts = torch.cat( pts, torch.Tensor(1,points_map:size(2)*points_map:size(3)):fill(1), 1)	
		pts = transform*pts
		points_map = torch.reshape(pts:sub(1,3), 3, points_map:size(2), points_map:size(3))	

	-- For each plane mask convert to x,y,z,r,g,b pointcloud and write to .pts file 
	num_planes = plane_masks:size(1)
	print("Number of planes: ", num_planes)
	local output_fname = out_arc_io:workStr("ground_plane", string.format("%.3d.pts",scan_num))										
	print("Writing segmented region: ", output_fname)
	local output_fh = io.open( output_fname, "w" )

	--torch.range(1,num_planes):apply( function( plane_idx ) 
		local mask = plane_masks[{{ind}, {}, {}}]		
		local points_x = points_map:select(1,1)[mask]
		local points_y = points_map:select(1,2)[mask]
		local points_z = points_map:select(1,3)[mask]
		-- Output point data for visualization 
		torch.range(1,points_x:nElement()):apply( function( idx )
			local rgb = cmap[ind]
			output_fh:write( points_x[idx] ..' '.. points_y[idx] ..' '.. points_z[idx] ..' '.. 
							 rgb[1] ..' '..   rgb[2] ..' '.. rgb[3] .. '\n')	 
		end
		)
	--end
	--)
	output_fh:close()
end

function extract_samples( walls )
	-- Create all possible samples from walls
	samples = {}
	angles = {}
	inds = {}
	for i = 1,#walls do 
		for j = 1,#walls do 
			-- Check that combination is not co-planar
			angle = angle_between( walls[i].eqn:sub(1,3), walls[j].eqn:sub(1,3) ) 
			if angle > math.pi/4 and math.pi-angle > math.pi/4 then 
				table.insert( samples, {walls[i], walls[j]} )
				table.insert( inds, {i,j} )
				table.insert( angles, angle )
				print(string.format("angle[%d,%d]: %f", i,j,angle))
			end
		end
	end
	print("#samples0: ", #samples )
	return samples, angles, inds 
end

--[[
ground0, ground0_i = extract_ground( planes0 )
ground1, ground1_i = extract_ground( planes1 )
]]--

--[[
walls0, wall_inds0 = extract_walls( planes0 )
walls1, wall_inds1 = extract_walls( planes1 )

print("#walls0: ", #walls0)
print("#walls1: ", #walls1)

samples0, angles0, inds0 = extract_samples( walls0 )
samples1, angles1, inds1 = extract_samples( walls1 )
]]--

scan_ids = {1,2,3,4,6,7,8,9,10,11,12,13,14,15,16,17,18,19,21,22,23,24,25,26,27}

for i = 1,#scan_ids do 
	scan_data0 = in_arc_io:loadTorch("refitted_planes", string.format("scan%.3d", scan_ids[i]))
	planes0 = scan_data0.planes
	ground0, ground0_i = extract_ground( planes0 )
	output_plane( scan_ids[i], ground0_i )

	--walls0, wall_inds0 = extract_walls( planes0 )	
	--output_walls( scan_ids[i], wall_inds0 )
end

--[[
for i = 1,#walls0 do 
	print(walls0[i].eqn)
end
]]--

-- Compute differences
diffs = torch.Tensor(#angles0*#angles1)
inds = torch.LongTensor(2,#angles0*#angles1)
cnt = 1
for i = 1,#angles0 do 
	for j = 1,#angles1 do 		
		diffs[cnt] = math.abs(angles0[i] - angles1[j])
		inds[{{1},{cnt}}] = i
		inds[{{2},{cnt}}] = j
		cnt = cnt + 1
	end
end
sorted_diffs, sorted_inds = torch.sort(diffs)

normal_thresh = math.pi/8
residual_thresh = 15

sample0 = {nil, nil, nil}
sample1 = {nil, nil, nil}


best_tf = nil 
best_inliers = nil
-- Validate hypotheses 
for k = 1,sorted_inds:nElement() do	

	i = inds[{{1},{sorted_inds[k]}}]:squeeze()
	j = inds[{{2},{sorted_inds[k]}}]:squeeze()
	
	sample0 = samples0[i]	
	sample1 = samples1[j]

	sample0[1] = samples0[i][1]
	sample0[2] = samples0[i][2]
	sample0[3] = ground0

	sample1[1] = samples1[j][1]
	sample1[2] = samples1[j][2]
	sample1[3] = ground1

	est_tf = estimate_transform_from_planes( sample1, sample0 )
	inliers = validate_tf( est_tf, walls0, walls1, normal_thresh, residual_thresh )
	if best_tf == nil then
		best_tf = est_tf
		best_inliers = inliers
	end
	if inliers > best_inliers then 
		best_tf = est_tf
		best_inliers = inliers

	end
	print(string.format("inliers[%d,%d]: %d", i,j,inliers))
	collectgarbage()
end

print("best_inliers: ", best_inliers)

print(sorted_diffs:nElement())

--[[
for i = 1,sorted_diffs:nElement() do 
	print(string.format("diff: %f", sorted_diffs[i]))
end
]]--


--[[
output_walls( scan0, wall_inds0 )
output_walls( scan1, wall_inds1 )
]]-- 


--output_plane( scan0, ground0_i )
--output_plane( scan1, ground1_i )



--[[
n_samples = 300
angle_thresh = math.pi/4

-- Generate samples
samples0 = generate_samples( normals0, centroids0, ds0, angle_thresh, n_samples )
samples1 = generate_samples( normals1, centroids1, ds1, angle_thresh, n_samples )

sorted_vals, sorted_inds = match_samples( samples0, samples1 )

normal_thresh = math.pi/8
residual_thresh = 15

top_n = 2500


best_tf = nil 
best_inliers = nil

for k = 1,top_n do 	
	i = sorted_inds[{{1},{k}}]:squeeze()	
	j = sorted_inds[{{2},{k}}]:squeeze()


	est_tf = estimate_transform_from_planes( samples1[i], samples0[j] )
	inliers = validate_tf( est_tf, planes0, planes1, normal_thresh, residual_thresh )

	print(string.format("error[%d,%d]: %f", i,j, sorted_vals[k]))
	print("inliers: ", inliers)	

	if best_tf == nil then
		best_tf = est_tf
		best_inliers = inliers
	end
	if inliers > best_inliers then 
		best_tf = est_tf
		best_inliers = inliers

	end

	collectgarbage()
	
end

print("best_inliers: ", best_inliers)
print("best_tf: ", best_tf)
]]--
--[[
best_tf = nil 
best_inliers = nil 
for i = 1,n_samples do 
	for j = 1,n_samples do 
		est_tf = estimate_transform_from_planes( samples1[i], samples0[j] )
		inliers = validate_tf( est_tf, planes0, planes1, normal_thresh, residual_thresh )

		if best_tf == nil then
			best_tf = est_tf
			best_inliers = inliers
		end
		if inliers > best_inliers then 
			best_tf = est_tf
			best_inliers = inliers
		end
		print(string.format("inliers[%d,%d]: %d", i,j,inliers))
	end
	collectgarbage()
end
print("best_inliers: ", best_inliers)
print("best_tf: ", best_tf)
]]--



--[[
inds0 = torch.LongTensor({1,11,8})
inds1 = torch.LongTensor({1,21,14})

is_valid0, samples0 = extract_sample_set( inds0, normals0, centroids0, ds0, angle_thresh )
is_valid1, samples1 = extract_sample_set( inds1, normals1, centroids1, ds1, angle_thresh )
]]--
--[[
print("is_valid0: ", is_valid0)
print("is_valid1: ", is_valid1)
]]--

--[[
est_tf = estimate_transform_from_planes( samples1, samples0 )
print("est_tf: ", est_tf)
]]--

-- Find transforms using generated samples
--est_tf = estimate_transform_from_planes( samples1[1], samples0[1] )
-- Identify inliers using an normal_threshold and residual_threshold
-- First transform all planes from one coordinate system into the other
--[[
tfd_planes = {}
for i = 1,#planes1 do 
	table.insert( tfd_planes, transform_plane(planes0[i], est_tf) )	
end



-- Next count the number of planes that are within the angle/residual dist
match_matrix = torch.Tensor(#planes0,#tfd_planes):zero()
for i=1,#planes0 do 
	for j = 1,#tfd_planes do 
		angle = angle_between( planes0[i].eqn:sub(1,3) , tfd_planes[j].eqn:sub(1,3) )		
		residual = math.abs(torch.dot( planes0[i].eqn:sub(1,3), torch.add(tfd_planes[j].centroid, -planes0[i].centroid) ))
		--print(string.format("angle between [%d,%d]: %f", i,j,angle))
		if angle < normal_thresh and residual < residual_thresh then
			match_matrix[{i,j}] = 1
		end
	end
end

inlier_count = match_matrix:sum()
print("inlier_count: ", inlier_count)

image.display(match_matrix)
]]--


-- Output transformed plane points ... raw copy paste 

scan_num = scan0
pc = in_arc_io:getScan( scan_num )
points_map = pc:get_xyz_map()

-- Load in merged planes 
input_data = in_arc_io:loadTorch( "merged_planes", string.format("%.3d", scan_num))

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
local output_fname = out_arc_io:workStr("segmented_points_cull", string.format("%.3d.pts",scan_num))										
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
pc = in_arc_io:getScan( scan_num )
points_map = pc:get_xyz_map()

-- Load in merged planes 
input_data = in_arc_io:loadTorch( "merged_planes", string.format("%.3d", scan_num))

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
local output_fname = out_arc_io:workStr("segmented_points_cull", string.format("%.3d.pts",scan_num))										
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

--[[
output = {}
output.transform = transform
out_arc_io:dumpTorch( output, "transformations", string.format("%.3d", 17))
]]--




			
						



---while true do 

	
	-- Select 3 planes from each plane set 
	--[[
	inds0 = torch.rand(3):mul(#planes0):long()+1
	inds1 = torch.rand(3):mul(#planes1):long()+1

	print("inds0: ", inds0)
	print("inds1: ", inds1)

	sample_normals0 = normals0:index(2,inds0)
	sample_normals1 = normals1:index(2,inds1)

	print(validate_samples( sample_normals0, angle_thresh ))
	print(validate_samples( sample_normals1, angle_thresh ))

	sample_centroids0 = centroids0:index(2,inds0)
	sample_centroids1 = centroids1:index(2, inds1)

	print("sample_normals0: ", sample_normals0)
	print("sample_normals1: ", sample_normals1)
	print("sample_centroids0: ", sample_centroids0)
	print("sample_centroids1: ", sample_centroids1)
	]]--






--[[
normals = torch.Tensor(3,#planes)
for i = 1,#planes do 
	normals[{{},{i}}] = planes[i].eqn:sub(1,3)
end

tfd_normals = torch.Tensor(3,#tfd_planes)
for i = 1,#tfd_planes do 
	tfd_normals[{{},{i}}] = tfd_planes[i].eqn:sub(1,3)
end

print("normals: ", normals)
print("tfd_normals: ", tfd_normals)

-- Compute rotation
U,S,V = torch.svd(tfd_normals*normals:t())

R = U*V:t()
print("R: ", R)
print("det(R): ", det(R))
-- Check for reflection
if det3x3(R):squeeze() < 0 then 
	R[{{},{3}}] = R[{{},{3}}]:mul(-1)
end
print("R: ", R)
print("det(R): ", det(R))

print("S: ", S)

print("rotation - rotation_est: ", rotation - R)

-- Compute translation 
d_errs = torch.Tensor(#planes)
for i = 1,#planes do 
	d_errs[{i}] = tfd_planes[i].eqn[{4}] - planes[i].eqn[{4}]
end

print("d_errs: ", d_errs)

t_est = torch.inverse(tfd_normals:t())*d_errs
print()
print("t_est: ", t_est)
print("translation: ", translation)
print("translation - t_est: ", t_est - translation )
]]--









