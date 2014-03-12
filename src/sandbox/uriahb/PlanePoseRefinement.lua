--[[	
	Given to pairs of planes with a strong prior transformation ( provided my the 2d matcher )
	acquire a refined transform using plane constraints 
]]--
io = require('io')

ArcSpecs = data.ArcSpecs
ArcIO = data.ArcIO

estimate_transform_from_planes = Plane.util.estimate_transform_from_planes
transform_plane = Plane.util.transform_plane
angle_between = geom.util.angle_between

local TransformFromPlanes = Class()

function TransformFromPlanes:__init( job_id, pair_id )
	self.job_id = job_id
	self.pair_id = pair_id

	-- Convert from linear index to actual scan index
	scan_ids = ArcSpecs.scan_ids[job_id]

	-- Init ArcIO nodes
	self.planes_io = ArcIO.new( job_id, 'merged_planes' )
	self.tf_io = ArcIO.new( job_id, "tf_from_planes" )

	tf2d = self.tf_io:loadTorch("transformations2d", "transformations2d")	
	self.tf_prior = tf2d[self.pair_id].rough_transformation

	self.root_id = scan_ids[tf2d[self.pair_id].sweep1]
	self.child_id = scan_ids[tf2d[self.pair_id].sweep2]

	print("parent_id: ", self.root_id)
	print("child_id: ", self.child_id)

	
	self.root_planes = self.planes_io:loadTorch("merged_planes", string.format("scan%.3d", self.root_id)).planes		
	self.child_planes = self.planes_io:loadTorch("merged_planes", string.format("scan%.3d", self.child_id)).planes	
end

function TransformFromPlanes:matchPlanes( normal_thresh, residual_thresh )
	tfd_planes = {}
	for i = 1,#self.child_planes do 	
		table.insert( tfd_planes, transform_plane(self.child_planes[i], self.tf_prior) )	
	end

	matches = {}
	match_adjacency = {}
	for i=1,#self.root_planes do		
		for j = 1,#tfd_planes do 
			local angle = angle_between( self.root_planes[i].eqn:sub(1,3) , tfd_planes[j].eqn:sub(1,3) )		
			local residual = math.abs(torch.dot( self.root_planes[i].eqn:sub(1,3), torch.add(tfd_planes[j].centroid, -self.root_planes[i].centroid) ))			
			if angle < normal_thresh and residual < residual_thresh then
				table.insert( matches, {i,j} )
				-- Fill in adjacency representation
				if match_adjacency[i] == nil then
					match_adjacency[i] = {}
				end
				table.insert(match_adjacency[i], j)

				print(string.format("[%d,%d]: %f, %f", i,j,angle, residual))				
			end
		end		
	end
	self.matches_cmap = image.colormap(#matches):mul(255.0):int()
	return matches, match_adjacency
end

function TransformFromPlanes:overlapCull( match_adj ) 
	-- Compute matches
	child_reprojection_map, child_bounds_msk = self:reprojectionMap( self.planes_io, self.root_id, self.child_id, self.tf_prior )	
	child_plane_data = self.planes_io:loadTorch( "merged_planes", string.format("scan%.3d", self.child_id))
	child_planes = child_plane_data.planes

	parent_reprojection_map, parent_bounds_msk = self:reprojectionMap( self.planes_io, self.child_id, self.root_id, torch.inverse(self.tf_prior) )
	parent_plane_data = self.planes_io:loadTorch( "merged_planes", string.format("scan%.3d", self.root_id))
	parent_planes = parent_plane_data.planes

	p_msk_height = parent_planes[1].mask:size(1)
	p_msk_width = parent_planes[1].mask:size(2)
	c_msk_height = child_planes[1].mask:size(1)
	c_msk_width = child_planes[2].mask:size(2)	
	
	verified_match_adjacency = {}	
	for k,m in pairs(match_adjacency) do						
		for _,v in ipairs(m) do 			
			child_proj_mask_linear = torch.Tensor(1,p_msk_width*p_msk_height):zero()			
			child_mask_linear = child_planes[v].mask:reshape(1,c_msk_width*c_msk_height)[child_bounds_msk]
			child_mask_linear[child_mask_linear:gt(1)] = 1						
			if child_mask_linear:sum() > 0 then 
				child_proj_mask_linear:indexFill(2, child_reprojection_map[child_mask_linear], 1)
				child_proj_mask = child_proj_mask_linear:reshape(p_msk_height, p_msk_width)
	
				-- Compute overlap between parent mask and child mask projected into parent frame 				
				overlap = (parent_planes[k].mask+child_proj_mask:byte()):eq(2):sum()
				if overlap > 0 then 					
					-- Fill in adjacency representation
					if verified_match_adjacency[k] == nil then
						verified_match_adjacency[k] = {}
					end
					table.insert(verified_match_adjacency[k], v)
				end	
			end			
		end
		collectgarbage()		
	end
	--self:printMatchAdjacency( verified_match_adjacency )
	--self:outputAdjacencyPlanes( verified_match_adjacency )
	return verified_match_adjacency
end

function TransformFromPlanes:adjacencyToPairs( match_adjacency )
	-- Reduce adjacency lists to possible pairs 
	print("Converting from adjacency to pairs")
	match_pairs = {}
	for k,m in pairs(match_adjacency) do		
		for _,v in ipairs(m) do 
			table.insert(match_pairs, {k,v})			
		end		
	end
	print("Number of pairs: ", #match_pairs)
	for i = 1,#match_pairs do 
		print( string.format("%d: %d", match_pairs[i][1], match_pairs[i][2]) )
	end
	return match_pairs
end

function TransformFromPlanes:printMatchAdjacency( match_adjacency )	
	for k,m in pairs(match_adjacency) do
		str = string.format("%d: ", k)
		for _,v in ipairs(m) do 
			str = str .. string.format("%d ", v)
		end
		print(str)
	end
end

function TransformFromPlanes:outputPlanePoints( dir_name, fsuffix, scan_num, plane_inds, transform, cmap)
	pc = self.planes_io:getScan( scan_num )
	points_map = pc:get_xyz_map()
	--points_map = points_map:sub(1,3,50,points_map:size(2), 1,points_map:size(3)):clone()

	-- Load in merged planes 
	input_data = self.planes_io:loadTorch( "merged_planes", string.format("scan%.3d", scan_num))
	planes = input_data.planes

	points_map = self:transformPointsMap( transform, points_map )

	num_planes = #planes
	print("Number of planes: ", num_planes)
	local output_fname = self.tf_io:workStr(dir_name, string.format("%.3d%s.pts",self.root_id,fsuffix))										
	print("Writing segmented region: ", output_fname)
	local output_fh = io.open( output_fname, "w" )

	-- For each plane mask convert to x,y,z,r,g,b pointcloud and write to .pts file 
	for i = 1,#plane_inds do 
		plane_idx = plane_inds[i]	
		local mask = planes[plane_idx].mask
		self:outputPlane( output_fh, points_map, mask, cmap[i])		
	end	
	output_fh:close()
end

function TransformFromPlanes:outputAdjacencyPlanes( match_adjacency )
	print("Writing Adjacency Planes")
	-- output match adjacency planes with same color 
	local root_xyz_map = self.planes_io:getScan( self.root_id ):get_xyz_map()
	local child_xyz_map = self.planes_io:getScan( self.child_id ):get_xyz_map()
	local child_xyz_map = self:transformPointsMap( self.tf_prior, child_xyz_map )

	local parent_planes = self.planes_io:loadTorch( "merged_planes", string.format("scan%.3d", self.root_id)).planes
	local child_planes = self.planes_io:loadTorch( "merged_planes", string.format("scan%.3d", self.child_id)).planes

	for k,m in pairs(match_adjacency) do
		local output_fname = self.tf_io:workStr(string.format('match_hypotheses_%.3d', self.root_id), string.format("%.3d.pts",k))										
		local output_fh = io.open( output_fname, "w" )	
		local parent_mask = parent_planes[k].mask
		self:outputPlane( output_fh, root_xyz_map, parent_mask, {255,0,0})
		for _,v in ipairs(m) do 			
			local child_mask = child_planes[v].mask
			self:outputPlane( output_fh, child_xyz_map, child_mask, {0,0,255})
		end
		output_fh:close()
	end
end

function TransformFromPlanes:transformPointsMap( transform, points_map )
	-- Transform points map 
	pts = torch.reshape(points_map, 3,points_map:size(2)*points_map:size(3))
	pts = torch.cat( pts, torch.Tensor(1,points_map:size(2)*points_map:size(3)):fill(1), 1)	
	pts = transform*pts
	points_map = torch.reshape(pts:sub(1,3), 3, points_map:size(2), points_map:size(3))	
	return points_map
end

function TransformFromPlanes:outputPlane( output_fh, points_map, mask, rgb )		
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

function TransformFromPlanes:reprojectionMap( planes_io, parent_id, child_id, transform )
	local parent_pc = planes_io:getScan( parent_id )
	local _,parent_phi,parent_theta = parent_pc:get_xyz_map()
	local parent_height = parent_phi:size(1)
	local parent_width = parent_phi:size(2)
	local phi_scl = parent_height/(parent_phi:max()-parent_phi:min())
	local theta_scl = parent_width/(parent_theta:max()-parent_theta:min())
	local phi_offs = math.abs(parent_phi:max())
	local theta_offs = math.abs(parent_theta:max())

	local child_pc = planes_io:getScan( child_id )
	--child_pc = tfplanes.planes_io:getScan( tfplanes.root_id )
	local child_xyz = child_pc:get_xyz_map()
	local child_height = child_xyz:size(1)
	local child_width = child_xyz:size(2)

	-- Transform child points to parent scan coordinates
	local pts = torch.reshape(child_xyz,3,child_xyz:size(2)*child_xyz:size(3))
	pts = torch.cat( pts, torch.Tensor(1,child_xyz:size(2)*child_xyz:size(3)):fill(1), 1)	
	pts = transform*pts

	-- Get spherical coordinates of points
	-- using torch.pow to ensure copy occurs. Im surprised that we are using (x,y) not (x,y,z)
	local radius = torch.pow(pts:sub(1,2),2):sum(1):sqrt()
	local phi = -torch.atan2( pts:select(1,3), radius )
	local theta = -torch.atan2( pts:select(1,2), pts:select(1,1) )

	-- Rescale/round into uv space
	local theta_scaled = ((theta+theta_offs)*theta_scl):add(0.5):int() 
	local phi_scaled = ((phi+phi_offs)*phi_scl):add(0.5):int() 

	-- Remove out-of-bounds values 
	local bounds_msk = (theta_scaled:gt(parent_width-1) + theta_scaled:lt(1) + phi_scaled:gt(parent_height-1) + phi_scaled:lt(1)):lt(1)
	-- Calculate linear reprojection map
	reprojection_map = (phi_scaled[bounds_msk]*parent_width + theta_scaled[bounds_msk]):long()

	return reprojection_map, bounds_msk
end

function TransformFromPlanes:calcThetaMap( planes_io, sub_id, base_id, search_bound )
	-- Since the xyz_maps are different sizes between scans in the theta direction
	-- calculate the best mapping in the theta direction
	sbound = search_bound or 10 

	sub_pc = planes_io:getScan( sub_id )
	base_pc = planes_io:getScan( base_id )

	_,_,sub_theta_map = sub_pc:get_xyz_map()
	_,_,base_theta_map = base_pc:get_xyz_map()

	print(sub_theta_map[{{1},{}}])
	print(base_theta_map[{{1},{}}])

	sub_thetas = sub_theta_map[{{1},{}}]:squeeze()
	base_thetas = base_theta_map[{{1},{}}]:squeeze()

	-- The search bound should at least be bigger than the difference between 
	-- 	 the theta sizes 	
	print("search_bound: ", sbound)

	ind_map = torch.LongTensor(sub_thetas:nElement())

	j = 1
	for i = 1,sub_thetas:nElement() do 
		-- Search for closest element from t2 to t1 around j 
		local min_d = nil
		local min_j = nil
		for k = j-sbound,j+sbound do 
			if k > 1 and k < base_thetas:nElement() then
				cur_d = torch.pow(sub_thetas[i] - base_thetas[k],2) 
				if not min_d or cur_d < min_d then
					min_d = cur_d
					min_j = k
				end
			end
		end
		-- Set j to closest element 
		j = min_j
		ind_map[i] = j 
		print(string.format("[%d,%d]:[%f, %f]", i, j, sub_thetas[i], base_thetas[j]))
	end

	-- Use :index function to create indices lists mapping from bigger to smaller, and sub to map from 
	-- smaller to bigger 
	base_map = torch.reshape( torch.range(1,base_theta_map:size(1)*base_theta_map:size(2)), base_theta_map:size(1), base_theta_map:size(2) ) 		
	sub_map = base_map:index(2, ind_map)
	sub_map = sub_map:reshape( sub_theta_map:size(1)*sub_theta_map:size(2) )
	print(base_map)
	print(sub_map)
	return sub_map:long()
end

function TransformFromPlanes:extractGoodSamples( match_pairs, good_angle_thresh )
	sample0 = {nil, nil, nil}
	sample1 = {nil, nil, nil}
	local inds
	local i1
	local i2
	local i3
	while true do 
		inds = torch.rand(3):mul(#match_pairs):long()+1		
		i1 = match_pairs[ inds[1] ][1]
		i2 = match_pairs[ inds[2] ][1]
		i3 = match_pairs[ inds[3] ][1]		
		local normals1 = self.root_planes[i1].eqn:sub(1,3)
		local normals2 = self.root_planes[i2].eqn:sub(1,3)
		local normals3 = self.root_planes[i3].eqn:sub(1,3)
		local angle12 = angle_between( normals1, normals2 )
		local angle13 = angle_between( normals1, normals3 )
		local angle23 = angle_between( normals2, normals3 )
		if angle12 > good_angle_thresh and math.pi-angle12 > good_angle_thresh and
		   angle13 > good_angle_thresh and math.pi-angle13 > good_angle_thresh and	
		   angle23 > good_angle_thresh and math.pi-angle23 > good_angle_thresh then
		   break
		end
	end

	local j1 = match_pairs[ inds[1] ][2]
	local j2 = match_pairs[ inds[2] ][2]
	local j3 = match_pairs[ inds[3] ][2]		
	sample0[1] = self.root_planes[i1]
	sample0[2] = self.root_planes[i2]
	sample0[3] = self.root_planes[i3]

	sample1[1] = self.child_planes[j1]
	sample1[2] = self.child_planes[j2]
	sample1[3] = self.child_planes[j3]
	return sample0, sample1
end

function TransformFromPlanes:validateTransform( est_tf, match_pairs, normals_thresh, residual_thresh )
	local tfd_planes = {}
	local inlier_cnt = 0
	local inliers = {}
	for i = 1,#match_pairs do 			
		local plane = self.root_planes[match_pairs[i][1] ] 
		local tfd_plane = transform_plane(self.child_planes[match_pairs[i][2] ], est_tf)
		local angle = angle_between( plane.eqn:sub(1,3) , tfd_plane.eqn:sub(1,3) )		
		local residual = math.abs(torch.dot( plane.eqn:sub(1,3), torch.add(tfd_plane.centroid, -plane.centroid) ))
		--print(string.format("angle between [%d,%d]: %f", i,j,angle))
		if angle < normal_thresh and residual < residual_thresh then
			inlier_cnt = inlier_cnt + 1
			table.insert( inliers, i )
		end		 
	end	
	return inlier_cnt, inliers
end

function TransformFromPlanes:ransacExtractTransform( match_pairs ) 
	-- Extract transform using match pairs of planes 
	angle_thresh = math.pi/8

	residual_thresh = 10
	normal_thresh = math.pi/16

	best_tf = nil 
	best_inlier_cnt = nil
	best_inliers = nil 
	best_inds = nil

	n_samples = 1000
	for i = 1,n_samples do 
		-- Extract three good samples
		sample0, sample1 = self:extractGoodSamples( match_pairs, angle_thresh )

		-- Estimate transform from sample sets
		est_tf = estimate_transform_from_planes( sample0, sample1 )
		
		-- Validate transform
		inlier_cnt, inliers = self:validateTransform( est_tf, match_pairs, normal_thresh, residual_thresh )

		if best_tf == nil or inlier_cnt > best_inlier_cnt then
			best_tf = est_tf
			best_inlier_cnt = inlier_cnt
			best_inliers = inliers
			best_inds = inds
		end

		print("inlier_cnt: ", inlier_cnt )
	end
	return best_tf, best_inlier_cnt, best_inliers
end

function colorReprojectedPlanes( child_planes, parent_planes, cmap, reprojection_map, bounds_msk )
	-- Given masks and colormap draw planes
	msk_height = parent_planes[1].mask:size(1)
	msk_width = parent_planes[1].mask:size(2)
	rgb_map = torch.Tensor(3, msk_height, msk_width):zero()
	r_map_linear = rgb_map[{{1},{},{}}]:reshape(1,msk_height*msk_width)
	g_map_linear = rgb_map[{{2},{},{}}]:reshape(1,msk_height*msk_width)
	b_map_linear = rgb_map[{{3},{},{}}]:reshape(1,msk_height*msk_width)

	print(msk_height*msk_width)
	print(theta_map)
	print(reprojection_map)
	print(bounds_msk)

	cmap = image.colormap(#child_planes)
	--for msk_idx = 1,#planes do 	
	for k,child_plane in pairs(child_planes) do 
		-- This is where it gets tricky, need to change everything into linear array		
		mask_linear = child_plane.mask:reshape(1,child_plane.mask:size(1)*child_plane.mask:size(2))[bounds_msk]							
		if mask_linear:sum() > 0 then 
			r_map_linear:indexFill(2, reprojection_map[mask_linear], cmap[{{k},{1}}]:squeeze())
			g_map_linear:indexFill(2, reprojection_map[mask_linear], cmap[{{k},{2}}]:squeeze())
			b_map_linear:indexFill(2, reprojection_map[mask_linear], cmap[{{k},{3}}]:squeeze())
		end
	end	
	rgb_map[{{1},{},{}}] = r_map_linear:reshape(1,msk_height,msk_width)
	rgb_map[{{2},{},{}}] = g_map_linear:reshape(1,msk_height,msk_width)
	rgb_map[{{3},{},{}}] = b_map_linear:reshape(1,msk_height,msk_width)
	return rgb_map
end

function colorPlanes( masks, cmap )
	-- Given masks and colormap draw planes
	msk_height = masks[1]:size(1)
	msk_width = masks[2]:size(2)
	rgb_map = torch.Tensor(3, msk_height, msk_width)
	for msk_idx = 1,masks:size(1) do 
		for i = 1,3 do	
			rgb_map[{{i},{},{}}][masks[msk_idx]] = cmap[{{msk_idx},{i}}]:squeeze()
		end
	end
	return rgb_map
end


--[[
	Transform tests ... prototype and test code here, merge into appropriate modules afterwards
]]-- 

-- Tests 
function test_matching( tfplanes )
	normal_thresh = math.pi/4
	residual_thresh = 100
	_, match_adj = tfplanes:matchPlanes( normal_thresh, residual_thresh )
	tfplanes:printMatchAdjacency( match_adj )
	tfplanes:outputAdjacencyPlanes()
end

function test_colorization( tfplanes )
	root_plane_data = tfplanes.planes_io:loadTorch( "merged_planes", string.format("%.3d", tfplanes.root_id))	
	root_masks = root_plane_data.plane_masks
	root_cmap = root_plane_data.colormap
	root_rgb_map = colorPlanes( root_masks, root_cmap )
	image.display(root_rgb_map)
end

function test_reprojection( tfplanes )
	-- Reproject planes by transforming points, converting to spherical coords, and bining into uv coords
	-- 	for debugging purposes will use the uv coordinate to map colors 	

	child_reprojection_map, child_bounds_msk = tfplanes:reprojectionMap( tfplanes.planes_io, tfplanes.root_id, tfplanes.child_id, tfplanes.tf_prior )
	theta_map = tfplanes:calcThetaMap( tfplanes.planes_io, tfplanes.root_id, tfplanes.child_id )
	child_plane_data = tfplanes.planes_io:loadTorch( "merged_planes", string.format("scan%.3d", tfplanes.child_id))
	child_planes = child_plane_data.planes
	child_cmap = child_plane_data.colormap
	

	parent_reprojection_map, parent_bounds_msk = tfplanes:reprojectionMap( tfplanes.planes_io, tfplanes.child_id, tfplanes.root_id, torch.inverse(tfplanes.tf_prior) )
	theta_map = tfplanes:calcThetaMap( tfplanes.planes_io, tfplanes.child_id, tfplanes.root_id )
	parent_plane_data = tfplanes.planes_io:loadTorch( "merged_planes", string.format("scan%.3d", tfplanes.root_id))
	parent_planes = parent_plane_data.planes
	parent_cmap = parent_plane_data.colormap

	child_rgb_map = colorReprojectedPlanes( child_planes, parent_planes, child_cmap, child_reprojection_map, child_bounds_msk )
	parent_rgb_map = colorReprojectedPlanes( parent_planes, child_planes, parent_cmap, parent_reprojection_map, parent_bounds_msk )

	print(child_plane_data.planes[1].mask)
	print(child_rgb_map)
	print(parent_plane_data.planes[1].mask)
	print(parent_rgb_map)

	-- Display
	image.display(child_rgb_map)
	image.display(parent_rgb_map)

	tfplanes.tf_io:dumpImage( child_rgb_map, 'reprojected_planes', string.format('parent%.3d', tfplanes.root_id))						
	tfplanes.tf_io:dumpImage( parent_rgb_map, 'reprojected_planes', string.format('child%.3d', tfplanes.root_id))						

end

function test_reprojection_match( tfplanes )
	-- Compute matches
	normal_thresh = math.pi/4
	residual_thresh = 200
	_, match_adj = tfplanes:matchPlanes( normal_thresh, residual_thresh )
	tfplanes:printMatchAdjacency( match_adj )

	child_reprojection_map, child_bounds_msk = tfplanes:reprojectionMap( tfplanes.planes_io, tfplanes.root_id, tfplanes.child_id, tfplanes.tf_prior )
	child_theta_map = tfplanes:calcThetaMap( tfplanes.planes_io, tfplanes.root_id, tfplanes.child_id )
	child_plane_data = tfplanes.planes_io:loadTorch( "merged_planes", string.format("scan%.3d", tfplanes.child_id))
	child_masks = child_plane_data.plane_masks

	parent_reprojection_map, parent_bounds_msk = tfplanes:reprojectionMap( tfplanes.planes_io, tfplanes.child_id, tfplanes.root_id, torch.inverse(tfplanes.tf_prior) )
	parent_theta_map = tfplanes:calcThetaMap( tfplanes.planes_io, tfplanes.child_id, tfplanes.root_id )
	parent_plane_data = tfplanes.planes_io:loadTorch( "merged_planes", string.format("%.3d", tfplanes.root_id))
	parent_masks = parent_plane_data.plane_masks

	p_msk_height = parent_masks[1]:size(1)
	p_msk_width = parent_masks[2]:size(2)
	c_msk_height = child_masks[1]:size(1)
	c_msk_width = child_masks[2]:size(2)

	-- Given match_adj, reproject appropriate masks in both views to see if there is any overlap
	-- throw out non-overlapping matches

	--[[
	parent_mask_linear = parent_masks[k]:reshape(1,p_msk_width*p_msk_height)[parent_bounds_msk]							
	image.display( parent_masks[k] )
	child_mask_linear = child_masks[v]:reshape(1,c_msk_width*c_msk_height)[child_bounds_msk]
	proj_child_mask_linear = child_reprojection_map[child_mask_linear]							
	image.display(child_masks[v])
	image.display(proj_child_mask_linear:reshape(c_msk_height, c_msk_width))
	]]--

	-- tf_io for outputing random planes 
	tf_io = tfplanes.tf_io
	
	verified_match_adjacency = {}	
	for k,m in pairs(match_adjacency) do				
		--parent_mask_linear = parent_masks[k]:reshape(1,p_msk_width*p_msk_height)[parent_bounds_msk]	
		--tf_io:dumpImage( parent_masks[k]:mul(255), 'overlap_masks', string.format('parent%.3d', k))						
		--image.display( parent_masks[k] )
		for _,v in ipairs(m) do 			
			child_proj_mask_linear = torch.Tensor(1,c_msk_width*c_msk_height):zero()
			child_mask_linear = child_masks[v]:reshape(1,c_msk_width*c_msk_height)[child_bounds_msk]
			child_mask_linear[child_mask_linear:gt(1)] = 1						
			if child_mask_linear:sum() > 0 then 
				child_proj_mask_linear:indexFill(2, child_theta_map[child_reprojection_map[child_mask_linear]], 1)
				child_proj_mask = child_proj_mask_linear:reshape(c_msk_height, c_msk_width)

				--tf_io:dumpImage( child_masks[v]:mul(255), 'overlap_masks', string.format('child%.3d_%.3d', k, v))
				--tf_io:dumpImage( proj_mask:mul(255), 'overlap_masks', string.format('proj%.3d_%.3d', k, v))				

				-- Compute overlap between parent mask and child mask projected into parent frame 
				overlap = (parent_masks[k]+child_proj_mask:byte()):eq(2):sum()
				if overlap > 0 then 					
					-- Fill in adjacency representation
					if verified_match_adjacency[k] == nil then
						verified_match_adjacency[k] = {}
					end
					table.insert(verified_match_adjacency[k], v)
				end
				
			end
			--tf_io:dumpImage( proj_mask + parent_masks[k]:double(), 'overlap_masks', string.format('overlap%.3d_%.3d', k, v)) weird behavior, adding doesn't change grayscale color
			--image.display(child_masks[v])
			--image.display(proj_mask:reshape(c_msk_height, c_msk_width))
		end
		collectgarbage()		
	end
	tfplanes:printMatchAdjacency( verified_match_adjacency )
	tfplanes:outputAdjacencyPlanes( verified_match_adjacency )
	return verified_match_adjacency
end

-- Run Tests 
tf_output = {}
tfplanes = TransformFromPlanes.new('precise-transit-6548', 16)
test_reprojection( tfplanes )
--[[
for i = 1,25 do 
tfplanes = TransformFromPlanes.new('precise-transit-6548', i)
--test_reprojection( tfplanes )


normal_thresh = math.pi/4
residual_thresh = 200
_, match_adj = tfplanes:matchPlanes( normal_thresh, residual_thresh )
tfplanes:printMatchAdjacency( match_adj )

verified_match_adj = tfplanes:overlapCull( match_adj )
tfplanes:printMatchAdjacency( verified_match_adj )
tfplanes:outputAdjacencyPlanes( verified_match_adj )


match_pairs = tfplanes:adjacencyToPairs( verified_match_adj )

est_tf, inlier_cnt = tfplanes:ransacExtractTransform( match_pairs )

print("tf_prior: ", tfplanes.tf_prior)
print("tf_est: ", est_tf)
print("inlier_cnt: ", inlier_cnt)


parent_plane_inds = {}
child_plane_inds = {}
for i = 1,#match_pairs do 
	table.insert( parent_plane_inds, match_pairs[i][1])
	table.insert( child_plane_inds, match_pairs[i][2])
end

tfplanes:outputPlanePoints( 'tfd_planes', 'parent', tfplanes.root_id, parent_plane_inds, torch.eye(4), tfplanes.matches_cmap)
tfplanes:outputPlanePoints( 'tfd_planes', 'child', tfplanes.child_id, child_plane_inds, tfplanes.tf_prior, tfplanes.matches_cmap)
tfplanes:outputPlanePoints( 'tfd_planes', 'tfchild', tfplanes.child_id, child_plane_inds, est_tf, tfplanes.matches_cmap)

scan_output = {}

scan_output.job_id = tfplanes.job_id
scan_output.refined_transform = est_tf
scan_output.sweep1 = tfplanes.root_id
scan_output.sweep2 = tfplanes.child_id
table.insert( tf_output, scan_output )
end

tfplanes.tf_io:dumpTorch( tf_output, "transforms", "refined_transforms" )
]]--


--tfplanes:outputAdjacencyPlanes( verified_match_adj )


--test_reprojection_match( tfplanes )


