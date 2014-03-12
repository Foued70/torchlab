--[[
	Merge Planes over multiple scans
]]--
ArcIO = data.ArcIO
MergePlanes = CPlane.MergePlanes
fit_plane_to_points = Plane.util.fit_plane_to_points 
transform_plane = Plane.util.transform_plane

local MultiScanMergePlanes = Class( MergePlanes )

default_overlap_thresh = 50
default_input_io = 'single_scan_merge_planes'

-- This needs a good place to hang out
function reprojectionMap( planes_io, parent_id, child_id, transform )
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

function colorReprojectedPlanes( child_planes, parent_planes, reprojection_map, bounds_msk )
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

function MultiScanMergePlanes:__init( job_id, scan_ids, normal_thresh, residual_thresh, overlap_thresh )	
	self.overlap_thresh = overlap_thresh or default_overlap_thresh
	input_io = default_input_io

	-- Initialize MergePlanes
	__super__.__init(self, job_id, input_io, normal_thresh, residual_thresh ) 

	self.tf_io = ArcIO.new( job_id, 'global_transformations' )
	self.output_io = ArcIO.new( job_id, 'multi_scan_merge_planes')

	self.scan_ids = scan_ids
	self.scan_planes = {}
	self.scan_transforms = {}
	for i = 1,#scan_ids do 
		planes = self.input_io:loadTorch("merged_planes", string.format("scan%.3d", scan_ids[i])).planes
		tf = self.tf_io:loadTorch("transformations", string.format("sweep%.3d", scan_ids[i]))
		table.insert( self.scan_planes, planes )
		table.insert( self.scan_transforms, tf )
	end
end

-- For each scan_id cat the transform and concatenate the planes
function MultiScanMergePlanes:preparePlanes()
	self.planes = {}
	for i = 1,#self.scan_planes do 
		planes = self.scan_planes[i]
		tf = self.scan_transforms[i]
		for j = 1,#planes do 
			plane = planes[j]			
			table.insert( self.planes, transform_plane(plane, tf) )
		end
	end
	print("n_planes: ", #self.planes)
end

-- TODO: use reprojection to maintain connectivity of planes
function MultiScanMergePlanes:overlapCull( match_list )
	match_matrix = torch.ByteTensor(#self.planes,#self.planes):zero()
	culled_match_list = {}

	child_id = scan_ids[1]
	parent_id = scan_ids[2]

	child_tf = self.scan_transforms[child_id]
	parent_tf = self.scan_transforms[parent_id]

	-- We want the relative transform between the parent and child
	ctf = torch.inverse(parent_tf)*child_tf
	ptf = torch.inverse(child_tf)*parent_tf
	--tf = parent_tf*torch.inverse(child_tf)

	-- Compute matches
	child_reprojection_map, child_bounds_msk = reprojectionMap( self.input_io, parent_id, child_id, ctf)		
	parent_reprojection_map, parent_bounds_msk = reprojectionMap( self.input_io, child_id, parent_id, ptf )

	-- Walk through the matches
	for k,m in pairs(self.match_list) do		
		for _,v in ipairs(m) do			
			-- If planes are from the same scan then we have already checked their overlap			

			if self.planes[k].scan_id ~= self.planes[v].scan_id then 
				-- Check overlaps for child and parent, this is gross but exists to verify the method.
				-- TODO: clean up, make general 
				
				if self.planes[k].scan_id == child_id then 
					c_msk_height = self.planes[k].mask:size(1)
					c_msk_width = self.planes[k].mask:size(2)
					p_msk_height = self.planes[v].mask:size(1)
					p_msk_width = self.planes[v].mask:size(2)
					
					child_proj_mask_linear = torch.Tensor(1,p_msk_width*p_msk_height):zero()			
					child_mask_linear = self.planes[k].mask:reshape(1,c_msk_width*c_msk_height)[child_bounds_msk]
					child_mask_linear[child_mask_linear:gt(1)] = 1						
					if child_mask_linear:sum() > 0 then 
						child_proj_mask_linear:indexFill(2, child_reprojection_map[child_mask_linear], 1)
						child_proj_mask = child_proj_mask_linear:reshape(p_msk_height, p_msk_width)

						-- Compute overlap between parent mask and child mask projected into parent frame 				
						overlap = (self.planes[v].mask+child_proj_mask:byte()):eq(2):sum()
						print(overlap)
						if overlap > self.overlap_thresh then
							match_matrix[{k,v}] = 1
						end				
					end						
				end		
				collectgarbage()			
				--match_matrix[{k,v}] = 1
			end
		end
	end
	--rgb_map = colorReprojectedPlanes( self.scan_planes[1], self.scan_planes[2], child_reprojection_map, child_bounds_msk )
	--image.display( rgb_map )
	return match_matrix, match_list
end

function MultiScanMergePlanes:fitMergedPlanes( plane_sets )
	-- Get plane masks and planes	
	local point_maps = {}
	local masks = {}
	for i = 1,#self.scan_ids do 
		local points_map = self.input_io:getScan( self.scan_ids[i] ):get_xyz_map()
		imgh = points_map:size(2)
		imgw = points_map:size(3)
		local mask = torch.ByteTensor(imgh, imgw):zero()
		table.insert( point_maps, points_map )
		masks[self.scan_ids[i]] = mask		
	end
		
	local fitted_planes = {}
	for k,v in pairs(plane_sets) do 
		-- Accumulate over all masks
		for i = 1,#self.scan_ids do 
			masks[i]:zero()
		end
		for j=1, v:nElement() do 			
			plane = self.planes[v[j]]			
			masks[plane.scan_id]:add( plane.mask )
		end
		local pts = nil		
		local plane_scan_ids = {}
		local plane_masks = {}		
		for i = 1,#self.scan_ids do 						
			local mask = masks[self.scan_ids[i]]:gt(0.5)
			if mask:sum() > 0 then 
				table.insert(plane_scan_ids, self.scan_ids[i])				
				table.insert(plane_masks, mask)
				local points_x = point_maps[i]:select(1,1)[mask]
				local points_y = point_maps[i]:select(1,2)[mask]
				local points_z = point_maps[i]:select(1,3)[mask]
				-- Fit plane to new planes 
				if not pts then 
					pts = torch.cat(torch.cat(points_x,points_y,2), points_z,2)
				else
					pts = torch.cat(pts, torch.cat(torch.cat(points_x,points_y,2),points_z,2), 1)
				end
			end
		end
		
		local plane_eqn, plane_centroid = fit_plane_to_points(pts:t())
		local plane_normal = plane_eqn:sub(1,3)

		plane = {}	
		plane.scan_ids = plane_scan_ids	
		plane.masks = plane_masks
		plane.eqn = plane_eqn
		plane.centroid = plane_centroid
		plane.color = self.cmap[{{k},{}}]		
		fitted_planes[k] = plane 		
	end
	return fitted_planes
end

-- Given planes with masks etc, output colored scan images
function MultiScanMergePlanes:colorScans( planes )
	cmap = image.colormap(#planes)
	rgb_maps = {}
	for i,plane in ipairs( planes ) do 
		scan_ids = plane.scan_ids
		for idx, scan_id in ipairs( scan_ids ) do 
			mask = plane.masks[idx]
			if not rgb_maps[scan_id] then
				rgb_maps[scan_id] = torch.Tensor(3,mask:size(1), mask:size(2)):zero()
			end
			rgb_map = rgb_maps[scan_id]
			--[[
			print(rgb_map:select(1,2))
			print(mask)
			print(rgb_map:select(1,1)[mask])
			print(rgb_map:select(1,2)[mask])
			print(rgb_map:select(1,3)[mask])
			print(self.cmap[{{i},{1}}])
			]]--
			for k = 1,3 do 
				sub_map = rgb_map:select(1,k)	
				--if sub_map[mask]:sum() == 0 then 			
					sub_map[mask] = self.cmap[{{i},{k}}]:squeeze()
					rgb_map[{{k},{},{}}] = sub_map
				--end
			end
		end
		collectgarbage()
	end
	--[[
	for i = 1,#rgb_maps do 
		image.display(rgb_maps[i])
	end
	]]--
	return rgb_maps
end

-- Test 
job_id = 'precise-transit-6548'
scan_ids = {1,2}
th_scan_ids = torch.Tensor(scan_ids)
output_id = string.format("scan%.3d-%.3d", th_scan_ids:min(), th_scan_ids:max())

normal_thresh = math.pi/6
residual_thresh = 100

merger = MultiScanMergePlanes.new( job_id, scan_ids, normal_thresh, residual_thresh )
merger:preparePlanes()

--[[
match_list = merger:matchPlanes()

output = {}
output.match_list = match_list
-- Output match_list 
merger.output_io:dumpTorch( output, "match_lists", output_id )
]]--

match_input = merger.output_io:loadTorch( "match_lists", output_id )
match_list = match_input.match_list

match_matrix, match_list_culled = merger:overlapCull( match_list )
print("match_matrix: ", match_matrix)
plane_sets = merger:computeMergeSets( match_matrix )
merger:printPlaneSets( plane_sets ) 

fitted_planes = merger:fitMergedPlanes( plane_sets )	
rgb_maps = merger:colorScans( fitted_planes )

print(#rgb_maps)
merged_id = string.format('merged_plane_ims%.3d-%.3d', th_scan_ids:min(), th_scan_ids:max())
-- Output colored scan images
for _,scan_id in ipairs(merger.scan_ids) do 
	merger.output_io:dumpImage( rgb_maps[scan_id], merged_id, string.format("scan%.3d", scan_id) )
end

-- Output useful data 
output = {}
output.scan_id = merger.scan_id
output.colormap = merger.cmap
output.planes = fitted_planes
merger.output_io:dumpTorch( output, "merged_planes", output_id )




