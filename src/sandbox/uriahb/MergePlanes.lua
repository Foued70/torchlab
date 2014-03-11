--[[
	Merge planes:
		basic single scan plane merging implementation, hopefully this can be avoided in the future
]]--

ArcIO = data.ArcIO
specs = data.ArcSpecs

angle_between = geom.util.angle_between
fit_plane_to_points = Plane.util.fit_plane_to_points 

local MergePlanes = Class()

default_normal_thresh = math.pi/8
default_residual_thresh = 15
default_overlap_thresh = 100 

function MergePlanes:__init( job_id, scan_id, normal_thresh, residual_thresh, overlap_thresh ) 
	self.job_id = job_id
	self.scan_id = scan_id

	self.normal_thresh = normal_thresh or default_normal_thresh
	self.residual_thresh = residual_thresh or default_residual_thresh
	self.overlap_thresh = overlap_thresh or default_overlap_thresh

	self.input_io = ArcIO.new( job_id, 'extract_planes_ransac' )
	self.output_io = ArcIO.new( job_id, 'merged_planes' )

	regions_data = self.input_io:loadTorch("region_masks", string.format("%.3d", scan_id))
	self.planes = regions_data.planes
	self.plane_rng = torch.range(1,#self.planes)	

	print("n_planes: ", #self.planes)

	self.imgh = self.planes[1].inlier_map:size(1)
	self.imgw = self.planes[1].inlier_map:size(2)

	self.match_mask = self:computeOverlaps()
	-- Output match mask 
	self.output_io:dumpImage( torch.mul(self.match_mask,255), "match_matrices", string.format("scan%.3d", self.scan_id))

	plane_sets = self:computeMergeSets()
	
	fitted_planes = self:fitMergedPlanes( plane_sets )	

	self:printPlaneSets( plane_sets )
	planes_rgb = self:colorPlaneSets( plane_sets )

	-- Output planes rgb 
	self.output_io:dumpImage( planes_rgb, "merged_plane_ims", string.format("scan%.3d", self.scan_id)) 	

	-- Output useful data 
	output = {}
	output.scan_id = self.scan_id
	output.colormap = self.cmap
	output.planes = fitted_planes
	self.output_io:dumpTorch( output, "merged_planes", string.format("scan%.3d", self.scan_id))

end

function MergePlanes:computeOverlaps( )
	-- Plane match matrix, represents the overlap between plane i and plane j 
	match_matrix = torch.Tensor(#self.planes,#self.planes):zero()

	-- Compute overlaps between all planes 
	print("Computing overlaps between all planes")
	local overlap = 0
	local smaller = 0
	for i=1, #self.planes do
		print(string.format("computing overlaps for plane: %d",i))
		for j=i, #self.planes do
			if j ~= i then
				angle = angle_between( self.planes[i].eqn:sub(1,3) , self.planes[j].eqn:sub(1,3) )				
				residual = math.abs(torch.dot( self.planes[i].eqn:sub(1,3), torch.add(self.planes[i].centroid, -self.planes[j].centroid) ))				
				if angle < self.normal_thresh and residual < self.residual_thresh then
					print(string.format("angle for [%d,%d]: %f", i, j, angle))
					print(string.format("residual for [%d,%d]: %f", i, j, residual))
					overlap = (self.planes[i].inlier_map + self.planes[j].inlier_map):eq(2):sum()
					if overlap > self.overlap_thresh then 							
						match_matrix[{i,j}] = overlap
					end				
				end				
				collectgarbage()
			end
		end
	end
	return match_matrix:gt(0)
end

function MergePlanes:recursiveMerge( ind, ind_set )
	local h_set = self.plane_rng[self.match_mask[{ind,{}}]]
	local v_set = self.plane_rng[self.match_mask[{{},ind}]]	
	local cur_ind_set = self:unique_set(h_set, v_set)	
	-- Only explore where what we haven't already seen
	local ind_set = self:unique_set( ind_set, torch.Tensor({ind}) )	
	local cur_ind_set = self:subtract_sets(cur_ind_set, ind_set)
	for j=1, cur_ind_set:nElement() do		
		ind_set = self:recursiveMerge(cur_ind_set[j], ind_set)
	end
	-- Don't forget to add our ind 
	return self:unique_set(ind_set, cur_ind_set)		
end

function MergePlanes:computeMergeSets( )
	print("Computing merge sets")
	local to_merge
	local ind_set
	local merged = nil
	local rm_mask = torch.ByteTensor(self.plane_rng:nElement()):fill(1)
	local plane_sets = {}
	while true do
		if merged == nil then
			to_merge = self.plane_rng:clone():long()
		else
			-- TODO: replace with subtract 
			rm_mask[merged:long()] = 0
			to_merge = self.plane_rng[rm_mask]
			if to_merge:nElement() == 0 then 
				break
			end
		end		
		ind_set = self:recursiveMerge(to_merge[1],torch.Tensor({}))
		if merged == nil then
			merged = ind_set
		else
			merged = self:unique_set( merged, ind_set )
		end		
		table.insert(plane_sets, ind_set)
		collectgarbage()
	end	
	self.cmap = image.colormap(#plane_sets)
	return plane_sets			
end

function MergePlanes:printPlaneSets( plane_sets )
	-- pretty printing 
	print("Number of merged plane sets: ", #plane_sets)	
	str = ""
	for k,v in pairs(plane_sets) do		
		str = str .. string.format("%d:", k)
		for j=1, v:nElement() do 
			str = str .. string.format(" %d ", v[j])
		end
		str = str .. string.format("\n")
	end
	print(str)
end

function MergePlanes:fitMergedPlanes( plane_sets )
	-- Get plane masks and planes	
	local points_map = self.input_io:getScan( self.scan_id ):get_xyz_map()
	
	local mask = torch.ByteTensor(self.imgh, self.imgw):zero()
	local fitted_planes = {}
	for k,v in pairs(plane_sets) do 
		-- Add all masks 
		mask:zero()
		for j=1, v:nElement() do 
			mask:add( self.planes[v[j]].inlier_map )
		end
		mask = mask:gt(0.5) 
		local points_x = points_map:select(1,1)[mask]
		local points_y = points_map:select(1,2)[mask]
		local points_z = points_map:select(1,3)[mask]
		-- Fit plane to new planes 
		local pts = torch.cat(torch.cat(points_x,points_y,2), points_z,2)
		local plane_eqn, plane_centroid = fit_plane_to_points(pts:t())
		local plane_normal = plane_eqn:sub(1,3)

		plane = {}		
		plane.mask = mask:clone()
		plane.eqn = plane_eqn
		plane.centroid = plane_centroid
		plane.color = self.cmap[{{k},{}}]
		print(k)
		fitted_planes[k] = plane 		
	end
	return fitted_planes
end

function MergePlanes:colorPlaneSets( plane_sets )
	
	planes_rgb = torch.Tensor(3, self.imgh, self.imgw):zero()
	planes_r = planes_rgb[1]
	planes_g = planes_rgb[2]
	planes_b = planes_rgb[3]

	-- Now draw pretty merged sets and copy those masks into our plane masks set 
	plane_masks = torch.ByteTensor( #plane_sets, self.imgh, self.imgw )
	mask = torch.ByteTensor(self.imgh, self.imgw):zero()
	for k,v in pairs(plane_sets) do 
		-- Add all masks 
		mask:zero()
		for j=1, v:nElement() do 
			mask:add( self.planes[v[j]].inlier_map )
		end
		mask = mask:gt(0.5) 
		planes_r[mask] = self.cmap[{k,1}]
		planes_g[mask] = self.cmap[{k,2}]
		planes_b[mask] = self.cmap[{k,3}]

		-- Write to plane masks 
		plane_masks[{k,{},{}}] = mask
	end
	planes_rgb[1] = planes_r
	planes_rgb[2] = planes_g
	planes_rgb[3] = planes_b
	return planes_rgb
end

-- Utility set functions 
function MergePlanes:unique_set( s0, s1 )
	-- Check if s0 or s1 are empty if so return the other, or empty
	if s0:dim() == 0 and s1:dim() == 0 then
		return torch.Tensor({})
	end
	if s0:dim() == 0 then
		return s1
	end
	if s1:dim() == 0 then
		return s0
	end
	local rng = torch.range(1,math.max(s0:max(),s1:max()))
	local mask = torch.ByteTensor(rng:nElement()):zero()
	mask[s0:cat(s1):long()] = 1
	return rng[mask]
end

function MergePlanes:subtract_sets( s0, s1 )
	if s0:dim() == 0 then
		return s0 
	end
	if s1:dim() == 0 then
		return s0 
	end
	local mx = math.max(s0:max(), s1:max())	
	local mask = torch.ByteTensor(mx):zero()
	mask[s0:long()] = 1
	mask[s1:long()] = 0 
	return torch.range(1,mx)[mask]
end

-- Test
job_id = 'precise-transit-6548'

scan_ids = specs.scan_ids[job_id]
for scan_i = 1,#scan_ids do 
	scan_num = scan_ids[scan_i]	
	MergePlanes.new( job_id, scan_num )
end


