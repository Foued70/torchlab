--[[
	Single Scan Plane Merging
]]--

ArcIO = data.ArcIO
MergePlanes = CPlane.MergePlanes
fit_plane_to_points = Plane.util.fit_plane_to_points 

local SingleScanMergePlanes = Class( MergePlanes )

default_overlap_thresh = 100 

function SingleScanMergePlanes:__init( job_id, scan_id, input_io, normal_thresh, residual_thresh, overlap_thresh )	
	-- Initialize MergePlanes
	__super__.__init(self, job_id, input_io, normal_thresh, residual_thresh ) 

	self.overlap_thresh = overlap_thresh or default_overlap_thresh	
	self.scan_id = scan_id

	self.output_io = ArcIO.new( job_id, 'single_scan_merge_planes')
	self.regions_data = self.input_io:loadTorch("region_masks", string.format("%.3d", scan_id))	

end

function SingleScanMergePlanes:preparePlanes()
	self.planes = self.regions_data.planes
	print("n_planes: ", #self.planes)
	self.imgh = self.planes[1].inlier_map:size(1)
	self.imgw = self.planes[1].inlier_map:size(2)
end

function SingleScanMergePlanes:overlapCull( match_list )
	match_matrix = torch.ByteTensor(#self.planes,#self.planes):zero()
	culled_match_list = {}
	-- Walk through the math
	for k,m in pairs(self.match_list) do
		for _,v in ipairs(m) do 				
			overlap = (self.planes[k].inlier_map + self.planes[v].inlier_map):eq(2):sum()					
			if overlap > self.overlap_thresh then 
				match_matrix[{k,v}] = 1
				if culled_match_list[k] == nil then
					culled_match_list[k] = {}
				end
				table.insert(culled_match_list[k], j)										
			end
		end
	end
	return match_matrix, culled_match_list
end

function SingleScanMergePlanes:fitMergedPlanes( plane_sets )
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
		plane.scan_id = self.scan_id	
		plane.mask = mask:clone()
		plane.eqn = plane_eqn
		plane.centroid = plane_centroid
		plane.color = self.cmap[{{k},{}}]		
		fitted_planes[k] = plane 		
	end
	return fitted_planes
end

function SingleScanMergePlanes:colorPlaneSets( plane_sets )
	
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

-- Test 
job_id = 'precise-transit-6548'
scan_id = 2
merge = SingleScanMergePlanes.new( job_id, scan_id )

match_matrix,_ = merge:matchPlanes()
print("match_matrix: ", match_matrix)
plane_sets = merge:computeMergeSets( match_matrix )
merge:printPlaneSets( plane_sets ) 

planes_rgb = merge:colorPlaneSets( plane_sets )
fitted_planes = merge:fitMergedPlanes( plane_sets )	

-- Output planes rgb 
merge.output_io:dumpImage( planes_rgb, "merged_plane_ims", string.format("scan%.3d", merge.scan_id)) 	

-- Output useful data 
output = {}
output.scan_id = merge.scan_id
output.colormap = merge.cmap
output.planes = fitted_planes
merge.output_io:dumpTorch( output, "merged_planes", string.format("scan%.3d", merge.scan_id))

