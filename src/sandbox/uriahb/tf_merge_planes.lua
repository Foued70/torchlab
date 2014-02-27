--[[
	Tf merge planes, a lazy copy of simple_merge_planes so that I don't have to clean this code up yet
	Used to match planes from two scans after transforming them into global coordinates
]]--
require 'gnuplot'
gnuplot.setgnuplotexe("/usr/local/bin/gnuplot")
gnuplot.setterm("x11")

ArcIO = data.ArcIO
specs = data.ArcSpecs
angle_between = geom.util.angle_between

scan_nums = specs.merge_scan_ids

arc_io = ArcIO.new( specs.job_id, specs.work_id )

-- Using flattenplanes to check for actual overlap 
FlattenedPlaneNew = flattened_plane.FlattenedPlaneNew


scan_ids = specs.scan_ids

-- Transform a plane given a homogenous transformation matrix 
function transform_plane( plane, transform )	

	local normal = torch.cat(plane.eqn:sub(1,3), torch.Tensor({0}))
	local centroid = torch.cat(plane.centroid, torch.Tensor({1}))	

	-- Transform plane normal and centroid
	local tfd_normal = transform*normal
	local tfd_centroid = transform*centroid

	-- Compute new d for plane eqn 
	local d = torch.Tensor({torch.dot( tfd_normal:sub(1,3), tfd_centroid:sub(1,3) )})

	local tfd_plane = {}
	tfd_plane.eqn = torch.cat( tfd_normal:sub(1,3), d)
	tfd_plane.centroid = tfd_centroid:sub(1,3)
	return tfd_plane
end

--[[
scan_start = 6
scan_end = 27
for scan_num = scan_start, scan_end do
]]--
-- for scan_i = 1,#scan_ids do 
	--scan_num = scan_ids[scan_i]	

	scan_num_map = {}
	for j = 1,#scan_nums do 
		scan_num_map[scan_nums[j]] = j
	end

	pointclouds = {}
	for j = 1,#scan_nums do 
		pointclouds[scan_nums[j]] = arc_io:getScan(scan_nums[j])
	end

	planes = {}
	for j=1,#scan_nums do 
		scan_data = arc_io:loadTorch("refitted_planes", string.format("scan%.3d", scan_nums[j]))
		table.insert( planes, scan_data.planes )
	end

	-- TODO: transform all the planes into the same coordinate system, then do ALL PAIRS matching
	-- This will allow me to use the same merge code from simple_merge_planes

	-- Load in transform ... TODO, need a way to access things outside of work_id 
	transforms = {}
	for j=1,#scan_nums do 
		tf = arc_io:loadTorch("transformations", string.format("sweep%.3d", scan_nums[j]))
		table.insert( transforms, tf )
	end

	-- Transform both sets of planes into the same coordinate frame
	tfd_planes = {}
	for j=1,#scan_nums do 
		for i=1,#planes[j] do 	
			tfd_plane = transform_plane( planes[j][i], transforms[j] )
			tfd_plane.local_eqn = planes[j][i].eqn
			tfd_plane.plane_num = i
			tfd_plane.scan_num = scan_nums[j]
			tfd_plane.inlier_map = planes[j][i].mask			
			table.insert(tfd_planes, tfd_plane)
		end
	end			

	--[[
	print( string.format("scan_nums: {%d,%d}", tfd_planes[163].scan_num, tfd_planes[55].scan_num) )
	print( string.format("plane_ids: {%d,%d}", tfd_planes[163].plane_num, tfd_planes[55].plane_num) )
	]]--

	--Use both normal and residual thresh to keep from incorrectly merging
	scan_data = arc_io:loadTorch("region_masks", string.format("%.3d", scan_nums[1]))
	normal_thresh = scan_data.normal_thresh
	--residual_thresh = scan_data.residual_thresh
	residual_thresh = 30

	print("n_planes: ", #tfd_planes)

	-- Plane match matrix, represents the overlap between plane i and plane j 
	match_matrix = torch.Tensor(#tfd_planes,#tfd_planes):zero()

	-- Really shitty match threshold, match_matrix should really hold percentage overlap 
	-- relative to the smaller plane mask 
	--match_threshold = 100

	-- Flattened planes structure
	flattened_planes = {}

	-- Compute overlaps between all planes 
	-- TODO: don't do all pairs merge, just do temporally adjacent merge 
	print("Computing overlaps between all planes")
	local overlap = 0
	local smaller = 0
	for i=1, #tfd_planes do
		print(string.format("computing overlaps for plane: %d",i))
		for j=1, #tfd_planes do
			if j ~= i then	
				--if tfd_planes[i].scan_num ~= tfd_planes[j].scan_num then 									
					--angle = angle_between( planes[i].eqn:sub(1,3) , planes[j].eqn:sub(1,3) )
					angle = angle_between( tfd_planes[i].eqn:sub(1,3) , tfd_planes[j].eqn:sub(1,3) )

					-- Not sure if this residual calculation is justified, probably the correct thing to do 
					-- is to look at each one of the overlap points and evaluate their residuals using the normal
					-- of each plane then see how many overlap ... possibly also justified to to with normal thresholds

					-- Ignore residuals for now 
					--residual = math.abs(torch.dot( planes[i].eqn:sub(1,3), torch.add(planes[i].centroid, -planes[j].centroid) ))
					residual = math.abs(torch.dot( tfd_planes[i].eqn:sub(1,3), torch.add(tfd_planes[j].centroid, -tfd_planes[i].centroid) ))
					--print(string.format("angle between [%d,%d]: %f", i,j,angle))
					if angle < normal_thresh and residual < residual_thresh then
						
						--[[
						print(string.format("overlap for [%d,%d]", i,j))
						print( string.format("scan_nums: [%d,%d]", tfd_planes[i].scan_num, tfd_planes[j].scan_num) )
						print( string.format("plane_ids: [%d,%d]", tfd_planes[i].plane_num, tfd_planes[j].plane_num) )

						orig_transformation = transforms[scan_num_map[tfd_planes[i].scan_num] ]
						new_transformation = transforms[scan_num_map[tfd_planes[j].scan_num] ]
									
						local fplane_i = FlattenedPlaneNew.new(pointclouds[tfd_planes[i].scan_num], 
																    tfd_planes[i].local_eqn, 
																    tfd_planes[i].inlier_map,
																    {orig_transformation, orig_transformation})
						local minT, maxT = fplane_i:calculate_min_and_max_t()												
						local minSoFar = minT
						local maxSoFar = maxT											

						local fplane_j = FlattenedPlaneNew.new(pointclouds[tfd_planes[j].scan_num], 
																    tfd_planes[i].local_eqn, 
																    tfd_planes[j].inlier_map,
																    {new_transformation, orig_transformation})						

						local minT, maxT = fplane_j:calculate_min_and_max_t()						
						minSoFar = torch.min(torch.cat(minT, minSoFar,1),1)
						maxSoFar = torch.max(torch.cat(maxT, maxSoFar,1),1) 						

						fplane_i:set_min_and_max(minSoFar, maxSoFar)
						fplane_i:add_all_frustrum_and_empties(true)							
						fplane_j:set_min_and_max(minSoFar, maxSoFar)
						fplane_j:add_all_frustrum_and_empties(true)		

						local rgb_map = torch.Tensor(3,fplane_i.ioccupiedF:size(1),fplane_i.ioccupiedF:size(2)):zero()
						-- Output images for each overlapping pair 					
						rgb_map[{{1},{},{}}] = fplane_i.ioccupiedF
						rgb_map[{{2},{},{}}] = fplane_j.ioccupiedF
						arc_io:dumpImage( rgb_map, "fplane_overlaps", 
							              string.format("scans_%.3d_%.3d_planes_%.3d_%.3d", tfd_planes[i].scan_num, tfd_planes[j].scan_num, 
							              	 											    tfd_planes[i].plane_num, tfd_planes[j].plane_num) )


						print("residual: ", residual)
						print("angle: ", angle)
					--if angle < normal_thresh then 
						]]--
						--if (fplane_i.ioccupiedF + fplane_j.ioccupiedF):gt(1.5):sum() > 0 then
							match_matrix[{i,j}] = 1
						--end
					end
				--end
				collectgarbage()
			end
		end
	end

	match_mask = match_matrix:gt(0.5)

	plane_rng = torch.range(1,#tfd_planes)

	print("Size of match_matrix: ", match_matrix:size())
	

	-- Utility functions for my hacky recursive merge ... 
	-- TODO: re-write/use-a-lib this since its basically a horribly implemented dfs 

	-- Return unique value from two 1D Tensors , this can be very memory intensive, but since I know that
	-- I probably won't have more than at most a couple thousand planes its ok 
	function unique(s0,s1)
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

	-- subtract set s1 from set s0 
	function subtract(s0,s1)
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

	-- Note this merge is probably inefficient, the for loop can run unnecessarily over 
	-- nodes that have already been merged 
	function merge(ind, ind_set )
		local h_set = plane_rng[match_mask[{ind,{}}]]
		local v_set = plane_rng[match_mask[{{},ind}]]
		local cur_ind_set = unique(h_set, v_set)
		-- Only explore where what we haven't already seen
		ind_set = unique( ind_set, torch.Tensor({ind}) )
		cur_ind_set = subtract(cur_ind_set, ind_set)
		for j=1, cur_ind_set:nElement() do
			print(string.format("cur_ind_set[%d]", j))
			ind_set = merge(cur_ind_set[j], ind_set)
		end
		-- Don't forget to add our ind 
		return unique(ind_set, cur_ind_set)		
	end

	print("Computing Merge Sets")
	merged = nil
	rm_mask = torch.ByteTensor(plane_rng:nElement()):fill(1)
	plane_sets = {}
	while true do
		if merged == nil then
			to_merge = plane_rng:clone():long()
		else
			-- TODO: replace with subtract 
			rm_mask[merged:long()] = 0
			to_merge = plane_rng[rm_mask]
			if to_merge:nElement() == 0 then 
				break
			end
		end
		ind_set = merge(to_merge[1],torch.Tensor({}))
		if merged == nil then
			merged = ind_set
		else
			merged = unique( merged, ind_set )
		end
		table.insert(plane_sets, ind_set)
		collectgarbage()
	end			

	print("Number of merged plane sets: ", #plane_sets)
	-- pretty printing 
	str = ""
	for k,v in pairs(plane_sets) do 
		str = str .. string.format("%d:", k)
		for j=1, v:nElement() do 
			str = str .. string.format(" %d ", v[j])
		end
		str = str .. string.format("\n")
	end
	print(str)


	print("Number of merged plane sets: ", #plane_sets)
	-- pretty printing 
	str = ""
	for k,v in pairs(plane_sets) do 
		str = str .. string.format("%d:", k)
		for j=1, v:nElement() do 
			str = str .. string.format(" %d ", v[j])
		end
		str = str .. string.format("\n")
	end
	print(str)

	-- Merge masks and output 
	cmap = image.colormap(#plane_sets)
	for j=1,#scan_nums do 

		imgh = planes[j][1].mask:size(1)
		imgw = planes[j][1].mask:size(2)	

		planes_rgb = torch.Tensor(3, imgh, imgw):zero()
		planes_r = planes_rgb[1]
		planes_g = planes_rgb[2]
		planes_b = planes_rgb[3]

		-- Now draw pretty merged sets and copy those masks into our plane masks set 
		plane_masks = torch.ByteTensor( #plane_sets, imgh, imgw )
		mask = torch.ByteTensor(imgh, imgw):zero()
		for k,v in pairs(plane_sets) do 			
			-- Add all masks 
			mask:zero()
			for l=1, v:nElement() do 
				if tfd_planes[ v[l] ].scan_num == scan_nums[j] then
					mask:add( tfd_planes[ v[l] ].inlier_map )
				end
			end
			mask = mask:gt(0.5) 
			planes_r[mask] = cmap[{k,1}]
			planes_g[mask] = cmap[{k,2}]
			planes_b[mask] = cmap[{k,3}]

			-- Write to plane masks 
			plane_masks[{k,{},{}}] = mask						
		end
		planes_rgb[1] = planes_r
		planes_rgb[2] = planes_g
		planes_rgb[3] = planes_b

		image.display(planes_rgb)
		collectgarbage()

		-- Output data for each scan 
		pc = arc_io:getScan( scan_nums[j] ) -- TODO: shouldn't need to do this
		arc_io:dumpImage( planes_rgb, "multi_scan_merged_plane_ims", string.format("%.3d", scan_nums[j]))
		output = {}
		output.scan_num = scan_nums[j]
		output.planes = tfd_planes
		output.plane_masks = plane_masks
		output.colormap = cmap
		arc_io:dumpTorch( output, "multi_scan_merged_planes", string.format("%.3d", scan_nums[j]))
	end
 --[[	
	pc = arc_io:getScan( scan0 ) -- TODO: shouldn't need to do this
	arc_io:dumpImage( planes_rgb, "multi_scan_merged_plane_ims", string.format("%.3d_to_%.3d", scan0, scan1))
	output = {}
	output.scan_num = scan0
	output.planes = tfd_planes
	output.plane_masks = plane_masks
	output.colormap = cmap
	arc_io:dumpTorch( output, "multi_scan_merged_planes", string.format("%.3d_to_%.3d", scan0, scan1))

	-- Copied
	imgh1 = planes1[1].inlier_map:size(1)
	imgw1 = planes1[1].inlier_map:size(2)

	cmap = image.colormap(#plane_sets)

	planes_rgb = torch.Tensor(3, imgh1, imgw1):zero()
	planes_r = planes_rgb[1]
	planes_g = planes_rgb[2]
	planes_b = planes_rgb[3]

	-- Now draw pretty merged sets and copy those masks into our plane masks set 
	plane_masks = torch.ByteTensor( #plane_sets, imgh1, imgw1 )
	mask = torch.ByteTensor(imgh1, imgw1):zero()
	for k,v in pairs(plane_sets) do 
		--if tfd_planes[ k ].scan_num == scan1 then
			-- Add all masks 
			mask:zero()
			for j=1, v:nElement() do 
				if tfd_planes[ v[j] ].scan_num == scan1 then
					mask:add( tfd_planes[ v[j] ].inlier_map )
				end
			end
			mask = mask:gt(0.5) 
			planes_r[mask] = cmap[{k,1}]
			planes_g[mask] = cmap[{k,2}]
			planes_b[mask] = cmap[{k,3}]

			-- Write to plane masks 
			plane_masks[{k,{},{}}] = mask
		--end
	end
	planes_rgb[1] = planes_r
	planes_rgb[2] = planes_g
	planes_rgb[3] = planes_b

	image.display(planes_rgb)

	pc = arc_io:getScan( scan1 ) -- TODO: shouldn't need to do this
	arc_io:dumpImage( planes_rgb, "multi_scan_merged_plane_ims", string.format("%.3d_to_%.3d", scan1, scan0))
	output = {}
	output.scan_num = scan1
	output.planes = tfd_planes
	output.plane_masks = plane_masks
	output.colormap = cmap
	arc_io:dumpTorch( output, "multi_scan_merged_planes", string.format("%.3d_to_%.3d", scan1, scan0))


	arc_io:dumpImage( match_matrix, "tf_merge_match_matrices", string.format("%.3d", scan0))
--end
]]--









