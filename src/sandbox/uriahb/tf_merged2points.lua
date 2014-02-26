--[[
	Ouput merged mask/color pairs to 3d point data to be loaded in cloudcompare
]]--

io = require('io')

ArcIO = data.ArcIO
specs = data.ArcSpecs

--scan_ids = specs.m2p_scan_ids
--[[
scan_start = 6
scan_end = 27
for scan_num = scan_start, scan_end do
]]--

--scan_nums = {2,3}
scan_nums = specs.merge_scan_ids

arc_io = ArcIO.new( specs.job_id, specs.work_id )

fit_plane_to_points = Plane.util.fit_plane_to_points 

-- Store all points from all scans for each region

merged_planes = {}
merged_points = {}

for j=1,#scan_nums do 

--for scan_i = 1,#scan_ids do 
	--scan_num = scan_ids[scan_i]
	print("Outputing Points for Segmented Region in scan_num: ", scan_nums[j])

	-- Load in merged planes 
	--input_data = arc_io:loadTorch( "merged_planes", string.format("%.3d", scan_num))
	input_data = arc_io:loadTorch( "multi_scan_merged_planes", string.format("%.3d", scan_nums[j]) )

	-- Load in transform ... TODO, need a way to access things outside of work_id 
	transform = arc_io:loadTorch("transformations", string.format("sweep%.3d", scan_nums[j]))

	pc = arc_io:getScan( scan_nums[j] )
	points_map = pc:get_xyz_map()

	-- Transform points map 
	pts = torch.reshape(points_map, 3,points_map:size(2)*points_map:size(3))
	pts = torch.cat( pts, torch.Tensor(1,points_map:size(2)*points_map:size(3)):fill(1), 1)	
	pts = transform*pts
	points_map = torch.reshape(pts:sub(1,3), 3, points_map:size(2), points_map:size(3))

	cmap = input_data.colormap:mul(255.0):int()
	plane_masks = input_data.plane_masks
	planes = input_data.planes
	print(input_data.planes[1])

	-- For each plane mask convert to x,y,z,r,g,b pointcloud and write to .pts file 
	num_planes = plane_masks:size(1)
	print("Number of planes: ", num_planes)
	local output_fname = arc_io:workStr("total_merge", 
										string.format("merged_points%.3d.pts", scan_nums[j]))
	print("Writing segmented region: ", output_fname)
	local output_fh = io.open( output_fname, "w" )	

	torch.range(1,num_planes):apply( function( plane_idx ) 

		-- Check if plane normal is too perpendicular to ray to centroid		
			local mask = plane_masks[{plane_idx, {}, {}}]
			if mask:gt(0.5):sum() > 0 then

				local points_x = points_map:select(1,1)[mask]
				local points_y = points_map:select(1,2)[mask]
				local points_z = points_map:select(1,3)[mask]

				-- Accumulate all points into appropriate bins
				pts = torch.cat(torch.cat(points_x,points_y,2), points_z,2)

				-- Fill in merged_planes datastructure
				if merged_planes[plane_idx] == nil then
					plane = {}
					plane.pts = pts
					plane.scan_ids = {}
					plane.masks = {}
					plane.eqns = {}
					plane.centroids = {}
					merged_planes[plane_idx] = plane
				else				
					print(merged_planes[plane_idx].pts)
					merged_planes[plane_idx].pts = torch.cat(merged_planes[plane_idx].pts,pts,1)
				end
				plane_eqn, plane_centroid = fit_plane_to_points(pts:t())
				table.insert(merged_planes[plane_idx].eqns, plane_eqn)
				table.insert(merged_planes[plane_idx].centroids, plane_centroid)
				table.insert(merged_planes[plane_idx].masks, mask)
				table.insert(merged_planes[plane_idx].scan_ids, j)

				--[[
				if merged_points[plane_idx] == nil then
					merged_points[plane_idx] = pts
				else
					merged_points[plane_idx] = torch.cat(merged_points[plane_idx],pts,1)
				end
				]]--
				print(string.format("plane_idx: %d", plane_idx),merged_planes[plane_idx].pts:size())

			-- Not sure why i need this if statement
				torch.range(1,points_x:nElement()):apply( function( idx )
					local rgb = cmap[plane_idx]
					output_fh:write( points_x[idx] ..' '.. points_y[idx] ..' '.. points_z[idx] ..' '.. 
									 rgb[1] ..' '..   rgb[2] ..' '.. rgb[3] .. '\n')	 
				end
				)
			end			
		
	end
	)
	output_fh:close()

	-- Gotta collect that garbage just in case 
	collectgarbage()
end

-- Now fit planes to all the merged sets 
for plane_idx = 1,#merged_planes do 
	pts = merged_planes[plane_idx].pts		
	plane_eqn, plane_centroid = fit_plane_to_points(pts:t())

	merged_planes[plane_idx].eqn = plane_eqn
	merged_planes[plane_idx].centroid = plane_centroid	
	merged_planes[plane_idx].color = cmap[plane_idx]	
end

output = {}
output.planes = merged_planes
output.scan_nums = scan_nums

-- Output merged planes datastructure
arc_io:dumpTorch( output, "multi_merged_planes", "merged_planes")



--[[
	-- Gross copy 
	-- Load in merged planes 
	--input_data = arc_io:loadTorch( "merged_planes", string.format("%.3d", scan_num))
	input_data = arc_io:loadTorch( "multi_scan_merged_planes", string.format("%.3d_to_%.3d", scan1, scan0))

	-- Load in transform ... TODO, need a way to access things outside of work_id 
	transform = arc_io:loadTorch("transformations", string.format("sweep%.3d", scan1))

	pc = arc_io:getScan( scan1 )
	points_map = pc:get_xyz_map()

	-- Transform points map 
	pts = torch.reshape(points_map, 3,points_map:size(2)*points_map:size(3))
	pts = torch.cat( pts, torch.Tensor(1,points_map:size(2)*points_map:size(3)):fill(1), 1)	
	pts = transform*pts
	points_map = torch.reshape(pts:sub(1,3), 3, points_map:size(2), points_map:size(3))

	cmap = input_data.colormap:mul(255.0):int()
	plane_masks = input_data.plane_masks
	planes = input_data.planes
	print(input_data.planes)

	-- For each plane mask convert to x,y,z,r,g,b pointcloud and write to .pts file 
	num_planes = plane_masks:size(1)
	print("Number of planes: ", num_planes)
	local output_fname = arc_io:workStr(string.format("multi_merge_%.3d_to_%.3d", scan1, scan0), 
										'segmented_regions.pts')
	print("Writing segmented region: ", output_fname)
	local output_fh = io.open( output_fname, "w" )
	

	torch.range(1,num_planes):apply( function( plane_idx ) 		
		local mask = plane_masks[{plane_idx, {}, {}}]
		if mask:gt(0.5):sum() > 0 then

			local points_x = points_map:select(1,1)[mask]
			local points_y = points_map:select(1,2)[mask]
			local points_z = points_map:select(1,3)[mask]
			print(points_x:nElement())

		-- Not sure why i need this if statement
			torch.range(1,points_x:nElement()):apply( function( idx )
				local rgb = cmap[plane_idx]
				output_fh:write( points_x[idx] ..' '.. points_y[idx] ..' '.. points_z[idx] ..' '.. 
								 rgb[1] ..' '..   rgb[2] ..' '.. rgb[3] .. '\n')	 
			end
			)
		end			
	end
	)
	output_fh:close()

	-- Gotta collect that garbage just in case 
	collectgarbage()
--end

]]--
--[[
for scan_i = 1,#scan_ids do 
	scan_num = scan_ids[scan_i]
	print("Outputing Points for Segmented Region in scan_num: ", scan_num)

	arc_io = ArcIO.new( specs.job_id, specs.work_id )
	input_data = arc_io:loadTorch( "merged_planes", string.format("%.3d", scan_num))

	pc = arc_io:getScan( scan_num )
	points_map = pc:get_xyz_map()

	cmap = input_data.colormap:mul(255.0):int()
	plane_masks = input_data.plane_masks

	-- For each plane mask convert to x,y,z,r,g,b pointcloud and write to .pts file 
	num_planes = plane_masks:size(1)
	print("Number of planes: ", num_planes)

	torch.range(1,num_planes):apply( function( plane_idx ) 
		local mask = plane_masks[{plane_idx, {}, {}}]
		local points_x = points_map:select(1,1)[mask]
		local points_y = points_map:select(1,2)[mask]
		local points_z = points_map:select(1,3)[mask]
		local output_fname = arc_io:workStr(string.format("%.3d",scan_num), 
											string.format('segmented_region_%.3d.pts', plane_idx))
		print("Writing segmented region: ", output_fname)
		local output_fh = io.open( output_fname, "w" )

		torch.range(1,points_x:nElement()):apply( function( idx )
			local rgb = cmap[plane_idx]
			output_fh:write( points_x[idx] ..' '.. points_y[idx] ..' '.. points_z[idx] ..' '.. 
							 rgb[1] ..' '..   rgb[2] ..' '.. rgb[3] .. '\n')	 
		end
		)
		output_fh:close()
	end
	)

	-- Gotta collect that garbage just in case 
	collectgarbage()
end
]]--
