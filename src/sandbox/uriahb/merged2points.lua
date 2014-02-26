--[[
	Ouput merged mask/color pairs to 3d point data to be loaded in cloudcompare
]]--

io = require('io')

fit_plane_to_points = Plane.util.fit_plane_to_points 

ArcIO = data.ArcIO
specs = data.ArcSpecs

scan_ids = specs.m2p_scan_ids
--scan_ids = {22,24}
--[[
scan_start = 6
scan_end = 27
for scan_num = scan_start, scan_end do
]]--
transform_points = false

for scan_i = 1,#scan_ids do 
	scan_num = scan_ids[scan_i]
	print("Outputing Points for Segmented Region in scan_num: ", scan_num)

	arc_io = ArcIO.new( specs.job_id, specs.work_id )

	pc = arc_io:getScan( scan_num )
	points_map = pc:get_xyz_map()

	-- Load in merged planes 
	input_data = arc_io:loadTorch( "merged_planes", string.format("%.3d", scan_num))

	if transform_points then 
		-- Load in transform ... TODO, need a way to access things outside of work_id 	
		transform = arc_io:loadTorch("transformations", string.format("sweep%.3d", scan_num))

		-- Transform points map 
		pts = torch.reshape(points_map, 3,points_map:size(2)*points_map:size(3))
		pts = torch.cat( pts, torch.Tensor(1,points_map:size(2)*points_map:size(3)):fill(1), 1)	
		pts = transform*pts
		points_map = torch.reshape(pts:sub(1,3), 3, points_map:size(2), points_map:size(3))	
	end

	cmap = input_data.colormap:mul(255.0):int()
	plane_masks = input_data.plane_masks

	-- For each plane mask convert to x,y,z,r,g,b pointcloud and write to .pts file 
	num_planes = plane_masks:size(1)
	print("Number of planes: ", num_planes)
	local output_fname = arc_io:workStr("segmented_points_cull", string.format("%.3d.pts",scan_num))										
	print("Writing segmented region: ", output_fname)
	local output_fh = io.open( output_fname, "w" )
	
	planes = {}

	torch.range(1,num_planes):apply( function( plane_idx ) 
		local mask = plane_masks[{plane_idx, {}, {}}]
		local points_x = points_map:select(1,1)[mask]
		local points_y = points_map:select(1,2)[mask]
		local points_z = points_map:select(1,3)[mask]

		-- Fit plane to new planes 
		pts = torch.cat(torch.cat(points_x,points_y,2), points_z,2)
		plane_eqn, plane_centroid = fit_plane_to_points(pts:t())

		local plane_normal = plane_eqn:sub(1,3)


		plane = {}		
		plane.mask = mask
		plane.eqn = plane_eqn
		plane.centroid = plane_centroid
		plane.color = cmap[plane_idx]
		planes[plane_idx] = plane 

		-- DEBUG: Don't draw if its outside of bounds
		if math.acos( torch.dot( plane_normal/plane_normal:norm(), plane_centroid/plane_centroid:norm() )) < (math.pi/2 - math.pi/16) then


			-- Output point data for visualization 
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

	-- Output plane data 
	output = {}
	output.planes = planes
	output.scan_id = scan_num	
	output.colormap = cmap

	-- Output merged planes datastructure
	arc_io:dumpTorch( output, "refitted_planes", string.format("scan%.3d", scan_num) )

	-- Gotta collect that garbage just in case 
	collectgarbage()
end



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
