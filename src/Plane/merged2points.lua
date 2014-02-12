--[[
	Ouput merged mask/color pairs to 3d point data to be loaded in cloudcompare
]]--

io = require('io')

ArcIO = data.ArcIO
specs = data.ArcSpecs

--[[
scan_start = 6
scan_end = 27
for scan_num = scan_start, scan_end do
]]--
scan_num = 3
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
											string.format('segmented_region_%.3d.pts', plane_idx ))

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
--end


