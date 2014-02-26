--[[
	Merged_planes to pairs

	Given the full multi-scan merged pairs data structure output only overlapping sets 

	overlapping_sets[]:
		- planes 
		- masks 
]]


ArcIO = data.ArcIO
specs = data.ArcSpecs

scan_nums = specs.merge_scan_ids

arc_io = ArcIO.new( specs.job_id, specs.work_id )

merged_planes = arc_io:dumpTorch( "multi_merged_planes", "merged_planes" )

-- TODO: Should finish this at some point ... 

