ids = {"001","002"}
scan_paths = "/Users/stavbraun/Documents/planes/planes_for_scans_0to3/"
fplanes = {}
eq_ids = {4,2}
transformations = "/Users/stavbraun/Documents/planes/transformations"
po_loc = "/Users/stavbraun/Documents/precise-transit-6548/precise-transit-6548/source/po_scan/a/" 
save_dir = "test"
util.fs.mkdir_p(save_dir)
util.fs.mkdir_p(path.join(save_dir, "FlattenedPlaneNew"))

path = require "path"
orig_transformation = torch.load(path.join(transformations,"sweep" ..ids[1] .. ".data" ))




all_planes_orig = torch.load(path.join(scan_paths, "scan" .. ids[1] .. ".t7"))
all_planes_second = torch.load(path.join(scan_paths, "scan" .. ids[2] .. ".t7"))



--looking at equation in one plane
all_planes_orig = torch.load(path.join(scan_paths, "scan" .. ids[1] .. ".t7"))





for i=1, table.getn(ids) do
	loader = pointcloud.loader.load_pobot_ascii(path.join(po_loc, ids[i]))
	pc = pointcloud.pointcloud.new(loader)
	this_transformation = torch.load(path.join(transformations,"sweep" ..ids[i] .. ".data" ))
	all_planes = torch.load(path.join(scan_paths, "scan" .. ids[i] .. ".t7"))

	mask = all_planes.planes[eq_ids[i]].mask
	eqn = all_planes_orig.planes[eq_ids[1]].eqn
	
	fplanes[i] = flattened_plane.FlattenedPlaneNew.new(pc, mask, eqn, {this_transformation, orig_transformation})
	minT, maxT = fplanes[i]:calculate_min_and_max_t()
	if(minSoFar) then
		minSoFar = torch.min(torch.cat(minT, minSoFar,1),1)
		maxSoFar = torch.max(torch.cat(maxT, maxSoFar,1),1) 
	else
		minSoFar = minT
		maxSoFar = maxT
	end
end

for i = 1, table.getn(ids) do
	fplanes[i]:set_min_and_max(minSoFar, maxSoFar)
	fplanes[i]:add_all_frustrum_and_empties(true)	
end

image.display(fplanes[1].ioccupied)
image.display(fplanes[1].iempties)

image.display(fplanes[2].ioccupied)
