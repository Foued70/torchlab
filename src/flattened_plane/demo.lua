ids = {"024","022"}
scan_paths = "/Users/stavbraun/Documents/planes/planes_for_all_scans/"
fplanes = {}
transformations = "/Users/stavbraun/Documents/planes/transformations"
po_loc = "/Users/stavbraun/Documents/precise-transit-6548/precise-transit-6548/source/po_scan/a/" 

path = require "path"
orig_transformation = torch.load(path.join(transformations,"sweep" ..ids[1] .. ".data" ))


eq_ids = {43,55} --good_ids[2]

--looking at equation in one plane
all_planes_orig = torch.load(path.join(scan_paths, "scan" .. ids[1] .. ".t7"))


minSoFar = nil
maxSoFar = nil
for i=1, table.getn(ids) do
	pc = pointcloud.loader.load_pobot_ascii(path.join(po_loc, ids[i]))
	this_transformation = torch.load(path.join(transformations,"sweep" ..ids[i] .. ".data" ))
	all_planes = torch.load(path.join(scan_paths, "scan" .. ids[i] .. ".t7"))

	mask = all_planes.planes[eq_ids[i]].mask
	eqn = all_planes_orig.planes[eq_ids[1]].eqn
	
	fplanes[i] = flattened_plane.FlattenedPlaneNew.new(pc, eqn, mask, {this_transformation, orig_transformation})
	fplanes[i]:set_min_and_max(nil, nil)
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

result = torch.zeros(3, fplanes[1].ioccupied:size(1), fplanes[1].ioccupied:size(2))
result[1] = fplanes[1].ioccupiedF
result[3] = fplanes[2].ioccupiedF
image.display(result)