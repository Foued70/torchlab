ids = {"001","002"}
scan_paths = "/Users/stavbraun/Documents/planes/planes_for_all_scans/"
fplanes = {}
transformations = "/Users/stavbraun/Documents/planes/transformations"
po_loc = "/Users/stavbraun/Documents/arc/precise-transit-6548/source/po_scan/a/" 

path = require "path"
orig_transformation = torch.load(path.join(transformations,"sweep" ..ids[1] .. ".data" ))


eq_ids = {4,2} --floors

--looking at equation in one plane
all_planes_orig = torch.load(path.join(scan_paths, "scan" .. ids[1] .. ".t7"))


minSoFar = nil
maxSoFar = nil
--this finds all points that satisfy eq1 in sweep ids[1] and ids[2]
for i=1, table.getn(ids) do
	pc = pointcloud.loader.load_pobot_ascii(path.join(po_loc, ids[i]))
	this_transformation = torch.load(path.join(transformations,"sweep" ..ids[i] .. ".data" ))
	all_planes = torch.load(path.join(scan_paths, "scan" .. ids[i] .. ".t7"))
	vmsk = pc:get_valid_masks()
	mask = all_planes.planes[eq_ids[i]].mask:sub(1,-1,1,vmsk:size(2))
	eqn = all_planes_orig.planes[eq_ids[1]].eqn --seeing which satisfy the first one's equation
	
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




---add intersections to plane 1, with other planes all in sweep 1 (say planes 1-3)
pc = pointcloud.loader.load_pobot_ascii(path.join(po_loc, ids[1]))
this_transformation = torch.load(path.join(transformations,"sweep" ..ids[1] .. ".data" ))
all_planes = torch.load(path.join(scan_paths, "scan" .. ids[i] .. ".t7"))
vmsk = pc:get_valid_masks()

intersect_line = {}
for i=1,3 do
	mask = all_planes.planes[i].mask:sub(1,-1,1,vmsk:size(2)):clone()
	eqn = all_planes_orig.planes[i].eqn
	tplane = flattened_plane.FlattenedPlaneNew.new(pc, eqn, mask, {this_transformation, orig_transformation})
	intersect_line[i] = fplanes[1]:get_intersection_with(tplane)
end
image.display(fplanes[1].ioccupiedF + intersect_line[2]:getLineImage())

