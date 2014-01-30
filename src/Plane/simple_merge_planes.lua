--[[
	Very simple plane merger using binary masks from planes as output by 
	extract_planes_taguchi
]]--
require 'gnuplot'
gnuplot.setgnuplotexe("/usr/local/bin/gnuplot")
gnuplot.setterm("x11")

planes_fname = '/Users/uriah/Downloads/precise-transit-6548/work/output/planes.t7'
output_dir = '/Users/uriah/Downloads/precise-transit-6548/work/output'

-- Load in planes, since plane masks are embedded in the datastructure this is all we need 
planes = torch.load( planes_fname )

imgh = planes[1].inlier_map:size(1)
imgw = planes[1].inlier_map:size(2)

-- Plane match matrix, represents the overlap between plane i and plane j 
match_matrix = torch.Tensor(#planes,#planes):zero()

-- Really shitty match threshold, match_matrix should really hold percentage overlap 
-- relative to the smaller plane mask 
match_threshold = 100

-- Compute overlaps between all planes 
local overlap = 0
for i=1, #planes do
	for j=i, #planes do
		overlap = (planes[i].inlier_map + planes[j].inlier_map):eq(2):sum()
		print(string.format("overlap for [%d,%d]: %d", i, j, overlap))
		match_matrix[{i,j}] = overlap
	end
end

match_mask = match_matrix:gt(0)
plane_rng = torch.range(1,#planes)

-- Utility functions for my hacky recursive merge ... its basically a poorly written DFS 
-- Return unique value from two 1D Tensors , this can be very memory intensive, but since I know that
-- I probably won't have more than at most a couple thousand planes its ok 
function unique(s0,s1)
	local rng = torch.range(1,math.max(s0:max(),s1:max()))
	local mask = torch.ByteTensor(rng:nElement()):zero()
	mask[s0:cat(s1):long()] = 1
	return rng[mask]
end

function merge(ind, ind_set)
	local cur_ind_set = plane_rng[match_mask[ind]]	
	print("cur_ind_set: ", cur_ind_set)
	for j=2, cur_ind_set:nElement() do
		cur_ind_set = merge(cur_ind_set[j], cur_ind_set)
	end
	return unique(ind_set, cur_ind_set)		
end

merged = nil
rm_mask = torch.ByteTensor(plane_rng:nElement()):fill(1)
plane_sets = {}
while true do
	if merged == nil then
		to_merge = plane_rng:clone():long()
	else
		rm_mask[merged:long()] = 0
		to_merge = plane_rng[rm_mask]
		if to_merge:nElement() == 0 then 
			break
		end
	end
	ind_set = plane_rng[match_mask[to_merge[1]]]
	if merged == nil then
		merged = ind_set
	else
		merged = unique( merged, ind_set )
	end
	table.insert(plane_sets, ind_set)
	collectgarbage()
end			

print("number of plane sets: ", #plane_sets)

for k,v in pairs(plane_sets) do 
	print("plane set:")
	print(k)
	print(v)
end

image.display(match_matrix)

local im_name = string.format("%s/plane_match_matrix.jpg",output_dir)
print("saving "..im_name)
image.save(im_name, match_matrix)







