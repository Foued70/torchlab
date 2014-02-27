-- Hull Test

--get_hull = Plane.util.get_hull
ArcIO = data.ArcIO

types = require '../opencv/types/Morph.lua'


job_id = "precise-transit-6548"

-- Load in two work arcs 
region_growing_arc_io = ArcIO.new( job_id, "region-growing-random-002" )
test_arc_io = ArcIO.new( job_id, "contour_testing" )

scan_num = 7
pc = region_growing_arc_io:getScan( scan_num )

plane_id = 1

--data = torch.load("/Users/uriah/Arcs/precise-transit-6548/work/region-growing-random-002/refitted_planes/scan007.t7")
data = region_growing_arc_io:loadTorch("refitted_planes", string.format("scan%.3d", scan_num))

mask = data.planes[1].mask

--[[
loader = pointcloud.loader.load_pobot_ascii("/Users/uriah/Arcs/precise-transit-6548/source/po_scan/a/007")
pc = pointcloud.pointcloud.new(loader)
]]--
-- Gotta use opencv for this jam 
test_arc_io:dumpImage( torch.mul(mask,255), "original_masks", string.format("scan%.3d", scan_num) )

-- Extract edge points using erode/dilate
structure_element = opencv.imgproc.getDefaultStructuringMat(1)
--structure_element = opencv.imgproc.getStructuringElement(types.MORPH_CROSS, 3, 3, 0, 0);

eroded_mask = mask:clone()
opencv.imgproc.erode( opencv.Mat.new(eroded_mask), structure_element)

--image.display(eroded_mask)

test_arc_io:dumpImage( torch.mul( eroded_mask, 255 ), "eroded_masks", string.format("scan%.3d", scan_num) )

--image.display( mask:add(-eroded_mask):gt(0.5) )

contour_mask = mask:add(-eroded_mask):gt(0.5)

image.display( contour_mask )

test_arc_io:dumpImage( torch.mul(contour_mask, 255), "contour_masks", string.format("scan%.3d", scan_num) )

-- Find contours, should print out a bunch of stuff

contours = opencv.imgproc.find_contours( opencv.Mat.new(mask) )
-- Convert contour indices into 1d mask indices and draw 
--[[
for i = 1,#contours do 
	contour = contours[i]
]]--

-- Find outer contour
max_i = nil
max_x = nil 
for i = 1,#contours do 
	cur_max = contours[i]:select(2,1):max()	
	if max_i == nil or cur_max > max_x then
		max_i = i
		max_x = cur_max
	end
end

-- Outer mask 
outer_mask = torch.Tensor( contour_mask:size() ):zero()
lin_outer_mask = outer_mask:reshape(outer_mask:size(1)*outer_mask:size(2))
inds = (contours[max_i]:select(2,2) * outer_mask:size(2) + contours[max_i]:select(2,1)):long()		
lin_outer_mask[inds] = 1;

-- Check if any contours overlap our outer contour if so remove them 
-- TODO: do this in the cpp code using std::map to make the whole process reasonably fast 
overlap_mask = torch.Tensor( contour_mask:size() ):zero()
lin_overlap_mask = overlap_mask:reshape(overlap_mask:size(1)*overlap_mask:size(2))

-- Draw contours based upon contour indices just for sanity
hole_mask = torch.Tensor( contour_mask:size() ):zero()
lin_hole_mask = hole_mask:reshape(hole_mask:size(1)*hole_mask:size(2))

for i = 1,#contours do 
	lin_overlap_mask:zero()
	if i ~= max_i then 
		-- Check for overlap ... this is annoying since it can be fairly costly
		inds = (contours[i]:select(2,2) * hole_mask:size(2) + contours[i]:select(2,1)):long()		
		lin_overlap_mask[inds] = 1
		if ( (lin_overlap_mask + lin_outer_mask):eq(2):sum() == 0 ) then 							
			lin_hole_mask[inds] = 1;				
		end		
	end
end

hole_mask = lin_hole_mask:reshape(hole_mask:size(1), hole_mask:size(2))
outer_mask = lin_outer_mask:reshape(outer_mask:size(1), outer_mask:size(2))

debug_mask = torch.Tensor(3,contour_mask:size(1), contour_mask:size(2)):zero()
debug_mask[{{1},{},{}}] = hole_mask
debug_mask[{{2},{},{}}] = outer_mask

image.display(debug_mask)

test_arc_io:dumpImage( torch.mul(debug_mask, 255), "output_contours", string.format("scan%.3d", scan_num) )



--[[
contours = find_contours( opencv.Mat.new(mask) )
image.display(mask)
image.save("contours.jpg", mask)
--image.display(contours)
--hull = get_hull(mask, xyz_map)
]]--

