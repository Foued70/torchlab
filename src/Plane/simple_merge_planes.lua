--[[
	Very simple plane merger using binary masks from planes as output by 
	extract_planes_taguchi
]]--
require 'gnuplot'
gnuplot.setgnuplotexe("/usr/local/bin/gnuplot")
gnuplot.setterm("x11")

job_id = "precise-transit-6548"

scan_start = 6
scan_end = 27
for scan_num = scan_start, scan_end do
	print("Merging Planes for scan_num: ", scan_num)

	planes_fname = string.format('/Users/uriah/Downloads/'.. job_id .. '/work/output/planes_%.3d.t7', scan_num)
	output_dir = '/Users/uriah/Downloads/' .. job_id .. '/work/output'

	-- Load in planes, since plane masks are embedded in the datastructure this is all we need 
	planes = torch.load( planes_fname )

	imgh = planes[1].inlier_map:size(1)
	imgw = planes[1].inlier_map:size(2)

	-- Plane match matrix, represents the overlap between plane i and plane j 
	match_matrix = torch.Tensor(#planes,#planes):zero()

	-- Really shitty match threshold, match_matrix should really hold percentage overlap 
	-- relative to the smaller plane mask 
	match_threshold = 0.01

	-- Compute overlaps between all planes 
	print("Computing overlaps between all planes")
	local overlap = 0
	local smaller = 0
	for i=1, #planes do
		print(string.format("computing overlaps for plane: %d",i))
		for j=i, #planes do
			if j ~= i then
				overlap = (planes[i].inlier_map + planes[j].inlier_map):eq(2):sum()
				smaller = math.min( planes[i].inlier_map:sum(), planes[j].inlier_map:sum() )	
				if overlap/smaller < match_threshold and overlap > 0 then
					print(string.format("overlap for [%d,%d]: %f", i, j, overlap/smaller))
				end
				match_matrix[{i,j}] = overlap/smaller
				collectgarbage()
			end
		end
	end

	match_mask = match_matrix:gt(match_threshold)
	plane_rng = torch.range(1,#planes)

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


	--[[ Reverse edges in plane_sets ... no longer needed :( 
	plane_sets_new = {}
	for k,v in pairs(plane_sets) do 
		for j=1, v:nElement() do 	
			if plane_sets_new[v[j] ] == nil then
				plane_sets_new[v[j] ] = torch.IntTensor({k})
			else
				plane_sets_new[v[j] ] = plane_sets_new[v[j] ]:cat( torch.IntTensor({k}) )
			end
		end
	end
	print("Reversed Edges: ", #plane_sets_new)
	-- TODO: pretty printing 
	str = ""
	for k,v in pairs(plane_sets_new) do 
		str = str .. string.format("%d:", k)
		for j=1, v:nElement() do 
			str = str .. string.format(" %d ", v[j])
		end
		str = str .. string.format("\n")
	end
	print(str)
	]]--

	cmap = image.colormap(#plane_sets)
	planes_rgb = torch.Tensor(3, imgh, imgw)
	planes_r = planes_rgb[1]
	planes_g = planes_rgb[2]
	planes_b = planes_rgb[3]

	-- Now draw pretty merged sets and copy those masks into our plane masks set 
	plane_masks = torch.ByteTensor( #plane_sets, imgh, imgw )
	mask = torch.ByteTensor(imgh, imgw):zero()
	for k,v in pairs(plane_sets) do 
		-- Add all masks 
		mask:zero()
		for j=1, v:nElement() do 
			mask:add( planes[v[j]].inlier_map )
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
	-- image.display(planes_rgb)

	-- Save association maatrix and merged planes image
	local im_name = string.format("%s/plane_match_matrix_%.3d.jpg",output_dir, scan_num)
	print("saving "..im_name)
	image.save(im_name, match_matrix)

	local im_name = string.format("%s/merged_planes_%.3d.jpg",output_dir, scan_num)
	print("saving "..im_name)
	image.save(im_name, planes_rgb)
	collectgarbage()

	-- Save merged mask 
	data_fname = string.format("%s/merged_planes_%.3d.t7",output_dir, scan_num)
	print("saving "..data_fname)


	-- Fill out output data structure, must have scan_num and job_id  
	output = {}
	output.scan_num = scan_num
	output.job_id  = job_id 
	output.plane_masks = plane_masks
	output.colormap = cmap
	torch.save(data_fname, output)
end










