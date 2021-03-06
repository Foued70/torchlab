--[[
	Very simple plane merger using binary masks from planes as output by 
	extract_planes_taguchi
]]--
require 'gnuplot'
gnuplot.setgnuplotexe("/usr/local/bin/gnuplot")
gnuplot.setterm("x11")

ArcIO = data.ArcIO
specs = data.ArcSpecs
angle_between = geom.util.angle_between

arc_io = ArcIO.new( specs.job_id, specs.work_id )

scan_ids = specs.scan_ids

--[[
scan_start = 6
scan_end = 27
for scan_num = scan_start, scan_end do
]]--
for scan_i = 1,#scan_ids do 
	scan_num = scan_ids[scan_i]

	print("Merging Planes for scan_num: ", scan_num)
	pc = arc_io:getScan( scan_num ) -- TODO: shouldn't need to do this
	regions_data = arc_io:loadTorch("region_masks", string.format("%.3d", scan_num))
	planes = regions_data.planes

	-- Use both normal and residual thresh to keep from incorrectly merging
	normal_thresh = regions_data.normal_thresh
	residual_thresh = regions_data.residual_thresh

	imgh = planes[1].inlier_map:size(1)
	imgw = planes[1].inlier_map:size(2)

	print("n_planes: ", #planes)
	-- Plane match matrix, represents the overlap between plane i and plane j 
	match_matrix = torch.Tensor(#planes,#planes):zero()

	-- Really shitty match threshold, match_matrix should really hold percentage overlap 
	-- relative to the smaller plane mask 
	match_threshold = 100

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
				--[[
				if overlap < match_threshold and overlap > 0 then
					print(string.format("overlap for [%d,%d]: %f", i, j, overlap))
				end
				]]--
				angle = angle_between( planes[i].eqn:sub(1,3) , planes[j].eqn:sub(1,3) )

				-- Not sure if this residual calculation is justified, probably the correct thing to do 
				-- is to look at each one of the overlap points and evaluate their residuals using the normal
				-- of each plane then see how many overlap ... possibly also justified to to with normal thresholds
				residual = math.abs(torch.dot( planes[i].eqn:sub(1,3), torch.add(planes[i].centroid, -planes[j].centroid) ))
				if overlap > match_threshold then 
					if angle < normal_thresh and residual < residual_thresh then
						match_matrix[{i,j}] = overlap
					else
						print(string.format("angle for [%d,%d]: %f", i, j, angle))
						print(string.format("residual for [%d,%d]: %f", i, j, residual))
					end
				end

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
	planes_rgb = torch.Tensor(3, imgh, imgw):zero()
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

	arc_io:dumpImage( match_matrix, "match_matrices", string.format("%.3d", scan_num))
	arc_io:dumpImage( planes_rgb, "merged_plane_ims", string.format("%.3d", scan_num))

	output = {}
	output.scan_num = scan_num
	output.plane_masks = plane_masks
	output.colormap = cmap
	arc_io:dumpTorch( output, "merged_planes", string.format("%.3d", scan_num))

end










