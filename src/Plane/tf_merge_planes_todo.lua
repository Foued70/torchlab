--[[
	Tf merge planes, a lazy copy of simple_merge_planes so that I don't have to clean this code up yet
	Used to match planes from two scans after transforming them into global coordinates
]]--
require 'gnuplot'
gnuplot.setgnuplotexe("/usr/local/bin/gnuplot")
gnuplot.setterm("x11")

ArcIO = data.ArcIO
specs = data.ArcSpecs
angle_between = geom.util.angle_between

scan0 = 6
scan1 = 7

arc_io = ArcIO.new( specs.job_id, specs.work_id )

scan_ids = specs.scan_ids

-- Transform a plane given a homogenous transformation matrix 
function transform_plane( plane, transform )	

	local normal = torch.cat(plane.eqn:sub(1,3), torch.Tensor({0}))
	local centroid = torch.cat(plane.centroid, torch.Tensor({1}))	

	-- Transform plane normal and centroid
	local tfd_normal = transform*normal
	local tfd_centroid = transform*centroid

	-- Compute new d for plane eqn 
	local d = torch.Tensor({torch.dot( tfd_normal:sub(1,3), tfd_centroid:sub(1,3) )})

	local tfd_plane = {}
	tfd_plane.eqn = torch.cat( tfd_normal:sub(1,3), d)
	tfd_plane.centroid = tfd_centroid:sub(1,3)
	return tfd_plane
end

--[[
scan_start = 6
scan_end = 27
for scan_num = scan_start, scan_end do
]]--
-- for scan_i = 1,#scan_ids do 
	--scan_num = scan_ids[scan_i]

	print("Merging Planes for scan_num: ", scan_num)
	--pc = arc_io:getScan( scan_num ) -- TODO: shouldn't need to do this
	--regions_data = arc_io:loadTorch("region_masks", string.format("%.3d", scan_num))
	regions_data0 = arc_io:loadTorch("region_masks", string.format("%.3d", scan0))
	planes0 = regions_data0.planes

	regions_data1 = arc_io:loadTorch("region_masks", string.format("%.3d", scan1))
	planes1 = regions_data1.planes

	-- Load in transform ... TODO, need a way to access things outside of work_id 
	transform0 = arc_io:loadTorch("transformations", string.format("sweep%.3d", scan0))
	transform1 = arc_io:loadTorch("transformations", string.format("sweep%.3d", scan1))

	print("transform0: ", transform0)
	print("transform1, ", transform1)

	-- Use both normal and residual thresh to keep from incorrectly merging
	normal_thresh = regions_data0.normal_thresh
	residual_thresh = regions_data0.residual_thresh

	--imgh = planes[1].inlier_map:size(1)
	--imgw = planes[1].inlier_map:size(2)

	print("n_planes0: ", #planes0)
	print("n_planes1: ", #planes1)
	-- Plane match matrix, represents the overlap between plane i and plane j 
	match_matrix = torch.Tensor(#planes0,#planes1):zero()

	-- Really shitty match threshold, match_matrix should really hold percentage overlap 
	-- relative to the smaller plane mask 
	--match_threshold = 100


	-- Compute overlaps between all planes 
	print("Computing overlaps between all planes")
	local overlap = 0
	local smaller = 0
	for i=1, #planes0 do
		print(string.format("computing overlaps for plane: %d",i))

		-- Compute transformed parent plane 
		tfd_plane0 = transform_plane( planes0[i], transform0 )

		for j=1, #planes1 do
			--if j ~= i then		
				-- Compute transformed child plane 
				tfd_plane1 = transform_plane( planes1[j], transform1 )

				--angle = angle_between( planes[i].eqn:sub(1,3) , planes[j].eqn:sub(1,3) )
				angle = angle_between( tfd_plane0.eqn:sub(1,3) , tfd_plane1.eqn:sub(1,3) )

				-- Not sure if this residual calculation is justified, probably the correct thing to do 
				-- is to look at each one of the overlap points and evaluate their residuals using the normal
				-- of each plane then see how many overlap ... possibly also justified to to with normal thresholds

				-- Ignore residuals for now 
				--residual = math.abs(torch.dot( planes[i].eqn:sub(1,3), torch.add(planes[i].centroid, -planes[j].centroid) ))
				residual = math.abs(torch.dot( tfd_plane0.eqn:sub(1,3), torch.add(tfd_plane0.centroid, -tfd_plane1.centroid) ))
				--print(string.format("angle between [%d,%d]: %f", i,j,angle))
				if angle < normal_thresh and residual < residual_thresh then				
				--if angle < normal_thresh then 
					match_matrix[{i,j}] = 1
				end
				collectgarbage()
			--end
		end
	end

	match_mask = match_matrix:gt(0.5)

	print("Size of match_matrix: ", match_matrix:size())

	

	output = {}
	output.match_mask = match_mask
	arc_io:dumpTorch( output, "match_masks", string.format("%.3d", scan0))
	
	--[[
	match_masks_data = arc_io:loadTorch("match_masks", string.format("%.3d", scan0))
	match_mask = match_masks_data.match_mask
	]]--

	plane_vrng = torch.range(1,#planes0)
	plane_hrng = torch.range(1,#planes1)	

	print("vrng: ", plane_vrng)
	print("hrng: ", plane_hrng)

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

	-- This merge is different than our dfs in simple merge, we are looking for two 
	-- sets of results, one for scan0 and another for scan1


	--[[
	-- Note this merge is probably inefficient, the for loop can run unnecessarily over 
	-- nodes that have already been merged 
	function merge(ind, ind_set )
		print(ind)
		local h_set = plane_hrng[match_mask[{ind,{}}] ]
		local v_set = plane_vrng[match_mask[{{},ind}] ]
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
	]]--


	-- Given hfront, add elements to vfront for each element in hfront not in hset
	function hmerge( hfront, hset, vset, depth )
		print("hmerge")
		print("hset: ", hset)
		print("vset: ", vset)
		local front = subtract(hfront, hset)		
		print("front: ", front)		
		if front:dim() == 0 then 
			print("no more elements in front")
			return hset, vset 
		end
		local vfront = torch.Tensor({})
		for j=1,front:nElement() do 
			vfront = unique( plane_vrng[match_mask[{{},{j}}]], vset )
		end
		hset = unique(hset, front)
		print("vfront: ", vfront)
		hset, vset = vmerge( vfront, hset, vset, depth )
		return hset, vset 
	end

	-- Given vfront, add elements to hfront for each element in vfront not in vset
	function vmerge( vfront, hset, vset, depth )
		depth = depth + 1			
		print("vmerge")
		print("hset: ", hset)
		print("vset: ", vset)
		local front = subtract(vfront, vset)
		print("front: ", front)		
		if front:dim() == 0 then 
			print("no more elements in front")
			return hset, vset 
		end	
		local hfront = torch.Tensor({})
		for j=1,front:nElement() do 
			hfront = unique( plane_hrng[match_mask[{{j},{}}]], hset )
		end
		vset = unique(vset, front)
		print("hfront: ", hfront)
		hset, vset = hmerge( hfront, hset, vset, depth  )
		return hset, vset
	end


	-- Test merge codes
	hset = torch.Tensor({})
	vset = torch.Tensor({})
	vstart = torch.Tensor({1})


	hset, vset = vmerge( vstart, hset, vset, 1 )
	print("hset: ", hset)
	print("vset: ", vset)



	--[[
	print("Computing Merge Sets")
	merged = nil
	rm_mask = torch.ByteTensor(plane_hrng:nElement()):fill(1)
	plane_sets = {}
	while true do
		if merged == nil then
			to_merge = plane_hrng:clone():long()
		else
			-- TODO: replace with subtract 
			rm_mask[merged:long()] = 0
			to_merge = plane_hrng[rm_mask]
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
	--]]
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
			mask:add( planes[v[j] ].inlier_map )
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

	
	

	--[[
	arc_io:dumpImage( planes_rgb, "merged_plane_ims", string.format("%.3d", scan_num))

	output = {}
	output.scan_num = scan_num
	output.plane_masks = plane_masks
	output.colormap = cmap
	arc_io:dumpTorch( output, "merged_planes", string.format("%.3d", scan_num))
	]]--

	arc_io:dumpImage( match_matrix, "tf_merge_match_matrices", string.format("%.3d", scan0))
--end










