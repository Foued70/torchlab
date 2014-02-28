self = {}
pk = require "/Users/joshua/Documents/GitDepot/cloudlab/src/sandbox/joshua/pancakes.lua"

function self.bundle_norms(pc)

end

function self.totally_visible_planes(planes,pc)
	--this function will find planes where all the borders of the planes are seen with a laser

	plane_stuff = {}

	for i=1,#planes do
		score, edge_list = self.plane_visibility_score(planes[i],pc)
		map = pk.listToMap(edge_list:index(2,torch.LongTensor{2,1}),1)
		plane_stuff[i] = {score,edge_list,map}
	end

	return plane_stuff
end

function self.plane_visibility_score(plane,pc,_occthresh)
	--the score could be a ratio of good pixels to total boundary pixels
	--bad pixels are "in front" of the plane by a specified distance (_occthresh defaults to 100mm), or they're invalid
	--good pixels are not bad pixels

	occthresh = _occthresh or 100

	xyz_map = pc:get_xyz_map()
	depth_map = pc:get_depth_map()
	valid_mask = pc:get_valid_masks()
	edge = self.walkerPixelRanger(plane.mask)
	xyz_edge = torch.zeros(edge:size(1),3)
	depth_edge = torch.zeros(edge:size(1))
	valid_edge = torch.ByteTensor(edge:size(1)):fill(0)
	for i=1,edge:size(1) do
		xyz_edge[i] = xyz_map[{{}, edge[i][1], edge[i][2] }]
		depth_edge[i] = depth_map[ edge[i][1] ][ edge[i][2] ]
		valid_edge[i] = valid_mask[ edge[i][1] ][ edge[i][2] ]
	end

	--for each boundary pixel find the difference between it's actual depth value and the depth value it would have if it were on the plane.
	--the current plane equations work as Nx+D = 0 where N always points "away from" the scanner
	--the point along the xyz ray that intersects the plane is t*xyz where t is a scalar -D/(N.xyz)
	--so if (-D/(N.xyz)-1)*d < 100 mm, that means the point is no more than 100mm in front of where it would be on the plane
	dist_from_edge_to_plane = torch.cmul(
		torch.cdiv(
			-torch.Tensor(depth_edge:size()):fill(plane.eqn[4]),
			torch.mv(xyz_edge,plane.eqn[{{1,3}}]))-1,
		depth_edge)
	num_good = dist_from_edge_to_plane:lt(occthresh).eq(valid_edge):sum()

	return num_good/edge:size(1), edge
end

function self.walkerPixelRanger(_mask)
	--the baddest function in town, he slings a lasso around the first connected component he encounters and returns the pixel coordinates of where his lasso lies
	--it is possible to return the same pixel coordinate up to three times, a "unique_rows" function should be made

	-- initialize a tensor the size of the area of the mask
	local e_ind = 1
	edge = torch.zeros(_mask:sum(),2)
	add_edge = function (pt)
					--print(pt)
					if (type(pt)=="userdata" and pt:numel()==2) or (type(pt)=="table" and #pt==2) then
						if e_ind <= edge:size(1) then
							edge[e_ind][1] = pt[1]
							edge[e_ind][2] = pt[2]
							e_ind = e_ind+1
						else
							error("You ran out of space for your edge list")
						end
					else
						error("Input a 2 element tensor or table")
					end
				end
	
	--find a point on the "edge". By taking the first true pixel in the mask
	mask = _mask:clone():contiguous()
	mask_storage = mask:storage()
	i = 1
	while (i <= #mask_storage) and (mask_storage[i]==0) do--tested that it breaks before checking the second condition if the first condition is false
		i = i+1
	end
	uv = torch.Tensor(2)
	uv[2] = i%mask:stride(1) --horizontal direction 
	uv[1] = math.ceil(i/mask:stride(1)) --vertical direction
	if uv[2]>1 then
		uv[2] = uv[2]-1
		prev_dir = torch.Tensor{1,0} --came from the vertical direction
	elseif uv[1]>1 then
		uv[1] = uv[1]-1
		prev_dir = torch.Tensor{0,1} --came from the horizontal direction
	else
		print("The top-left corner is a 1.  We'll most likely run into problems finding the 0 border pixels around the mask.")
		--some ways to handle this:
		--   1.)make a 0 pixel border around the entire mask and run the program again
	end
	add_edge( uv )

	
	new_half = torch.zeros(2,2) --image coordinates of the pixels in the square that we haven't seen yet
	old_half = torch.zeros(2,2) --image coordinates of the half that we have seen


	repeat--continue the loop until we've reached the beginning
		--look at a 2x2 square where uv is the top left corner.
		--print(mask[{{uv[1],uv[1]+1},{uv[2],uv[2]+1}}])
		square_sum = mask[{{uv[1],uv[1]+1},{uv[2],uv[2]+1}}]:sum()
		new_half[1]:copy(torch.Tensor{ uv[1]+math.max(prev_dir[1],0), uv[2]+math.max(prev_dir[2],0) })
		new_half[2]:copy(torch.Tensor{ uv[1]+math.min(prev_dir[1]+1,1), uv[2]+math.min(prev_dir[2]+1,1) })
		old_half[1]:copy(torch.Tensor{ uv[1]+math.max(-prev_dir[1],0), uv[2]+math.max(-prev_dir[2],0) })
		old_half[2]:copy(torch.Tensor{ uv[1]+math.min(-prev_dir[1]+1,1), uv[2]+math.min(-prev_dir[2]+1,1) })
		if square_sum==1 then
			--add the two pixels from the half you haven't seen yet because they are 0s
			add_edge( new_half[1] )
			add_edge( new_half[2] )
			--change direction toward the pixel with the 1 on your known half (we assume there is one)
			known_uv_adj = mask[{ old_half[1][1], old_half[1][2] }] --the pixel value adjacent to our uv (top-left) pixel on the known half
			prev_dir = (-torch.abs(prev_dir)+1)*(1-2*known_uv_adj)

		elseif square_sum==2 then
			--add the 0 pixel from the half you haven't seen yet (we assume there is one)
			new_uv_adj = mask[{ new_half[1][1], new_half[1][2] }] --the pixel value adjacent to our uv (top-left) pixel on the unknown half
			add_edge( new_half[ new_uv_adj+1] )
			--keep the same direction

		elseif square_sum==3 then
			--don't add anything because the new half are all 1s
			--change direction towards the pixel with the 0 on your known half (we assume there is one)
			known_uv_adj = mask[{ old_half[1][1], old_half[1][2] }] --the pixel value adjacent to our uv (top-left) pixel on the known half
			prev_dir = (-torch.abs(prev_dir)+1)*(2*known_uv_adj-1)
		else
			error("How did we get here? There are 4 pixels in the square!")
		end
		
		uv = uv+prev_dir
	until torch.eq(uv,edge[1]):sum()==2

	return edge[{{1,e_ind-1},{}}]:clone()
end

function laser_density(xyz_vec, normal)
	-- we should be able to tell by the normal of the plane and the distance of the point away from the scanner how dense the laser readings are

end

function self.ptd_to_xyz(phi,theta,_d)
	--phi,theta are in radians
	if (type(phi)=="userdata" or type(phi)=="table") and (#phi==3 or #phi==2) then
		_d = phi[3]
		theta = phi[2]
		phi = phi[1]
		local input_tensor = true
	else
		local input_tensor = false
	end
	local d = _d or 1
	local xyz = torch.Tensor{ math.cos(phi)*math.cos(theta),
		math.cos(phi)*math.sin(theta),
		math.sin(phi) }*d
	if input_tensor then
		return xyz
	else
		return xyz[1],xyz[2],xyz[3]
	end
end

function self.xyz_to_ptd(x, y, z)
	if (type(x)=="userdata" or type(x)=="table") and #x==3 then
		z = x[3]
		y = x[2]
		x = x[1]
		input_tensor = true
	else
		input_tensor = false
	end
	d = math.sqrt(x^2 + y^2 + z^2)
	phi = math.asin(z/d)
	theta = math.atan2(y,x)

	if input_tensor then
		return torch.Tensor{phi,theta,d}
	else
		return phi, theta, d
	end
end

function self.map_to_pt(v, u, meta)
	phi = (meta.h - v)*meta.elevation_per_point + meta.elevation_start + math.pi/2
	theta = (1-v)*meta.azimuth_per_point + (1-u)*meta.azimuth_per_line + math.pi
	return phi, theta
end

function self.pt_to_map(phi, theta, meta)
	v = math.floor((meta.elevation_start + math.pi/2 - phi)/meta.elevation_per_point + 0.5) + meta.h
	u = math.floor(((1-v)*meta.azimuth_per_point - theta + math.pi)/meta.azimuth_per_line) + 1
	return v,u
end

function self.xyz_to_map(x, y, z, meta)
	phi, theta = xyz_to_ptd(x,y,z)
	return ptd_to_map(phi,theta,meta)
end

function self.map_to_xyz(v, u, meta)
	phi, theta = map_to_pt(v, u, meta)
	return ptd_to_xyz({phi,theta})
end

return self