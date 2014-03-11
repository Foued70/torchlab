local ffi    = require 'ffi'

self = {}
pk = require "/Users/joshua/Documents/GitDepot/cloudlab/src/sandbox/joshua/pancakes.lua"
tp = require "/Users/joshua/Documents/GitDepot/cloudlab/src/sandbox/joshua/torch_plus.lua"
io = require "io"

ffi.cdef
[[
	int walker_pixel_ranger(int* edges, unsigned char* mask, int height, int width);
	int walker_pixel_ranger2(int* edges, unsigned char* mask, int height, int width);
	int flood_fill( char *flooded, char *mask, int seed_v, int seed_u, int height, int width );
	int find_first_true(int* seed, char* A,int height,int width);
]]

local libplane_viz   = util.ffi.load('libplane_tools')

--quick timer table to keep multiple timers running
self.JT = {}
self.JT.enabled = false
self.JT.create = function (name)
	if self.JT.enabled then
		if self.JT[name] then
			self.JT[name]:reset()
		else
			self.JT[name] = torch.Timer()
		end
	end
end

self.JT.lap = function (name)
	if self.JT.enabled then
		io.write(name," ",self.JT[name]:time().real,"s\n")
	end
end

self.JT.stop = function (name)
	if self.JT.enabled then
		self.JT.lap(name)
		self.JT[name]:stop()
	end
end

function self.totally_visible_planes(planes,pc,verbose)
	--this function will find planes where all the borders of the planes are seen with a laser
	self.JT.create("totally_visible_planes")
	local plane_stuff = {}
	verbose = verbose or false
	for i=1,#planes do
		if verbose then
			io.write('plane_score #',i,'\n')
		end
		local viz_score, abp_mask = self.plane_visibility_score(planes[i],pc)
		plane_stuff[i] = {score = viz_score, edge_mask = abp_mask}
	end
	self.JT.lap("totally_visible_planes")
	return plane_stuff
end

function self.plane_visibility_score(plane,pc,_occthresh)
	--the score could be a ratio of good pixels to total boundary pixels
	--bad pixels are "in front" of the plane by a specified distance (_occthresh defaults to 100mm), or are invalid
	--good pixels are not bad pixels
	self.JT.create("plane_visibility_score")
	local visibility_score
	local occthresh = _occthresh or 100

	local valid_mask = pc:get_valid_masks()
	local abp_list, abp_mask = self.get_plane_boundary_pixels(plane.mask) --all boundary pixels

	local vbp_mask = tp.AND(abp_mask,valid_mask) --valid boundary pixels

	if tp.ANY(vbp_mask) then
		local xyz_edge = pc:get_xyz_list(vbp_mask)
		local depth_edge = pc:get_depth_list(vbp_mask)

		--for each boundary pixel find the difference between it's actual depth value and the depth value it would have if it were on the plane.
		--the current plane equations work as Nx+D = 0 where N always points "away from" the scanner, so the D's are always negative.
		--the point along the xyz ray that intersects the plane is t*xyz where t is a scalar -D/(N.xyz)
		--so if (-D/(N.xyz)-1)*d < 100 mm, that means the point is no more than 100mm in front of where it would be on the plane
		local dist_from_edge_to_plane = torch.cmul(
			torch.cdiv(
				-torch.Tensor(depth_edge:size()):fill(plane.eqn[4]),
				torch.mv(xyz_edge,plane.eqn[{{1,3}}]))-1,
			depth_edge)
		local good_edges = dist_from_edge_to_plane:lt(occthresh)
		local num_good = good_edges:sum()

		visibility_score = num_good/abp_list:size(1)
	else
		visibility_score = 0
	end
	self.JT.lap("plane_visibility_score")
	return visibility_score, abp_mask
end

function self.get_plane_boundary_pixels(mask,_areathresh)
	--gets all the boundary pixels of the connected components in mask
	self.JT.create("get_plane_boundary_pixels")
	areathresh = _areathresh or 100
	local all_boundary_pixels
	local mask_clone = mask:clone()
	local itr = 1
	while tp.ANY(mask_clone) do
		itr = itr+1

		mask_clone,CC = self.lassoNextConnComp(mask_clone)

		if CC:sum() > areathresh then 
			local E, v = self.walkerPixelRanger(CC)

			local edge_list = E:index(1,torch.range(1,v:size(1)):long()[v:squeeze()])
			--local edge_list = torch.cat(E:t()[1][v],E:t()[2][v],2) --only the valid edge_list

			if all_boundary_pixels then
				all_boundary_pixels = torch.cat(all_boundary_pixels,edge_list,1)
			else
				all_boundary_pixels = edge_list
			end
		end
		if mask_clone:sum() < areathresh then
			break
		end
	end
	local abp_mask = pk.listOnMap(all_boundary_pixels,torch.zeros(mask:size()):byte())
	self.JT.lap("get_plane_boundary_pixels")
	return all_boundary_pixels, abp_mask
end

function self.walkerPixelRanger(_mask)
	--the baddest function in town, he slings a lasso around the first connected component he encounters and returns the pixel coordinates of where his lasso lies
	--it is possible to return the same pixel coordinate up to three times, a "unique_rows" function should be made, DONE
	self.JT.create("walkerPixelRanger")
	-- initialize a tensor the size of the area of the mask
	-- local e_ind = 1
	local edge = torch.zeros(2*_mask:sum()+2,2):int()
	-- local add_edge = function (pt)
	-- 				--this is just a cheap way so we don't have to keep track of the last edge index (e_ind)
	-- 				if (type(pt)=="userdata" and pt:numel()==2) or (type(pt)=="table" and #pt==2) then
	-- 					if e_ind <= edge:size(1) then
	-- 						edge[e_ind][1] = pt[1]
	-- 						edge[e_ind][2] = pt[2]
	-- 						e_ind = e_ind+1
	-- 					else
	-- 						error("You ran out of space for your edge list")
	-- 					end
	-- 				else
	-- 					error("Input a 2 element tensor or table")
	-- 				end
	-- 			end
	
	--pad the mask with 0s in case the mask touches the image borders
	local mask = torch.zeros(_mask:size(1)+2,_mask:size(2)+2):typeAs(_mask)
	mask[{{2,mask:size(1)-1},{2,mask:size(2)-1}}]:copy(_mask)

	-- --find a point on the "edge". By taking the first true pixel in the mask, or the first pixel different than the seed pixel
	-- --We'll be looking at a 2x2 square where uv is the top left corner.
	-- --initialize the uv coordinates just to the left of the first true pixel
	-- local uv = torch.Tensor(self.findFirstTrue(mask))
	-- uv[2] = uv[2]-1 --start just before the first true pixel
	-- add_edge( uv ) --this will be the first pixel of the edge

	-- --initialize a tensor to represent the direction (in uv coordinates) we just came from
	-- local prev_dir = torch.Tensor{1,0} --came from the vertical direction

	-- -- --initialize tensors to represent the pixel coordinates of two halves of the search square
	-- -- new_half = torch.zeros(2,2) --image coordinates of the pixels in the square that we haven't seen yet
	-- -- old_half = torch.zeros(2,2) --image coordinates of the half that we have seen


	-- repeat--continue the loop until we've reached the beginning
	-- 	--print(mask[{{uv[1],uv[1]+1},{uv[2],uv[2]+1}}])
	-- 	local square_sum = mask[{{uv[1],uv[1]+1},{uv[2],uv[2]+1}}]:sum()
	-- 	local new_half = { { uv[1]+math.max(prev_dir[1],0), uv[2]+math.max(prev_dir[2],0) },  --adjacent to uv
	-- 				 { uv[1]+math.min(prev_dir[1]+1,1), uv[2]+math.min(prev_dir[2]+1,1) } }
	-- 	local old_half = { { uv[1]+math.max(-prev_dir[1],0), uv[2]+math.max(-prev_dir[2],0) }, --adjacent to uv
	-- 				 { uv[1]+math.min(-prev_dir[1]+1,1), uv[2]+math.min(-prev_dir[2]+1,1) } }
	-- 	if square_sum==1 then
	-- 		--add the two pixels from the half you haven't seen yet because they are 0s
	-- 		-- add_edge( new_half[1] )
	-- 		-- add_edge( new_half[2] )
	-- 		local old_uv_adj = mask[old_half[1]] --the pixel value adjacent to our uv (top-left) pixel on the known half
	-- 		add_edge( new_half[ 2-old_uv_adj ] )
	-- 		--change direction toward the pixel with the 1 on your known half (we assume there is one)
	-- 		--known_uv_adj = mask[{ old_half[1][1], old_half[1][2] }] --the pixel value adjacent to our uv (top-left) pixel on the known half
	-- 		prev_dir = (-torch.abs(prev_dir)+1)*(1-2*old_uv_adj)

	-- 	elseif square_sum==2 then
	-- 		--Two scenarios: the true pixels are adjacent and the false pixels are adjacent, OR it makes a "checkered" pattern
	-- 		if mask[new_half[1]] == mask[old_half[1]] then 
	-- 			--add the 0 pixel from the half you haven't seen yet (we assume there is one)
	-- 			local new_uv_adj = mask[new_half[1]] --the pixel value adjacent to our uv (top-left) pixel on the unknown half
	-- 			add_edge( new_half[ new_uv_adj+1] )
	-- 			--keep the same direction
	-- 		else --"checkered" pattern.  
	-- 			--Treat it like a square_sum=3 scenario
	-- 			--don't add anything
	-- 			--change direction towards the pixel with the 0 on your known half
	-- 			local old_uv_adj = mask[old_half[1]] --the pixel value adjacent to our uv (top-left) pixel on the known half
	-- 			prev_dir = (-torch.abs(prev_dir)+1)*(2*old_uv_adj-1)

	-- 			-- --Treat it like a square_sum=1 scenario
	-- 			-- --add the pixel from new half which is adjacent to the true pixel in the old half
	-- 			-- old_uv_adj = mask[old_half[1]] --the pixel value adjacent to our uv (top-left) pixel on the known half
	-- 			-- add_edge( new_half[ 2-old_uv_adj ] )
	-- 			-- --change direction toward the pixel with the 1 on your known half (we assume there is one)
	-- 			-- prev_dir = (-torch.abs(prev_dir)+1)*(1-2*old_uv_adj)
	-- 		end
	-- 	elseif square_sum==3 then
	-- 		--don't add anything because the new half are all 1s
	-- 		--change direction towards the pixel with the 0 on your known half (we assume there is one)
	-- 		local old_uv_adj = mask[old_half[1]] --the pixel value adjacent to our uv (top-left) pixel on the known half
	-- 		prev_dir = (-torch.abs(prev_dir)+1)*(2*old_uv_adj-1)
	-- 	else
	-- 		error("How did we get here? There are 4 pixels in the square!")
	-- 	end
		
	-- 	uv = uv+prev_dir
	-- until torch.eq(uv,edge[1]):sum()==2 --repeat
	edge = edge:contiguous()
	mask = mask:contiguous()
	num_edges = libplane_viz.walker_pixel_ranger(torch.data(edge),torch.data(mask),mask:size(1),mask:size(2))

	local edge_list = edge[{{1,num_edges},{}}]:clone() --no need to decrement the coordinates by one to compensate for the 0 padded border, because the Cpp indexing starts at 0
	--edge_list = tp.unique_rows(edge_list) --it's possible that a pixel was added more than once, this function maintains the same order but removes non-unique rows if it's seen them before
	local valid_list = edge_list:ge(1):add(edge_list:le(torch.IntTensor{{_mask:size(1),_mask:size(2)}}:expandAs(edge_list))):sum(2):eq(4) --each row must satisfy 4 conditions: 1 <= u <= width and 1 <= v <= height
	self.JT.lap("walkerPixelRanger")
	return edge_list, valid_list
end

function self.lassoNextConnComp(map)
	self.JT.create("lassoNextConnComp")
	local map_clone = map:clone()
	local flooded, flipped

	seed = self.findFirstTrue(map_clone)

	if tp.ALL(seed:gt(0)) then
		flooded = self.floodFill(seed,map_clone)
		flipped = torch.ne(map_clone,flooded)
	end
	self.JT.lap("lassoNextConnComp")
	return flipped,flooded
end

function self.areaInContour(uv_list)
	--uses Green's theorem to calculate area of closed contour
	self.JT.create("areaInContour")
	local X = uv_list[{{1,-2},1}]
	local Y = uv_list[{{1,-2},2}]
	local dX = torch.add(uv_list[{{2,-1},1}],-uv_list[{{1,-2},1}])
	local dY = torch.add(uv_list[{{2,-1},2}],-uv_list[{{1,-2},2}])
	local area = (torch.cmul(X,dY) + torch.cmul(Y,dX))*0.5
	self.JT.lap("areaInContour")
	return area:sum()
end

function self.floodFill(seed,mask)
	self.JT.create("floodFill")
	--seed is the starting position
	--3.4 seconds in Lua; 0.005 seconds in C++
	local flooded = torch.ByteTensor(mask:size()):fill(0)
	libplane_viz.flood_fill(torch.data(flooded),torch.data(mask),seed[1],seed[2],mask:size(1),mask:size(2))
	self.JT.lap("floodFill")
	return flooded
end

function self.findFirstTrue(A)
	self.JT.create("findFirstTrue")
	-- for v = 1,A:size(1) do
	-- 	for u = 1,A:size(2) do
	-- 		if A[v][u] > 0 then
	-- 			self.JT.lap("findFirstTrue")
	-- 			return {v,u}
	-- 		end
	-- 	end
	-- end
	seed = torch.IntTensor(2):fill(-1)
	libplane_viz.find_first_true(torch.data(seed),torch.data(A),A:size(1),A:size(2))
	self.JT.lap("findFirstTrue")
	return seed+1
end

function laser_density(xyz_vec, normal)
	-- we should be able to tell by the normal of the plane and the distance of the point away from the scanner how dense the laser readings are

end

function self.verify_planes(planes,xyz_map,verbose)
	--this is basically a function to apply the get_pixel_offsets_from_plane function to all the planes in the table "planes"
	--xyz_map is a map of all the 3D positions of every pixel
	verbose = verbose or false
	local correctness = {}

	for i,p in ipairs(planes) do
		if verbose then
			io.write("verify plane# ",i,"\n")
		end
		correctness[i] = self.get_pixel_offsets_from_plane(p,xyz_map)
	end
	return function (ind,verbose)
			--[==[ USAGE
				> C = pv.verify_planes(planes,pc:get_xyz_map())
				> C(1)
				stdv=	4.487223889121
				mean=	1.0098173892399e-11
				min=	-95.514856876031
				max=	46.733520166387
				[[ Too many values to print: torch.Tensor.MAX_PRINT == 1000 ]]
				[torch.DoubleTensor of dimension 630x880]

				> C({1,6})
				stdv=	5.7332889976337
				mean=	9.2788149714601e-12
				min=	-95.514856876031
				max=	73.348317403277
				[[ Too many values to print: torch.Tensor.MAX_PRINT == 1000 ]]
				[torch.DoubleTensor of dimension 630x880]

				> image.display(C{1,61})
				stdv=	7.0361046392574
				mean=	7.9095214008346e-12
				min=	-141.64818813362
				max=	91.148341509167
				[[ Too many values to print: torch.Tensor.MAX_PRINT == 1000 ]]
				[torch.DoubleTensor of dimension 3x630x880]
				]==]
				verbose = verbose or false
				if type(ind)=="table" and #ind==2 then
					result = torch.zeros(correctness[1]:size())
					for i=ind[1],ind[2] do
						result = result:add(correctness[i])
					end
				elseif type(ind)=="number" then
					result = correctness[ind]
				else
					print("enter index as a number or a table range {start,end}")
					return nil
				end
				if verbose then
					print("stdv=",torch.std(result))
					print("mean=",torch.mean(result))
					print("min=",result:min())
					print("max=",result:max())
				end
				return result
			end
end

function self.get_pixel_offsets_from_plane(plane,xyz_map)
	--plane has fields "mask" and "eqn"
	--plane.mask is a byteTensor mask which matches the height and width of the xyz_map
	--plane.eqn is a 4 element vector which defines the plane equation --Nx+D=0, where N always points "away from" the scanner, so the D's are always negative.
	pixel_offset_map = torch.sum(torch.cmul(xyz_map,plane.eqn[{{1,3}}]:reshape(3,1,1):expandAs(xyz_map)),1):reshape(plane.mask:size()):add(torch.Tensor(plane.mask:size()):fill(plane.eqn[4])):cmul(plane.mask:double())
	return pixel_offset_map
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