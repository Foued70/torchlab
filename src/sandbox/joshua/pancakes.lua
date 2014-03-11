--[==[ USAGE
> pv = require "/Users/joshua/Documents/GitDepot/cloudlab/src/sandbox/joshua/pancakes.lua"
pointcloud:load_input() - 985.56278398633
> maps = pv.viewPlanesFaceOn()
> image.display(maps[1])
[[ Too many values to print: torch.Tensor.MAX_PRINT == 1000 ]]
[torch.DoubleTensor of dimension 3x3177x2494]

> image.display(maps[2])
[[ Too many values to print: torch.Tensor.MAX_PRINT == 1000 ]]
[torch.DoubleTensor of dimension 3x663x2730]

> I = pv.flattenView(pc:get_xyz_list(),torch.Tensor{1,1,0},10) --view the image when the camera faces in the {1,1,0} direction
> image.display(I)
[[ Too many values to print: torch.Tensor.MAX_PRINT == 1000 ]]
[torch.DoubleTensor of dimension 3x3775x1571]
]==]
self = {}

--local pnt_cloud=pointcloud.loader.load_pobot_ascii("/Users/joshua/Documents/Projects/precise-transit-6548/source/po_scan/a/007")
--local data=torch.load("/Users/joshua/Downloads/toJosh/scan007.t7")


function self.viewPlanesFaceOn(planes,pc,_res)
	--generates the image of each plane viewing in the direction of its normal
	planes = planes or data.planes
	pc = pc or pnt_cloud
	uv_maps = {}
	uv_lists = {}

	for i,p in ipairs(planes) do

		uv_maps[i],uv_lists[i] = self.flattenView(pc:get_xyz_list(p.mask),p.eqn[{{1,3}}],_res)
		
	end

	return uv_maps,uv_lists
end

function self.listToMap(uvlist,_res,_values,_dimOrder)
	--uvlist is an Nx2 Tensor of non-normalized, uncentered image coordinates
	--res is the width of an image pixel in mm (in the same units as uvlist)
	--dimOrder is the order of the vertical and horizontal coordinate in uvlist; defaults to vertical in the 1st column and horizontal in the 2nd column.
	dimOrder = _dimOrder or {1,2}

	--find boundaries are set by the min/max of uvlist 
	--and the resolution is set by the smallest distance between two adjacent points in the list
	res = _res or 1
	uv_min = uvlist:min(1)
	msize = torch.floor((uvlist:max(1)-uv_min)/res + 0.5):reshape(2)+1
	map = torch.zeros(msize[dimOrder[1]],msize[dimOrder[2]]):byte() --the image map

	img_ind = torch.add(uvlist,-uv_min:expandAs(uvlist)):div(res):index(2,torch.LongTensor{dimOrder[1],dimOrder[2]})+1
	self.listOnMap(img_ind,map,_values)

	return map
end

function self.listOnMap(uvlist,map,values)
	--like listToMap, except uvlist is already in image coordinates corresponding to positions in an input map, and this changes the input map by reference
	
	map_line = torch.ByteTensor(map:byte():storage()) --represents the map data in an Nx1 representation
	values = values or 1

	if uvlist then
		lin_ind = torch.mv(torch.floor(uvlist:double()+0.5)-1, --round the coordinates to nearest integer
							torch.Tensor{map:stride(1),map:stride(2)}):long()+1
		map_line[lin_ind] = values
	end
	return map
end

function self.xyzToUV(xyz_list,dir)
	--xyz_list is an Nx3 Tensor of 3D coordinates
	--dir is a 3 element Tensor describing the direction of the camera in {x,y,z}

	--find the u and v axes
	dir:div(dir:norm()) -- direction of the camera is the z_axis
	u_axis = torch.cross(torch.Tensor{0,0,-1},dir) -- u_axis = v_axis x z_axis, where our v_axis(image down) is mostly pointed in the global -z direction
	if u_axis:norm() < math.sin(math.pi/4) then
		u_axis = torch.cross(torch.Tensor{0,1,0},dir) -- our v_axis is mostly pointed in the global y direction
	end
	u_axis:div(u_axis:norm()) -- make it a unit vector
	v_axis = torch.cross(dir,u_axis) -- v_axis = z_axis x u_axis
	uv_axes = torch.cat(v_axis,u_axis,2) --vertical dimension first, then horizontal dimension

	uv_list = torch.mm(xyz_list,uv_axes) 

	return uv_list,uv_axes
end

function self.flattenView(xyz_list,dir,_res)
	
	uv_list,uv_axes = self.xyzToUV(xyz_list,dir)

	return self.listToMap(uv_list,_res),uv_list
end

return self