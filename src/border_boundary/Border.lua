local border_ffi = require './border_ffi'
local path = require "path"
local io = require 'io'
	
Class()

function save_boundary_example()
	local po_dir="/Users/stavbraun/Documents/precise-transit-6548/precise-transit-6548/source/po_scan/a"
	local rgb_dir = path.join(path.dirname(path.dirname(path.dirname(po_dir))), "rgb")
	local name = "001"
	local pc1 = pointcloud.loader.load_pobot_ascii(path.join(po_dir, name))
	local rgb_name = path.join(rgb_dir, "rgb".. name .. ".png")
	local save_result = "boundary.xyz"
	local save_points = "occupied.xyz"
	pc1:load_rgb_map(rgb_name)

	vmsk = pc1:get_valid_masks()
	--vmsk[{{1,100}}]=0

   -- load rgb
   local rgb_map, cmsk = pc1:get_rgb_map()
   local pc_rgb_points,msk = (pc1:get_rgb_list(cmsk)*255):byte():contiguous()
   local xyz_list = (pc1:get_xyz_list(cmsk):clone()/100):int():double():contiguous() --:sub(8310,8320):contiguous()

	a=os.date("%s")
	local center = torch.zeros(1,3):double():contiguous()
	local result = torch.IntTensor()
	local result_rgb = torch.ByteTensor()
   	
   	border_ffi.calculate_boundary(torch.cdata(xyz_list), torch.cdata(center), torch.cdata(pc_rgb_points), torch.cdata(result), torch.cdata(result_rgb))
   	
   	print(os.date("%s")-a)
	local npts = result:size(1)
	local objf = assert(io.open(save_result, "w"))
	for i = 1,npts do
		local pt = result[i]
		local c = result_rgb[i]
		objf:write(string.format("%f %f %f %d %d %d\n",pt[1],pt[2],pt[3], c[1], c[2], c[3]))
	end
	objf:close()

	objf = assert(io.open(save_points, "w"))
	for i = 1,xyz_list:size(1) do
		pt = xyz_list[i]
		objf:write(string.format("%f %f %f\n",pt[1],pt[2],pt[3]))
	end
	objf:close()
	collectgarbage()
   return result
end

