local path = require 'path'
local pcl = PointCloud.PointCloud
local scan = align_floors_endtoend.Scan
local leafsize = 0.01

local base_dir = '/Users/lihui815/tmp2/Office'

local aScan = scan.new(base_dir)

--aScan:load_and_save_all_sweeps()
--aScan:find_forward_and_backward_transformations()

--[[]]
for a = 0,104 do
	local inname = path.join(base_dir,'OD',''..(1000+a)..'.od')
	--local ouname = path.join(base_dir,'DOWNSAMPLEDXYZ',(1000+a)..'.xyz')
	local xyzname = path.join(base_dir,'DOWNSAMPLEDXYZ','c_xyz_'..(1000+a)..'.xyz')
	local objname = path.join(base_dir,'DOWNSAMPLEDXYZ','ds_obj_'..(1000+a)..'.obj')
	local od = pcl.new(inname)
	od:downsample(leafsize,xyzname,nil)
	--od:write(ouname)
	--od:write(inname)
	--od:save_obj(ouname)
	inname = nil
	ouname = nil
	od = nil
	collectgarbage()
end
--[[]]
