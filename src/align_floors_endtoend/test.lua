local path = require 'path'
local pcl = PointCloud.PointCloud
local scan = align_floors_endtoend.Scan
local leafsize = 0.01

--local base_dir = '/Users/lihui815/cloudlab/build/usr/local/tmp/arcs/virtuous-walk-1066/work/a_02'
local base_dir = '/Users/lihui815/tmp2/Office'
local complete_loop = true

local aScan = scan.new(base_dir,complete_loop)

aScan:load_and_save_all_sweeps()
aScan:find_forward_and_backward_transformations()

--[[
for a = 0,104 do
	local inname = path.join(base_dir,'OD',''..(1000+a)..'.od')
	local xyzname = path.join(base_dir,'DOWNSAMPLEDXYZ',(1000+a)..'.xyz')
	local od = pcl.new(inname)
	od:save_downsampled_global_to_xyz(leafsize,xyzname)
	od = nil
	collectgarbage()
end
--[[]]
