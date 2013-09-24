local path = require 'path'
local pcl = PointCloud.PointCloud
local scan = align_floors_endtoend.Scan

local base_dir = '/Users/lihui815/tmp2/Office'

local aScan = scan.new(base_dir)

--aScan:load_and_save_all_sweeps()
aScan:find_forward_and_backward_transformations()
