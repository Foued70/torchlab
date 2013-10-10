local path = require 'path'
local pcl = PointCloud.PointCloud
local scan = align_floors_endtoend.Scan
local leafsize = 0.01

--[[
local base_dir = '/Users/lihui815/cloudlab/build/usr/local/tmp/arcs/virtuous-walk-1066/work/a_03'
local complete_loop = false
local start = 70
local total = 90
--[[]]
--[[]]
local base_dir = '/Users/lihui815/tmp2/Office'
local complete_loop = true
local start = 1000
local total = 1104
--[[]]

local test = Class()

function show(i,forward)
  local swp1
  local swp2
  local swpPair
  if forward then
    swp1 = align_floors_endtoend.Sweep.new(base_dir,''..(start+i-1))
    swp2 = align_floors_endtoend.Sweep.new(base_dir,''..(start+i))
    swpPair = align_floors_endtoend.SweepPair.new(base_dir,swp1,swp2,i,forward)
  else
    if start+i == total then
      swp1 = align_floors_endtoend.Sweep.new(base_dir,''..start)
    else
      swp1 = align_floors_endtoend.Sweep.new(base_dir,''..(start+i+1))
    end
    swp2 = align_floors_endtoend.Sweep.new(base_dir,''..(start+i))
    swpPair = align_floors_endtoend.SweepPair.new(base_dir,swp1,swp2,(total-start)-i+1,forward)
  end
  swpPair:displayResult()
end

function do_loop()
  local aScan = scan.new(base_dir,complete_loop)
  aScan:find_forward_and_backward_transformations()
end



