require 'image'

local util = require 'util'
local p3p = require 'p3p'
local LensSensor = require "util.LensSensor"


-- top level filenames

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Compute Perspective 3 points algorithm')
cmd:text()
cmd:text('Options')
cmd:option('-posefile',
           -- '../data/test/texture_swap/scanner371_job224000_texture_info.txt',
           "../data/models/rivercourt_3307_scan/scanner371_job224000_texture_info.txt",
           'pose info file in same directory as the texture images')
cmd:option('-outdir','output/')
cmd:text()

-- parse input params
params = cmd:parse(arg)

-- load lens calibration
lens = LensSensor.new("nikon_10p5mm_r2t_full")

-- temp load data from ui
data = dofile("calibration_3_26_12.lua")

for si,sweep in pairs(data) do 
   for pi,photo in pairs(sweep) do 
      printf("[%d][%d]", si, pi)
      if not photo.white_wall then
         if not photo.world then 
            photo.world = photo.calibration_pairs:narrow(2,1,3)
            photo.uv = photo.calibration_pairs:narrow(2,4,2)
            photo.angles = lens:img_coords_to_world_angle(photo.uv,"uv")
         end
         p3p.compute_poses(photo.world,photo.angles)
      end
   end
end