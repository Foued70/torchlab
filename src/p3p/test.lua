require 'image'

local util = require 'util'
local geom = util.geom
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
         solutions = p3p.compute_poses(photo.world,photo.angles)
         for ui = 1,4 do 
            s = solutions[ui]
            trans = s[1]
            rot   = s:narrow(1,2,3)
            quat  = geom.rotation_matrix_to_quaternion(rot)
            if quat then 
               printf(" pdiff: %f", torch.sum(data[si][pi].pose_position - trans))
               printf(" trans: %f %f %f",trans[1],trans[2],trans[3])
               printf("rot[1]: %f %f %f", rot[1][1],rot[1][2],rot[1][3])
               printf("  quat: %f %f %f %f",quat[1],quat[2],quat[3],quat[4])
            else
               print("skipping")
            end
         end
      end
   end
end