require 'image'

local util = require 'util'
local geom = util.geom
local p3p = require 'p3p'

local r2d = 180 / math.pi
local d2r = math.pi / 180

-- top level filenames

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Compute depth maps')
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

data = dofile("calibration_3_26_12.lua")

for si,sweep in pairs(data) do 
   for li,lens in pairs(sweep) do 
      printf("[%d][%d]", si, li)
      if not lens.white_wall then
         if not lens.world then 
            lens.world = lens.calibration_pairs:narrow(2,1,3)
            -- need homogeneous coords for uv ...
            lens.uv    = torch.ones(lens.world:size(1),3)
            lens.uv:narrow(2,1,2):copy(lens.calibration_pairs:narrow(2,4,2))
            for uvi = 1,lens.uv:size(1) do 
               local n = lens.uv[uvi]:norm()
               lens.uv[uvi]:mul(1/n) -- Unitary vector
            end
         end
         p3p.compute_poses(lens)
      end
   end
end