require 'sys'
require 'torch'
require 'math'
require 'image'
require 'paths'
local geom = require "util/geom"

local pose = {}

local r2d = 180 / math.pi
local d2r = math.pi / 180

-- From: Florence Shaffer <florence@matterport.com>
-- Date: Tue, Sep 18, 2012 at 3:53 PM
-- The fields that make up each line are:

--     <texture filename> <qx> <qy> <qz> <qw> <tx> <ty> <tz> <center u>
--     <center v> <degrees per px x> <degrees per px y>

-- The first seven numbers define a transform from local sweep
-- coordinates to the global coordinates used for vertices in the .obj.
-- In the local sweep coordinates, the center of the texture is what
-- you'd see looking along the +x axis, and the +z axis is up (towards
-- the top of the texture).  The quaternion defined by the coefficients
-- (qx, qy, qz, qw) is a rotation from local sweep coordinates to
-- global coordinates, and the position (tx, ty, tz) is the origin of
-- the sweep in global coordinates.

-- The next two numbers define the uv coordinates of the point you
-- should treat as the center of the texture.  The u will always be
-- 0.5, but the v will vary a little bit due to how we crop the
-- texture.

-- The final two numbers are change in degrees per pixel as you move
-- around in the image horizintally (x) or vertically (y).

function pose.loadtxtfile(posefile)
   sys.tic()
   if (not posefile) then
      dok.error("must pass posefile")
   end
   pasedir  = paths.dirname(posefile)

   local poses = {}
   poses.images = {}
   poses.nposes =
      tonumber(io.popen(string.format("grep -c '.' %s",posefile)):read())
   print(string.format("Found %d poses in %2.2fs", poses.nposes, sys.toc()))
   sys.tic()
   poses.data = torch.Tensor(poses.nposes,19)

   local pf = io.open(posefile)
   local pc = 1
   for pl in pf:lines() do
      local n,pd = pl:match("^([%a%d_.]+) (.*)")
      poses[pc] = n
      poses.images[pc] =
         image.load(string.format("%s/%s", paths.dirname(posefile), n))
      poses.data[pc][18] = poses.images[pc]:size(3) -- width
      poses.data[pc][19] = poses.images[pc]:size(2) -- height

      local k = 1
      for n in pd:gmatch("[-.%d]+") do
         poses.data[pc][k] = tonumber(n)
         k = k + 1
      end
      poses.data[pc][16] = poses.data[pc][18] *    poses.data[pc][8]
      poses.data[pc][17] = poses.data[pc][19] * (1-poses.data[pc][9])
      pc = pc + 1
   end
   pose.process(poses)
   print(string.format("Loaded %d poses in %2.2fs", pc-1, sys.toc()))
   return poses
end

function pose.loaddata(data)
   sys.tic()
   local poses = {}
   poses.nposes = data:size(1)
   poses.data = data
   pose.process(poses)
   print(string.format("Loaded %d poses in %2.2fs", poses.nposes, sys.toc()))
   return poses
end

function pose.process(poses)
   poses.quat  = poses.data:narrow(2,1,4)
   poses.xyz   = poses.data:narrow(2,5,3)
   poses.uv    = poses.data:narrow(2,8,2)
   poses.px    = poses.data:narrow(2,10,2)
   poses.quat_r = poses.data:narrow(2,12,4)
   poses.quat_r:copy(poses.quat)
   poses.quat_r:narrow(2,1,3):mul(-1)
   poses.cntrx = poses.data:select(2,16)
   poses.cntry = poses.data:select(2,17)
   poses.w     = poses.data:select(2,18)
   poses.h     = poses.data:select(2,19)
   return poses
end

-- FIXME make a class
function pose.global2local(p,i,v)
   return geom.rotate_by_quat(v - p.xyz[i],p.quat_r[i])
end

function pose.local2global(p,i,v)
   return geom.rotate_by_quat(v,p.quat[i]) + p.xyz[i]
end

function pose.globalxyz2uv(p,i,pt)
   -- xyz in pose coordinates
   local v = pose.global2local(p,i,pt)
   local azimuth = - r2d * torch.atan2(v[2],v[1])
   local norm      = geom.normalize(v)
   local elevation = r2d * torch.asin(norm[3])
   -- print(azimuth,elevation)
   local proj_x  = p.cntrx[i] + (  azimuth / p.px[i][1])
   local proj_y  = p.cntry[i] - (elevation / p.px[i][2])
   local proj_u  = proj_x / p.w[i]
   local proj_v  = 1 - (proj_y / p.h[i])
   return proj_u, proj_v, proj_x, proj_y
end

-- for pixel xy (0,0 in top left, w,h in bottom right) to point +
-- direction ray for intersection work
function pose.localxy2globalray(p,i,x,y)
   local azimuth   = (x - p.cntrx[i]) * p.px[i][1]
   local elevation = (p.cntry[i] - y) * p.px[i][2]
   -- print(azimuth,elevation)
   azimuth   = - d2r * azimuth
   elevation = d2r * elevation
   local dir = torch.Tensor(3)
   -- local direction
   local h =       torch.cos(elevation)
   dir[3]  =       torch.sin(elevation) -- z'
   dir[2]  =   h * torch.sin(azimuth)   -- y'
   dir[1]  =   h * torch.cos(azimuth)   -- x'
   dir = geom.normalize(dir)
   -- return point and direction rotated to global coordiante
   return p.xyz[i], geom.rotate_by_quat(dir,p.quat[i])
end


return pose
