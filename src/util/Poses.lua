require 'sys'
require 'torch'
require 'math'
require 'image'
require 'paths'

local geom = require "util/geom"

local r2d = 180 / math.pi
local d2r = math.pi / 180

-- this is container class for all the poses
local Poses = torch.class('Poses')


--
-- Email containing information about the pose file from matterport
--
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

function Poses:__init(posefile)
   if (posefile) then
      sys.tic()
      local posedir = paths.dirname(posefile)
      self.images = {}
      self.names = {}
      self.nposes =
         tonumber(io.popen(string.format("grep -c '.' %s",posefile)):read())
      print(string.format("Found %d poses in %2.2fs", self.nposes, sys.toc()))
      sys.tic()
      self.data = torch.Tensor(self.nposes,19)

      local pf = io.open(posefile)
      local pc = 1
      for pl in pf:lines() do
         local n,pd = pl:match("^([%a%d_.]+) (.*)")
         self.names[pc]     = n
         self.images[pc]    = image.load(string.format("%s/%s", posedir, n))
         self.data[pc][18] = self.images[pc]:size(3) -- width
         self.data[pc][19] = self.images[pc]:size(2) -- height

         local k = 1
         for n in pd:gmatch("[-.%d]+") do
            self.data[pc][k] = tonumber(n)
            k = k + 1
         end
         self.data[pc][16] = self.data[pc][18] *    self.data[pc][8]
         self.data[pc][17] = self.data[pc][19] * (1-self.data[pc][9])
         pc = pc + 1
      end
      self:process()
      print(string.format("Loaded %d poses in %2.2fs", pc-1, sys.toc()))
   end
end

function Poses:loaddata(data)
   sys.tic()
   self.nposes = data:size(1)
   self.data   = data
   self:process()
   print(string.format("Loaded %d poses in %2.2fs", self.nposes, sys.toc()))
end

function Poses:process()
   self.quat  = self.data:narrow(2,1,4)
   self.xyz   = self.data:narrow(2,5,3)
   self.uv    = self.data:narrow(2,8,2)
   self.px    = self.data:narrow(2,10,2)
   self.quat_r = self.data:narrow(2,12,4)
   self.quat_r:copy(self.quat)
   self.quat_r:narrow(2,1,3):mul(-1)
   self.cntrx = self.data:select(2,16)
   self.cntry = self.data:select(2,17)
   self.w     = self.data:select(2,18)
   self.h     = self.data:select(2,19)
end

function Poses:global2local(i,v)
   return geom.rotate_by_quat(v - self.xyz[i],self.quat_r[i])
end

function Poses:local2global(i,v)
   return geom.rotate_by_quat(v,self.quat[i]) + self.xyz[i]
end

function Poses:globalxyz2uv(i,pt)
   -- xyz in pose coordinates
   local v       = self:global2local(i,pt)
   local azimuth = -r2d * torch.atan2(v[2],v[1])
   local norm    = geom.normalize(v)
   local elevation = r2d * torch.asin(norm[3])
   local proj_x  = 0.5 + self.cntrx[i] + (  azimuth / self.px[i][1])
   local proj_y  = 0.5 + self.cntry[i] - (elevation / self.px[i][2])
   local proj_u  = proj_x / self.w[i]
   local proj_v  = 1 - (proj_y / self.h[i])
   return proj_u, proj_v, proj_x, proj_y
end

-- for pixel xy (0,0 in top left, w,h in bottom right) to point +
-- direction ray for intersection work
function Poses:localxy2globalray(i,x,y)
   local azimuth   = (x - self.cntrx[i]) * self.px[i][1]
   local elevation = (self.cntry[i] - y) * self.px[i][2]
   -- print(azimuth,elevation)
   azimuth   = - d2r * azimuth
   elevation = d2r * elevation
   local dir = torch.Tensor(3)
   -- local direction
   local h =       torch.cos(elevation)
   dir[3]  =       torch.sin(elevation) -- z'
   dir[2]  =   h * torch.sin(azimuth)   -- y'
   dir[1]  =   h * torch.cos(azimuth)   -- x'
   dir     = geom.normalize(dir)
   -- return point and direction rotated to global coordiante
   return self.xyz[i], geom.rotate_by_quat(dir,self.quat[i])
end


return Poses
