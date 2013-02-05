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

-- chose not to have a class for the easy access.
function Poses:newPose(i)
   self[i]     = {}
   Pose        = self[i]
   Pose.quat   = self.quat[i]
   Pose.xyz    = self.xyz[i]
   Pose.uv     = self.uv[i]
   Pose.px     = self.px[i]
   Pose.quat_r = self.quat_r[i]
   Pose.cntrx  = self.cntrx[i]
   Pose.cntry  = self.cntry[i]
   Pose.w      = self.w[i]
   Pose.h      = self.h[i]
   return self[i]
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
   for i = 1,self.nposes do
      self:newPose(i)
   end
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

-- + FIXME improve speed : creation with a simple increment if possible. (SLERP)

function Poses:compute_dirs(i,scale)

   if not i then 
      print("Error must pass pose_id")
      return nil
   end
   if not scale then 
      scale = 1
   end
   printf("Computing dirs for pose[%d] at scale 1/%d",i,scale)

   local imgw = self[i].w
   local imgh = self[i].h

   local outw = math.ceil(imgw/scale)
   local outh = math.ceil(imgh/scale)

   -- dirs are 2D x 3
   local dirs = torch.Tensor(outh,outw,3)
   local cnt = 1
   local inh = 0
   local inw = 0
   for h = 1,outh do
      for w = 1,outw do
         local _,dir = self:localxy2globalray(i,inw,inh)
         dirs[h][w]:copy(dir:narrow(1,1,3))
         cnt = cnt + 1
         inw = inw + scale
      end
      inh = inh + scale
      inw = 0
   end
   if not self.dirs then 
      self.dirs = {}
   end
   if not self.dirs[i] then
      self.dirs[i] = {}
   end
   self.dirs[i][scale] = dirs
   if not self[i].dirs then
      self[i].dirs = {}
   end
   self[i].dirs[scale] = self.dirs[i][scale]
   return self[i].dirs[scale]
end

-- 
-- Caching
-- 
-- isdependant on pose rotation as well as image width, height, scale and center
function Poses:load_dirs(cachedir,i,scale,ps)

   local imgw = self[i].w
   local imgh = self[i].h

   local outw = math.ceil(imgw/scale)
   local outh = math.ceil(imgh/scale)

   local cntrx = self[i].cntrx
   local cntry = self[i].cntry

   local dirscache   = 
      string.format("%s/pose_rot_%f_%f_%f_%f_w_%d_h_%d_s_%d_cx_%d_cy_%f",
                    cachedir,
                    self[i].quat[1],self[i].quat[2],self[i].quat[3],self[i].quat[4],
                    imgw,imgh,scale,cntrx,cntry)
   
   if ps then 
      dirscache = dirscache .."_-_grid_".. ps
   end
   dirscache = dirscache ..".t7"

   local dirs = nil
   if paths.filep(dirscache) then
      sys.tic()
      dirs = torch.load(dirscache)
      printf("Loaded dirs from %s in %2.2fs", dirscache, sys.toc())
   else
      sys.tic()
      if ps then 
         dirs = util.grid_contiguous(self:compute_dirs(i,scale),ps,ps)
      else
         dirs = self:compute_dirs(i,scale)
      end
      printf("Built dirs in %2.2fs", sys.toc())
      torch.save(dirscache,dirs)
      printf("Saving dirs to %s", dirscache)
   end
   return dirs
end

-- is unique for each scale and pose.
function Poses:load_depth(cachedir,i,scale,ps)

   local imgw = self[i].w
   local imgh = self[i].h

   local outw = math.ceil(imgw/scale)
   local outh = math.ceil(imgh/scale)

   local cntrx = self[i].cntrx
   local cntry = self[i].cntry

   local depthcache   = cachedir .. 
      "orig_"..imgw.."x"..imgh.."_-_"..
      "scaled_"..outw.."x"..outh.."_-_"..
      "center_"..cntrx.."x"..cntry

   dirscache = dirscache ..".t7"

   local depthmap = nil
   if paths.filep(dirscache) then
      sys.tic()
      depthmap = torch.load(dirscache)
      printf("Loaded depths from %s in %2.2fs", posecache, sys.toc())
      if not self[i].depthmap then
         self[i].depthmap = {}
      end
      self[i].depthmap[scale] = depthmap
   else
      printf("No pose found")
   end
end


return Poses
