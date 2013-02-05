local geom = require "util/geom"

local r2d = 180 / math.pi
local d2r = math.pi / 180

local Pose = torch.class('Pose')

-- chose not to have a class for the easy access.
function Pose:__init(poses,i)
   self.pid    = i
   self.name   = poses.names[i]
   self.quat   = poses.quat[i]
   self.xyz    = poses.xyz[i]
   self.uv     = poses.uv[i]
   self.px     = poses.px[i]
   self.quat_r = poses.quat_r[i]
   self.cntrx  = poses.cntrx[i]
   self.cntry  = poses.cntry[i]
   self.w      = poses.w[i]
   self.h      = poses.h[i]
end

function Pose:global2local(v)
   return geom.rotate_by_quat(v - self.xyz,self.quat_r)
end

function Pose:local2global(v)
   return geom.rotate_by_quat(v,self.quat) + self.xyz
end

function Pose:globalxyz2uv(pt)
   -- xyz in pose coordinates
   local v       = self:global2local(pt)
   local azimuth = -r2d * torch.atan2(v[2],v[1])
   local norm    = geom.normalize(v)
   local elevation = r2d * torch.asin(norm[3])
   local proj_x  = 0.5 + self.cntrx + (  azimuth / self.px[1])
   local proj_y  = 0.5 + self.cntry - (elevation / self.px[2])
   local proj_u  = proj_x / self.w
   local proj_v  = 1 - (proj_y / self.h)
   return proj_u, proj_v, proj_x, proj_y
end

-- for pixel xy (0,0 in top left, w,h in bottom right) to point +
-- direction ray for intersection work
function Pose:localxy2globalray(x,y)
   local azimuth   = (x - self.cntrx) * self.px[1]
   local elevation = (self.cntry - y) * self.px[2]
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
   return self.xyz, geom.rotate_by_quat(dir,self.quat)
end

-- + FIXME improve speed : creation with a simple increment if possible. (SLERP)

function Pose:compute_dirs(scale)

   if not scale then 
      scale = 1
   end
   printf("Computing dirs for pose[%d] at scale 1/%d",self.pid,scale)

   local imgw = self.w
   local imgh = self.h

   local outw = math.ceil(imgw/scale)
   local outh = math.ceil(imgh/scale)

   -- dirs are 2D x 3
   local dirs = torch.Tensor(outh,outw,3)
   local cnt = 1
   local inh = 0
   local inw = 0
   for h = 1,outh do
      for w = 1,outw do
         local _,dir = self:localxy2globalray(inw,inh)
         dirs[h][w]:copy(dir:narrow(1,1,3))
         cnt = cnt + 1
         inw = inw + scale
      end
      inh = inh + scale
      inw = 0
   end
   return self:store_dirs(dirs,scale)
end

function Pose:store_dirs(dirs,scale)
   if not self.dirs then 
      self.dirs = {}
   end
   if not self.dirs then
      self.dirs = {}
   end
   self.dirs[scale] = dirs
   if not self.dirs then
      self.dirs = {}
   end
   self.dirs[scale] = self.dirs[scale]
   return self.dirs[scale]
end

-- 
-- Caching
-- 
-- isdependant on pose rotation as well as image width, height, scale and center
function Pose:load_dirs(scale,ps)

   local imgw = self.w
   local imgh = self.h

   local outw = math.ceil(imgw/scale)
   local outh = math.ceil(imgh/scale)

   local cntrx = self.cntrx
   local cntry = self.cntry

   local dirscache   = 
      string.format("%s/pose_rot_%f_%f_%f_%f_w_%d_h_%d_s_%d_cx_%d_cy_%f",
                    self.cachedir,
                    self.quat[1],self.quat[2],self.quat[3],self.quat[4],
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
         dirs = util.grid_contiguous(self:compute_dirs(scale),ps,ps)
      else
         dirs = self:compute_dirs(scale)
      end
      printf("Built dirs in %2.2fs", sys.toc())
      torch.save(dirscache,dirs)
      printf("Saving dirs to %s", dirscache)
   end
   return self:store_dirs(dirs,i,scale)
end

function Pose:get_dirs(scale,ps)
   if not self.dirs[scale] then
      return load_dirs(scale,ps)
   else
      return self.dirs[scale]
   end
end

-- is unique for each scale and pose.
function Pose:load_depth(scale,ps)

   local imgw = self.w
   local imgh = self.h

   local outw = math.ceil(imgw/scale)
   local outh = math.ceil(imgh/scale)

   local cntrx = self.cntrx
   local cntry = self.cntry

   local depthcache   = self.cachedir .. 
      "orig_"..imgw.."x"..imgh.."_-_"..
      "scaled_"..outw.."x"..outh.."_-_"..
      "center_"..cntrx.."x"..cntry

   dirscache = dirscache ..".t7"

   local depthmap = nil
   if paths.filep(dirscache) then
      sys.tic()
      depthmap = torch.load(dirscache)
      printf("Loaded depths from %s in %2.2fs", posecache, sys.toc())
      if not self.depthmap then
         self.depthmap = {}
      end
      self.depthmap[scale] = depthmap
   else
      printf("No pose found")
   end
end


return Pose