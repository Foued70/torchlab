local geom = require "util/geom"

local r2d = 180 / math.pi
local d2r = math.pi / 180

local Pose = torch.class('Pose')

function Pose:__init(poses,i)
   self.pid    = i
   self.name   = poses.names[i]

   if poses.images then
      self.image  = poses.images[i]
   end

   self.cachedir = poses.cachedir

   self.quat            = poses.quat[i]
   self.xyz             = poses.xyz[i]
   self.center_u        = poses.center_u[i]
   self.center_v        = poses.center_v[i]
   self.degree_per_px_x = poses.degree_per_px_x[i]
   self.degree_per_px_y = poses.degree_per_px_y[i]
   self.px_per_degree_x = 1/self.degree_per_px_x
   self.px_per_degree_y = 1/self.degree_per_px_y
   self.quat_r          = poses.quat_r[i]
   self.center_x        = poses.center_x[i]
   self.center_y        = poses.center_y[i]
   self.image_w         = poses.image_w[i]
   self.image_h         = poses.image_h[i]
   self.inv_image_w     = 1/self.image_w
   self.inv_image_h     = 1/self.image_h
end

function Pose:global2local(v)
   return geom.rotate_by_quat(v - self.xyz,self.quat_r)
end

function Pose:local2global(v)
   return geom.rotate_by_quat(v,self.quat) + self.xyz
end

-- FIXME optimize (in C) these funcs.
function Pose:globalxyz2uv(pt)
   -- xyz in pose coordinates
   local v       = self:global2local(pt)
   local azimuth = -r2d * torch.atan2(v[2],v[1])
   local norm    = geom.normalize(v)
   local elevation = r2d * torch.asin(norm[3])
   local proj_x  = 0.5 + self.center_x + (  azimuth * self.px_per_degree_x)
   local proj_y  = 0.5 + self.center_y - (elevation * self.px_per_degree_y)
   local proj_u  = proj_x * self.inv_image_w
   local proj_v  = 1 - (proj_y * self.inv_image_h)
   return proj_u, proj_v, proj_x, proj_y
end

-- for pixel xy (0,0 in top left, w,h in bottom right) to point +
-- direction ray for intersection work
function Pose:localxy2globalray(x,y)
   local azimuth   = (x - self.center_x) * self.degree_per_px_x
   local elevation = (self.center_y - y) * self.degree_per_px_y
   azimuth   = - d2r * azimuth
   elevation =   d2r * elevation
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

   local image_w = self.image_w
   local image_h = self.image_h

   local outw = math.ceil(image_w/scale)
   local outh = math.ceil(image_h/scale)

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
   self.dirs[scale] = dirs
   return self.dirs[scale]
end

-- 
-- Caching
-- 
-- isdependant on pose rotation as well as image width, height, scale and center
function Pose:load_dirs(scale,ps)

   local image_w = self.image_w
   local image_h = self.image_h

   local outw = math.ceil(image_w/scale)
   local outh = math.ceil(image_h/scale)

   local center_x = self.center_x
   local center_y = self.center_y

   local dirscache   = 
      string.format("%s/pose_rot_%f_%f_%f_%f_w_%d_h_%d_s_%d_cx_%d_cy_%f",
                    self.cachedir,
                    self.quat[1],self.quat[2],self.quat[3],self.quat[4],
                    image_w,image_h,scale,center_x,center_y)
   
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
   return self:store_dirs(dirs,scale)
end

-- loads if needed or return
function Pose:get_dirs(scale,ps)
   if not self.dirs then
      self.dirs = {}
   end
   if not self.dirs[scale] then
      return self:load_dirs(scale,ps)
   else
      return self.dirs[scale]
   end
end

-- is unique for each scale and pose.
function Pose:load_depth(scale,ps)

   local image_w = self.image_w
   local image_h = self.image_h

   local outw = math.ceil(image_w/scale)
   local outh = math.ceil(image_h/scale)

   local center_x = self.center_x
   local center_y = self.center_y

   local depthcache   = self.cachedir .. 
      "orig_"..image_w.."x"..image_h.."_-_"..
      "scaled_"..outw.."x"..outh.."_-_"..
      "center_"..center_x.."x"..center_y

   depthcache = depthcache ..".t7"

   local depthmap = nil
   if paths.filep(depthcache) then
      sys.tic()
      depthmap = torch.load(depthcache)
      printf("Loaded depths from %s in %2.2fs", depthcache, sys.toc())
      if not self.depthmap then
         self.depthmap = {}
      end
      self.depthmap[scale] = depthmap
   else
      printf("No depth map found")
   end
end

function Pose:draw_wireframe (obj)
   local pimage = self.image
   local psize = pimage:size()
   psize[1] = 4
   local wimage = torch.Tensor(psize):fill(0)
   local face_verts = obj.face_verts
   local nverts = obj.nverts_per_face
   for fi = 1,face_verts:size(1) do
      local nv = nverts[fi]
      local f = face_verts[fi]:narrow(1,1,nv)
      local pvert = f[nv]
      for vi = 1,nv do
         local cvert = f[vi]
         local dir = cvert - pvert
         local len = torch.norm(dir)
         if (len > 1e-8) then
            dir = dir/len
            step = dir * mpp
            -- printf("step: %f,%f,%f",step[1],step[2],step[3])
            for s = 0,len,mpp do
               -- draw verts first
               local u,v,x,y = self:globalxyz2uv(pvert)
               -- printf("u: %f v: %f x: %f y %f", u, v, x, y)
               if (u > 0) and (u < 1) and (v > 0) and (v < 1) then
                  wimage[{1,y,x}] = 1  -- RED
                  wimage[{4,y,x}] = 1  -- Alpha Channel
               end
               pvert = pvert + step 
            end
         end
      end
   end
   return wimage
end


return Pose