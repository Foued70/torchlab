local geom = require "util.geom"
local loader = require "util.loader"
local util = require "util.util"

local r2d = 180 / math.pi
local d2r = math.pi / 180

local Pose = torch.class('Pose')

function Pose:__init(poses,i)
   self.pid    = i
   self.name   = poses.names[i]

   if poses.images then
      self.image  = poses.images[i]
   end

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

-- Extrinsic Parameters: 
-- global x,y,z to camera local x,y,z (origin at
-- camera ray origin, and 0,0,0 direction in center of image)
function Pose:global2local(v)
   return geom.rotate_by_quat(v - self.xyz,self.quat_r)
end

function Pose:local2global(v)
   return geom.rotate_by_quat(v,self.quat) + self.xyz
end

-- Intrinsic parameters: from camera xyz to 
-- FIXME optimize (in C) these funcs.
-- FIXME simplify number of ops.
function Pose:globalxyz2uv(pt)
   -- xyz in pose coordinates
   local v       = self:global2local(pt)
   local azimuth = -r2d * torch.atan2(v[2],v[1])
   local norm    = geom.normalize(v)
   local elevation = r2d * torch.asin(norm[3])
   local proj_x  = 0.5 + self.center_x + (  azimuth * self.px_per_degree_x)
   local proj_y  = 0.5 + self.center_y - (elevation * self.px_per_degree_y)
   -- u,v = 0,0 in upper left
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
   geom.normalize(dir)
   -- return point and direction rotated to global coordiante
   return self.xyz, geom.rotate_by_quat(dir,self.quat)
end

-- + FIXME improve speed : creation with a simple increment if possible. (SLERP)

function Pose:compute_dirs(scale)
   if not scale then 
      scale = 1
   end
   log.trace("Computing dirs for pose", self.pid, "at scale 1/", scale)

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
   return dirs
end

function Pose:dirs_file(scale, ps)
  local f = 'pose-'..self.pid..'_'..self.name..'_s'..scale  
  if ps then f = f .."_-_grid_".. ps end  
  f = f..'-dirs.t7'
  return f
end

function Pose:build_dirs(scale,ps)  
  sys.tic()
  local dirs = nil
  if ps then
    dirs = util.grid_contiguous(self:compute_dirs(scale),ps,ps)
  else
    dirs = self:compute_dirs(scale)
  end
  log.trace("Built dirs in", sys.toc())
  return dirs
end

-- loads if needed or return
function Pose:get_dirs(scale, ps)
  self.dirs = self.dirs or {}
  if not self.dirs[scale] then 
    self.dirs[scale] = loader(self:dirs_file(scale, ps), self.build_dirs, self, scale, ps)
  end
  return self.dirs[scale]  
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