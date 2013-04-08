require 'torch'
require 'image'
require 'paths'

local LensSensor = require "util.LensSensor"
local projection = require "util.projection"
local geom = require 'util.geom'
local loader = require 'util.loader'

local r2d = 180 / math.pi
local d2r = math.pi / 180

local Photo = torch.class('Photo')

local REQUIRED_CALIBRATION_PAIRS = 4

function Photo:__init(parent_sweep, image_path)
  self.sweep = parent_sweep
  self.calibration_pairs = torch.Tensor(REQUIRED_CALIBRATION_PAIRS,5):fill(0)
  self.pairs_calibrated = 0
  self.vertex_set = false
  self.image_coordinate_set = false
  self.white_wall = false

  self.name = paths.basename(image_path)
  self.image_path = image_path
  self.image_data_raw = nil
  self.image_data_rectilinear = nil
  self.image_data_spherical = nil
  self.image_w = nil
  self.image_h = nil

  
  self.offset_position = nil
  self.offset_rotation = nil
  self.rotation = nil
  self.rotation_r = nil
  self.position = nil
  
  self.lens = nil
end

function Photo:add_vertex(vertex_position)
  if self.pairs_calibrated < REQUIRED_CALIBRATION_PAIRS then
    local pair_index = self.pairs_calibrated + 1
    self.calibration_pairs[pair_index]:sub(1, 3):copy(vertex_position)
    self.vertex_set = true
    self:update_calibration_status()
  end
end

function Photo:add_image_coordinate(screen_position)
  if self.pairs_calibrated < REQUIRED_CALIBRATION_PAIRS then
    local pair_index = self.pairs_calibrated + 1
    self.calibration_pairs[pair_index]:sub(4, 5):copy(screen_position)
    self.image_coordinate_set = true
    self:update_calibration_status()
  end
end

function Photo:update_calibration_status()
  if (self.vertex_set == true) and (self.image_coordinate_set == true) then
    self.pairs_calibrated = self.pairs_calibrated + 1
    self.vertex_set = false
    self.image_coordinate_set = false
  end
end

function Photo:calibration_complete()
  return (self.pairs_calibrated == REQUIRED_CALIBRATION_PAIRS)
end

function Photo:delete_calibration_pair(pair_index)
  for i = pair_index + 1, self.pairs_calibrated do
    self.calibration_pairs[i-1]:copy(self.calibration_pairs[i])
  end

  self.calibration_pairs[self.pairs_calibrated]:fill(0)
  self.pairs_calibrated = self.pairs_calibrated - 1
end

function Photo:get_image()
  if not self.image_data_raw then
    sys.tic()
    log.trace("Loading image from path:", "\""..self.image_path.."\"")
    self.image_data_raw = image.load(self.image_path)
    log.trace('Completed image load in', sys.toc())
  end
  return self.image_data_raw
end

function Photo:get_image_rectilinear()
  if not self.image_data_rectilinear then
    self.image_data_rectilinear = projection.remap(self:get_image(), self:get_lens().rectilinear)
  end
  return self.image_data_rectilinear
end

function Photo:get_image_spherical()
  if not self.image_data_spherical then
    self.image_data_spherical = projection.remap(self:get_image(), self:get_lens().spherical)
  end
  return self.image_data_spherical
end

function Photo:get_lens()
  if not self.lens then
    sys.tic()
    self.lens = self.sweep.scan:get_lens(self:get_image())
    log.trace('Loaded lens in', sys.toc())
  end
  return self.lens
end

function Photo:flush_image()
  self.image_data_raw = nil
  self.image_data_rectilinear = nil
  self.image_data_spherical = nil
  collectgarbage()
end

-- Extrinsic Parameters: 
-- global x,y,z to camera local x,y,z (origin at
-- camera ray origin, and 0,0,0 direction in center of image)
function Photo:global2local(v)
  return geom.rotate_by_quat(v - self.position, self.rotation_r)
end

function Photo:local2global(v)
   return geom.rotate_by_quat(v,self.rotation) + self.position
end

-- Intrinsic parameters: from camera xyz to 
-- FIXME optimize (in C) these funcs.
-- FIXME simplify number of ops.
function Photo:globalxyz2uv(pt)
   -- xyz in pose coordinates
   local v       = self:global2local(pt)
   local azimuth = -r2d * torch.atan2(v[2],v[1])
   local norm    = geom.normalize(v)
   local elevation = r2d * torch.asin(norm[3])
   
   local lens = self:get_lens()
   local proj_x  = 0.5 + lens.sensor.center_x + (  azimuth * lens.sensor.inv_hfov)
   local proj_y  = 0.5 + lens.sensor.center_y - (elevation * lens.sensor.inv_vfov)
   
   -- u,v = 0,0 in upper left
   local proj_u  = proj_x * lens.sensor.inv_image_w
   local proj_v  = 1 - (proj_y * lens.sensor.inv_image_h)
   return proj_u, proj_v, proj_x, proj_y
end

-- for pixel xy (0,0 in top left, w,h in bottom right) to point +
-- direction ray for intersection work
function Photo:localxy2globalray(x,y)
  local lens = self:get_lens()
  
  local azimuth   = (x - lens.sensor.center_x) * lens.sensor.hfov
  local elevation = (lens.sensor.center_y - y) * lens.sensor.vfov
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
  return self.position, geom.rotate_by_quat(dir,self.rotation)
end

function Photo:compute_dirs(scale)
  local lens = self:get_lens()
  
  if not scale then 
    scale = 1
  end
  log.trace("Computing dirs for photo", self.name, "at scale 1/", scale)

  local image_w = lens.sensor.image_w
  local image_h = lens.sensor.image_h

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

function Photo:dirs_file(scale, ps)
  local f = 'photo-'..self.name..'_s'..scale  
  if ps then f = f .."_-_grid_".. ps end  
  f = f..'-dirs.t7'
  return f
end

function Photo:build_dirs(scale,ps)  
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
function Photo:get_dirs(scale, ps)
  self.dirs = self.dirs or {}
  if not self.dirs[scale] then 
    self.dirs[scale] = loader(self:dirs_file(scale, ps), self.build_dirs, self, scale, ps)
  end
  return self.dirs[scale]  
end

function Photo:draw_wireframe()
  local obj = self.sweep.scan:get_model_data()
  local pimage = self:get_image()
  
  local psize = pimage:size()
  psize[1] = 4
  
  local wimage = torch.Tensor(psize):fill(0)
  local face_verts = obj.face_verts
  local nverts = obj.n_verts_per_face
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

return Photo