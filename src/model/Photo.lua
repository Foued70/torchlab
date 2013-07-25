local Photo = Class()

local Ray = geom.Ray

local bihtree = model.bihtree
local loader = require '../data/loader'
local interpolate = model.interpolate
local normalize = geom.util.normalize

-- TODO add back the link to sweep
-- function Photo:__init(parent_sweep, image_path)
--   self.sweep = parent_sweep

function Photo:__init(local_to_global_position, 
                     local_to_global_rotation, 
                     hfov, vfov, clip_distance)

   self.hfov                     = hfov
   self.vfov                     = vfov
   self.local_to_global_position = local_to_global_position:clone()
   self.local_to_global_rotation = local_to_global_rotation:clone()
   self.global_to_local_position = 
      local_to_global_position:clone():mul(-1)
   self.global_to_local_rotation = 
      geom.quaternion.conjugate(local_to_global_rotation)

   -- store this for information and xyz angle computation.  Can't change.
   self.forward_vector = torch.Tensor({0,1,0})
end

-- move vertical
function Photo:vertical_offset(z)
   self.local_to_global_position[3] = self.local_to_global_position[3] + z
   self.global_to_local_position[3] = self.local_to_global_position[3] * -1
end

function Photo:set_image_path(image_path,max_size)
   self.image_path = image_path
   self.max_size   = max_size
   if not util.fs.is_file(self.image_path) then
      error("can't find ".. self.image_path)
   end
   log.info("Loading image from path:", "\""..self.image_path.."\"")
   -- use the wand to store the image data
   self.wand = image.Wand.new(self.image_path,self.max_size)
   self.width, self.height = self.wand:size()
end

function Photo:get_image(tensorType,colorspace,dimensions)
   local nocopy = true -- use wand to store imagedata
   if self.wand then
      log.tic()
      tensorType = tensorType or "torch.ByteTensor"
      colorspace = colorspace or "RGB"
      dimensions = dimensions or "DHW"
      imagedata = self.wand:toTensor(tensorType,colorspace,dimensions,nocopy)
      log.trace('Completed image load in', log.toc())
   else
      error("need to Photo:set_image_path first")
   end
   return imagedata
end

function Photo:set_projection(projection)
   self.projection = projection
end

-- Extrinsic Parameters:
-- global x,y,z to camera local x,y,z (origin at
-- camera ray origin, and 0,0,0 direction in center of image)
function Photo:global_to_local(v)
   return geom.quaternion.translate_rotate(
      self.global_to_local_position,
      self.global_to_local_rotation,
      v)
end

-- Extrinsic Parameters:
-- global x,y,z to camera local x,y,z (origin at
-- camera ray origin, and 0,0,0 direction in center of image)
function Photo:local_to_global(v)
   return geom.quaternion.rotate_translate(
      self.local_to_global_rotation,
      self.local_to_global_position,
      v)
end

-- this is the inverse of local_xy_to_global_rot + self.local_to_global_position
function Photo:global_xyz_to_local_angles(global_xyz)
   -- xyz in photo coordinates
   local local_xyz       = self:global_to_local(global_xyz)
   
   -- TODO put xyz in first dimension or add dim parameter to all rotation operations
   -- for now transfer from Nx4 to 2xN
   local s       = local_xyz:size()
   local st      = {2}
   for i   = 1,local_xyz:nDimension()-1 do table.insert(st,s[i]) end
   angles  = torch.Tensor(torch.LongStorage(st))
   local d = local_xyz:nDimension()
   local norm    = geom.util.normalize(local_xyz:narrow(d,1,3),d)

   local x = norm:select(d,1)
   local y = norm:select(d,2)
   local z = norm:select(d,3)

   -- local quats = geom.quaternion.angle_between(norm,self.forward_vector)
   -- local euler = geom.quaternion.to_euler_angle(quats)

   -- elevation
   torch.asin(angles[1],z)
   -- azimuth
   torch.atan2(angles[2],x,y)
   
   return angles

end

function Photo:global_xyz_to_offset_and_mask(xyz,debug)
   local angles = self:global_xyz_to_local_angles(xyz,debug)
   return self.projection:angles_to_offset_and_mask(angles)
end

-- rather than frustum just check a bunch of points on the face, reuse
-- code hopefully fast enough.
function Photo:check_overlap(xyz,percent)
   percent = percent or 0.1
   -- compute offset and mask for xyz is added.
   offset,stride,mask = self:global_xyz_to_offset_and_mask(xyz)
   return mask:eq(0):sum() > xyz:nElement() * percent
end
 
function Photo:project(xyz, offset, mask)
   local img  = self:get_image()
   -- compute offset and mask for xyz is added.
   if not offset or not mask then
      offset,stride,mask = self:global_xyz_to_offset_and_mask(xyz)
   end
   -- do the projection
   return util.addr.remap(img, offset, mask), offset, mask
end

-- loads if needed or return
-- TODO return of loader
function Photo:get_dirs(scale, ps)
  self.dirs = self.dirs or {}
  if not self.dirs[scale] then 
     self.dirs[scale] = 
        geom.util.spherical_angles_to_unit_cartesian(self.projection:angles_map(scale))
  end
  return self.dirs[scale]  
end

function Photo:project_dirs(xyz, offset, mask)
   dirs = self:get_dirs()
   if not offset or not mask then
      -- compute offset and mask for xyz is added.
      offset,stride,mask = self:global_xyz_to_offset_and_mask(xyz)
   end
   -- do the projection
   return util.addr.remap(dirs, offset, mask), offset, mask
end

function Photo:dirs_file(scale, ps)
  return self:file(scale, ps, 'dirs')
end

function Photo:file(scale, ps, filetype)
  local f = string.format('%s-%s-%s-s%s-', 
    path.basename(self.sweep.scan.model_file), path.basename(self.sweep.scan.pose_file), self.name, scale)
    
  if ps then f = f .."-grid".. ps end  
  return f..filetype..'.t7'
end

function Photo:depth_map_file(scale, ps)
  return self:file(scale, ps, 'depth')
end

function Photo:build_depth_map(scale, packetsize)  
  local target    = self.sweep.scan:get_model_data()
  local tree      = self.sweep.scan:get_bihtree()

  local dirs      = self:get_dirs(scale, packetsize)
  local out_tree  = torch.Tensor(dirs:size(1),dirs:size(2))
  local fid_tree  = torch.LongTensor(dirs:size(1),dirs:size(2))

  log.tic()
  log.trace("Computing depth map for photo", self.name, 'at scale 1/'..scale)

  local tot = 0
  local totmiss = 0
  local position = self.position

  local percent_complete = 0
  local dirs_dim1 = dirs:size(1)
  local dirs_dim2 = dirs:size(2)
  local total_dirs = dirs_dim1 * dirs_dim2

  for ri = 1,dirs_dim1 do
    for ci = 1,dirs_dim2 do      
      local current_progress = math.floor(((ri-1)*dirs_dim2 + ci) / (total_dirs) * 100)

      if current_progress > percent_complete then
        percent_complete = current_progress 
        log.trace("Computing depth map for photo", self.name, percent_complete.."%")
      end

      local ray = Ray.new(position,dirs[ri][ci])
      local tree_d, tree_fid = bihtree.traverse(tree,target,ray) 

      tot = tot + 1
      out_tree[ri][ci] = tree_d
      fid_tree[ri][ci] = tree_fid
      if (tree_d == math.huge) then
        totmiss = totmiss + 1
      end
    end
  end

  log.trace("Done computing depth map for photo", self.name, log.toc())

  log.trace("Interpolating for", totmiss, "missed edges out of", tot)
  log.tic()
  interpolate.math_huge(out_tree)
  log.trace("Interpolation done", log.toc())

  image.display{image={out_tree},min=0,max=10,legend=self.name}
  
  return out_tree
end

function Photo:get_depth_map(scale, packetsize, only_cached)
  if not self.depth_map then
    if packetsize and packetsize < 1 then packetsize = nil end    
    local filepath = path.join(self.sweep.path, self:depth_map_file(scale, packetsize))
    
    if util.fs.is_file(filepath) then
      self.depth_map = torch.load(filepath)
    else
      if only_cached then
        self.depth_map = nil
      else
        self.depth_map = self:build_depth_map(scale, packetsize)
        torch.save(filepath, self.depth_map)
      end
    end    
  end
  
  return self.depth_map
end
