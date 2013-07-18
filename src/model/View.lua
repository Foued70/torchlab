local View = Class()

local normalize = geom.util.normalize

function View:__init(local_to_global_position, 
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
   

   self:set_frustrum(hfov,vfov,clip_distance)

   -- store this for information and xyz angle computation.  Can't change.
   self.forward_vector = torch.Tensor({0,1,0})
end

-- move vertical
function View:vertical_offset(z)
   self.local_to_global_position[3] = self.local_to_global_position[3] + z
   self.global_to_local_position[3] = self.local_to_global_position[3] * -1
end
      
function View:set_frustrum(hfov,vfov,clip_distance)
   -- set/reset (h,v)fov if passed
   hfov = hfov or self.hfov
   vfov = vfov or self.vfov

   local n_planes = 5
   if clip_distance then
      n_planes = 6
   end

   -- v,h +,+|+,-|-,-|-,+|
   local frustrum_angles = torch.Tensor({{vfov,vfov,-vfov,-vfov,0},{hfov,-hfov,-hfov,hfov,0}})
   frustrum_angles:mul(0.5)
   local frustrum_quat = geom.quaternion.from_euler_angle(frustrum_angles)
   -- move frustrum quat to global rotation
   frustrum_quat = 
      geom.quaternion.product(frustrum_quat,self.local_to_global_rotation)
   local frustrum_vect = geom.quaternion.rotate(frustrum_quat,geom.quaternion.y)

   local frustrum_plane = torch.zeros(n_planes,4)
   local prev = frustrum_vect[4]
   for v = 1,4 do
      local cur = frustrum_vect[v]
      frustrum_plane[{v,{1,3}}] = normalize(torch.cross(prev,cur))
      prev = cur
   end
   frustrum_plane[{5,{1,3}}] = frustrum_vect[5]

   if clip_distance then
      frustrum_plane[{6,{1,3}}] = frustrum_vect[5]
      -- far plane points in the opposite direction
      frustrum_plane[{6,{1,3}}]:mul(-1)
   end

   -- compute d for each plane normal vector
   frustrum_plane[{{},4}] =
      torch.cmul(frustrum_plane[{{},{1,3}}],self.local_to_global_position:reshape(1,3):expand(n_planes,3)):sum(2):mul(-1)

   if clip_distance then
      local far_point = local_to_global_position + (frustrum_vect[5] * clip_distance)
      frustrum_plane[6][4] = torch.dot(far_point,frustrum_plane[{6,{1,3}}])
   end

   self.frustrum   = frustrum_plane
   self.hfov       = hfov
   self.vfov       = vfov
   self.n_planes   = n_planes

end

function View:set_image_path(image_path,max_size)
   self.image_path = image_path
   self.max_size   = max_size
end

function View:get_image()
   if not self.image_data_raw then
      if not util.fs.is_file(self.image_path) then
         error("can't find ".. image_path)
      end
      log.tic()
      log.trace("Loading image from path:", "\""..self.image_path.."\"")

      local img = image.Wand.new(self.image_path,self.max_size):toTensor('double','RGB','DHW')
      self.image_data_raw = img
      collectgarbage()
      log.trace('Completed image load in', log.toc())
   end
   return self.image_data_raw
end

function View:set_projection(projection)
   self.projection = projection
end


-- Extrinsic Parameters:
-- global x,y,z to camera local x,y,z (origin at
-- camera ray origin, and 0,0,0 direction in center of image)
function View:global_to_local(v)
   return geom.quaternion.translate_rotate(
      self.global_to_local_position,
      self.global_to_local_rotation,
      v)
end

-- Extrinsic Parameters:
-- global x,y,z to camera local x,y,z (origin at
-- camera ray origin, and 0,0,0 direction in center of image)
function View:local_to_global(v)
   return geom.quaternion.rotate_translate(
      self.local_to_global_rotation,
      self.local_to_global_position,
      v)
end

-- this is the inverse of local_xy_to_global_rot + self.local_to_global_position
function View:global_xyz_to_local_angles(global_xyz,debug)
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
   
   if debug then
      log.trace("local_to_global_position", self.local_to_global_position)
      log.trace("local_to_global_rotation", self.local_to_global_rotation)
      log.trace("global_to_local_position", self.global_to_local_position)
      log.trace("global_to_local_rotation", self.global_to_local_rotation)
   end

   return angles

end

function View:global_xyz_to_offset_and_mask(xyz,debug)
   local angles = self:global_xyz_to_local_angles(xyz,debug)
   self.offset, self.stride, self.mask =
      self.projection:angles_to_offset_and_mask(angles)
   return self.offset, self.stride, self.mask
end

function View:project(xyz)
   local img  = self:get_image()
   local proj = self.projection

   -- recompute angles if xyz is added.
   if xyz then
      self:global_xyz_to_offset_and_mask(xyz)
   end

   -- do the projection
   return util.addr.remap(img, self.offset, self.mask)
end

