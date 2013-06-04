local Remap = Class()

function Remap:__init(projection_from, projection_to)
   self.projection_from = projection_from
   self.projection_to   = projection_to
end

function Remap:get_offset_and_mask (force)
   if force or (not self.offset) then
      local angle_map = self.projection_to:angles_map()
      self.offset, self.stride, self.mask =
         self.projection_from:angles_to_offset_and_mask(angle_map)
   end
   return self.offset, self.stride, self.mask
end

function Remap:remap(img,force)
   self:get_offset_and_mask(force)
   return util.addr.remap(img, self.offset, self.mask)
end