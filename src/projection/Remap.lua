local Remap = Class()

function Remap:__init(projection_from, projection_to)
   self.projection_from = projection_from
   self.projection_to   = projection_to
end

function Remap:get_index_and_mask (force)
   if force or (not self.index1D) then
      local angle_map     = self.projection_to:angles_map()
      self.index1D, self.stride, self.mask =
         self.projection_from:angles_to_index1D_and_mask(angle_map)
   end
   return self.index1D, self.stride, self.mask
end

function Remap:remap(img,force)
   self:get_index_and_mask(force)
   return projection.util.remap(img, self.index1D, self.stride, self.mask)
end