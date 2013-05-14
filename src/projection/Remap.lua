local Remap = Class()

function Remap:__init(projection_from, projection_to)
   self.projection_from = projection_from
   self.projection_to   = projection_to
   self.scale           = 1
end

function Remap:get_index_and_mask (scale)
   if (not self.index1D) or (scale ~= self.scale) then
      self.scale          = scale or 1
      local angle_map     = self.projection_to:angles_map(scale)
      self.index1D, self.stride, self.mask =
         self.projection_from:angles_to_index1D_and_mask(angle_map)
   end
   return self.index1D, self.stride, self.mask
end

function Remap:remap(img,scale)
   scale = scale or self.scale
   self:get_index_and_mask(scale)
   return projection.util.remap(img, self.index1D, self.stride, self.mask)
end