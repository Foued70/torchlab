local Remap = Class()

function Remap:__init(projection_from, projection_to)
   self.projection_from = projection_from
   self.projection_to   = projection_to
   self.scale           = 1
end

function Remap:get_index_and_mask (scale)
   if (not self.index_and_mask) or (scale ~= self.scale) then 
      self.scale          = scale
      local angle_map     = self.projection_to:angles_map(scale)
      self.index_and_mask = self.projection_from:angles_to_index1D_and_mask(angle_map)
   end
   return self.index_and_mask
end

function Remap:remap(img,scale)
   scale = scale or self.scale
   local index_and_mask = self:get_index_and_mask(scale)
   return projection.util.remap(img,index_and_mask)
end