local Remap = Class()

function Remap:__init(projection_input, projection_output)
   self.projection_input  = projection_input
   self.projection_output = projection_output
end

function Remap:set_input_lambda_phi(lambda,phi)
   self.projection_input:set_lambda_phi(lambda,phi)
end

function Remap:set_input_hfov_vfov(hfov,vfov)
   self.projection_input:set_hfov(hfov)
   self.projection_input:set_vfov(vfov)
end

function Remap:update()
   angle_map = self.projection_output:angles_map()
   self.offset, self.stride, self.mask =
      self.projection_input:angles_to_offset_and_mask(angle_map)
end

function Remap:get_offset_and_mask (force)
   if force or (not self.offset) then
      self:update()
   end
   return self.offset, self.stride, self.mask
end

-- mask is 1 in the areas which need to be masked
function Remap:get_mask (force)
   if force or (not self.offset) then
      self:update()
   end
   return self.mask
end

-- alpha is the inverted mask like an alpha channel.  1 is visible,opaque, 0 is masked, transparent.
function Remap:get_alpha (force)
   return self:get_mask(force):eq(0)
end

function Remap:remap(img,force)
   self:get_offset_and_mask(force)
   return util.addr.remap(img, self.offset, self.mask)
end
