local projection_util = util.projection
local Projection = Class()

function Projection:__init(width, height, hfov, vfov, pixel_center_x, pixel_center_y)
   self.width = width or 100
   self.height = height or 100
   
   self.hfov = hfov
   self.vfov = vfov

   if hfov then
      self.pixels_per_unit_x = self.width/hfov
   else
      self.pixels_per_unit_x = 1
   end

   if vfov then 
      self.pixels_per_unit_y = self.height/vfov      
   else
      self.pixels_per_unit_y = self.pixels_per_unit_x 
   end

   if not pixel_center_x then 
      pixel_center_x = self.width/2
   end

   if not pixel_center_y then 
      pixel_center_y = self.height/2
   end

   self.center = {pixel_center_x, pixel_center_y}
end

-- pixels - pixels from 0,0 in the upper left to width,height in the lower right
-- angles (optional) - azimuth, elevation from 0,0 center of projection
function Projection:pixels_to_angles(pixels, angles)
   angles = angles or torch.Tensor(pixels:size())

   local coords = pixels:clone()

   -- move 0,0 to the center
   -- convert to unit sphere coords
   coords[1]:add(-self.center[1]):div(self.pixels_per_unit_x)
   coords[2]:add(-self.center[2]):div(self.pixels_per_unit_y)

   -- convert unit sphere projection coords to angles
   return self:coords_to_angles(coords)
end


-- angles - azimuth, elevation from 0,0 center of projection
-- pixels (optional) - pixels from 0,0 in the upper left to width,height in the lower right
function Projection:angles_to_pixels(angles, pixels)
   pixels = pixels or torch.Tensor(angles:size())

   self:angles_to_coords(angles, pixels)

   -- convert from unit sphere coords to pixels
   -- move 0,0 to the upper left corner
   pixels[1]:mul(self.pixels_per_unit_x):add(self.center[1])
   pixels[2]:mul(self.pixels_per_unit_y):add(self.center[2])

   return pixels
end

-- 
function Projection:angles_map(scale,hfov,vfov)
   -- make map of angles
   scale  = scale or 1
   mapw   = self.width * scale
   maph   = self.height * scale

   hfov   = hfov or self.hfov
   vfov   = vfov or self.vfov

   half_hfov = hfov * 0.5
   half_vfov = vfov * 0.5

   lambda = torch.linspace(-half_hfov,half_hfov,mapw):resize(1,mapw):expand(maph,mapw)
   phi    = torch.linspace(-half_vfov,half_vfov,maph):resize(1,maph):expand(mapw,maph)

   angles = torch.Tensor(2,mapw,maph)
   angles[1]:copy(lambda)
   angles[2]:copy(phi:t())
   return angles
end

function Projection:angles_to_pixels_map(scale,hfov,vfov)
   return self:angles_to_pixels(self:angles_map(scale,hfov,vfov))
end

function Projection:angles_to_pixels_lookup_map(scale,hfov,vfov)

   pixels = self:angles_to_pixels_map(scale,hfov,vfov)

   -- +++++++
   -- make mask for out of bounds values
   -- +++++++ 
   mask = projection_util.make_mask(pixels,self.width,self.height)
   log.tracef("Masking %d/%d lookups (out of bounds)", mask:sum(),mask:nElement())

   -- +++++++
   -- convert the x and y values into a single 1D offset (y * stride + x)
   -- +++++++
   
   index_map = projection_util.make_index(pixels,mask,self.width)
   
   return {
      lookup_table     = index_map, -- 1D LongTensor for fast lookups
      mask             = mask,      -- ByteTensor invalid locations marked with 1
      height           = maph,      -- height of the output map
      width            = mapw       -- width of the output map
          }
end