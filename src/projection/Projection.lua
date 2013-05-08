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

