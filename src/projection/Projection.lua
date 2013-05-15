local Projection = Class()

function Projection:__init(width, height, hfov, vfov, pixel_center_x, pixel_center_y)
   self.width = width or 100
   self.height = height or 100

   -- in radians
   self.hfov = hfov or 1
   self.vfov = vfov or (self.hfov * self.height / self.width)

   pixel_center_x = pixel_center_x or self.width/2
   pixel_center_y = pixel_center_y or self.height/2

   self.center = {pixel_center_x, pixel_center_y}

   -- every projection must implement these
   self.units_per_pixel_x = nil
   self.units_per_pixel_y = nil

end
   
-- every projection must implement these low level functions.  they
-- are instance functions because some projections (such as the
-- calibrated ones) need access to instance variables
function Projection:coords_to_angles(coords, angles)
   error("Not implemented")
end

function Projection:angles_to_coords(angles, coords)
   error("Not implemented")
end

-- pixels - pixels from 0,0 in the upper left to width,height in the lower right
-- angles (optional) - azimuth, elevation from 0,0 center of projection
function Projection:pixels_to_angles(pixels, angles)
   angles = angles or torch.Tensor(pixels:size())
   angles:resize(pixels:size())

   local coords = pixels:clone()
   
   -- move 0,0 to the center
   -- convert to unit sphere coords
   coords[1]:add(-self.center[1]):mul(self.units_per_pixel_x)
   coords[2]:add(-self.center[2]):mul(self.units_per_pixel_y)

   -- convert unit sphere projection coords to angles
   self:coords_to_angles(coords,angles)

   -- make sure the angles lie between -pi and pi
   projection.util.recenter_angles(angles)

   return angles
end


-- angles - azimuth, elevation from 0,0 center of projection
-- pixels (optional) - pixels from 0,0 in the upper left to width,height in the lower right
function Projection:angles_to_pixels(angles, pixels)
   pixels = pixels or torch.Tensor(angles:size())
   pixels:resize(angles:size())

   self:angles_to_coords(angles, pixels)

   -- convert from unit sphere coords to pixels
   -- move 0,0 to the upper left corner
   pixels[1]:mul(1/self.units_per_pixel_x):add(self.center[1])
   pixels[2]:mul(1/self.units_per_pixel_y):add(self.center[2])

   return pixels
end

-- grid of pixels equally spaced
function Projection:pixels_map(scale, from_x, to_x, from_y, to_y)
   -- make map of angles
   scale  = scale  or 1
   
   local mapw   = self.width * scale
   local maph   = self.height * scale

   local pixels = torch.Tensor(2,maph,mapw)
   
   from_x = from_x or 1
   to_x   = to_x   or self.width
   from_y = from_y or 1
   to_y   = to_y   or self.height
   
   
   local x = torch.linspace(from_x,to_x,mapw):resize(1,mapw):expand(maph,mapw)
   local y = torch.linspace(from_y,to_y,maph):resize(1,maph):expand(mapw,maph)
   
   pixels[1]:copy(x)
   pixels[2]:copy(y:t())
   
   return pixels
end

-- grid of angles to equally spaced pixels
function Projection:angles_map(scale,pixels_map)
   scale   = scale   or 1
   pixels_map = pixels_map or self:pixels_map(scale)
   return self:pixels_to_angles(pixels_map)
end

-- 
-- index1D : (LongTensor) for fast lookups
--  stride : (number) stride (width) of input image
--    mask : (ByteTensor) invalid locations marked with 1
function Projection:pixels_to_index1D_and_mask(pixels, index1D, mask)

   local stride = self.width
   local max_x  = self.width
   local max_y  = self.height

   index1D = index1D or torch.LongTensor(pixels[1]:size())
   mask    =    mask or torch.ByteTensor(pixels[1]:size())

   
   -- make mask for out of bounds values
   mask = projection.util.make_mask(pixels,max_x,max_y,mask)
   log.tracef("Masking %d/%d pixel locations (out of bounds)", mask:sum(),mask:nElement())


   -- convert the x and y values into a single 1D offset (y * stride + x)
   index1D = projection.util.make_index(pixels,stride,mask,index1D)

   return index1D, stride, mask
end

function Projection:angles_to_index1D_and_mask(angles, index1D, mask)

   index1D = index1D or torch.LongTensor(angles[1]:size())
   mask    =    mask or torch.ByteTensor(angles[1]:size())
   
   local pixels = self:angles_to_pixels(angles)
   return self:pixels_to_index1D_and_mask(pixels,index1D,mask)

end

function Projection:index1D_and_mask_to_pixels(index1D,stride,mask,pixels)

   return projection.util.index1D_to_xymap(index1D, stride, pixels)

end


function Projection:index1D_and_mask_to_angles(index1D,stride,mask,angles)

   local pixels = projection.util.index1D_to_xymap(index1D, stride)
   return self:pixels_to_angles(pixels,angles)

end
