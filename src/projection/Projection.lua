local Projection = Class()

-- CAREFUL: The Projection class stores the indexes for x and y in the
-- same height,width order as the underlying torch.Tensors()

function Projection:__init(width, height, hfov, vfov, pixel_center_x, pixel_center_y)
   self.width = width or 100
   self.height = height or 100

   -- in radians
   self.hfov = hfov or 1
   self.vfov = vfov or (self.hfov * self.height / self.width)

   pixel_center_x = pixel_center_x or self.width/2
   pixel_center_y = pixel_center_y or self.height/2

   self.center = {pixel_center_y, pixel_center_x}

   -- every projection must implement these
   self.units_per_pixel_x = nil
   self.units_per_pixel_y = nil

end
   
-- every projection must implement these low level functions.  they
-- are instance functions because some projections (such as the
-- calibrated ones) need access to instance variables
function Projection:normalized_coords_to_angles(normalized_coords, angles)
   error("Not implemented")

   -- WARNING. You should call this function at the end of your code.
   -- It resets your angles to lie between -pi and pi
   projection.util.recenter_angles(angles)

end

function Projection:angles_to_normalized_coords(angles, normalized_coords)
   error("Not implemented")
end

-- pixels - pixels from 0,0 in the upper left to width,height in the lower right
-- angles (optional) - azimuth, elevation from 0,0 center of projection
function Projection:pixels_to_angles(pixels, angles)
   angles = angles or torch.Tensor(pixels:size())
   angles:resize(pixels:size())

   local normalized_coords = pixels:clone()
   
   -- move 0,0 to the center
   -- convert to unit sphere coords
   normalized_coords[1]:add(-self.center[1]):mul(self.units_per_pixel_y)
   normalized_coords[2]:add(-self.center[2]):mul(self.units_per_pixel_x)

   -- convert unit sphere projection normalized_coords to angles
   self:normalized_coords_to_angles(normalized_coords,angles)

   -- remove nans
   nans = angles:lt(1):add(angles:ge(1)):eq(0)
   n_nans = nans:sum()
   if (n_nans > 0) then 
      printf("removing %d nans",n_nans) 
      angles[nans] = 0
   end

   return angles
end


-- angles - azimuth, elevation from 0,0 center of projection
-- pixels (optional) - pixels from 0,0 in the upper left to width,height in the lower right
function Projection:angles_to_pixels(angles, pixels)
   angles = angles:narrow(1,1,2)
   pixels = pixels or torch.Tensor(angles:size())
   pixels:resize(angles:size())

   self:angles_to_normalized_coords(angles, pixels)

   -- convert from unit sphere coords to pixels
   -- move 0,0 to the upper left corner
   pixels[1]:mul(1/self.units_per_pixel_y):add(self.center[1])
   pixels[2]:mul(1/self.units_per_pixel_x):add(self.center[2])

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
   
   pixels[1]:copy(y:t())
   pixels[2]:copy(x)
   
   return pixels
end

-- grid of angles to equally spaced pixels
function Projection:angles_map(scale,pixels_map)
   scale   = scale   or 1
   pixels_map = pixels_map or self:pixels_map(scale)
   return self:pixels_to_angles(pixels_map)
end

-- 
-- offset : (LongTensor) for fast lookups
-- stride : (LongTensor) {width, 1} of input image
--   mask : (ByteTensor) invalid locations marked with 1
-- 
function Projection:pixels_to_offset_and_mask(pixels, offset, mask)

   local dims = torch.LongTensor({self.height,self.width})

   offset = offset or torch.LongTensor(pixels[1]:size())
   mask   =   mask or torch.ByteTensor(pixels[1]:size())
   
   -- make mask for out of bounds values
   mask = util.addr.mask_out_of_bounds(pixels,dims,nil,mask)
   -- log.tracef("Masking %d/%d pixel locations (out of bounds)", mask:sum(),mask:nElement())

   stride    = dims
   stride[1] = self.width
   stride[2] = 1

   -- convert the x and y values into a single 1D offset (y * stride + x)
   offset = util.addr.pixel_coords_to_offset(pixels,stride,mask,offset)

   return offset, stride, mask
end

function Projection:angles_to_offset_and_mask(angles, offset, mask)
   angles = angles:narrow(1,1,2)
   offset = offset or torch.LongTensor(angles[1]:size())
   mask   =   mask or torch.ByteTensor(angles[1]:size())
   
   local pixels = self:angles_to_pixels(angles)
   return self:pixels_to_offset_and_mask(pixels,offset,mask)
end

function Projection:offset_and_mask_to_pixels(offset,stride,mask,pixels)
   return util.addr.offset_to_pixel_coords(offset,stride,pixels)
end


function Projection:offset_and_mask_to_angles(offset,stride,mask,angles)
   local pixels = util.addr.offset_to_pixel_coords(offset,stride)
   return self:pixels_to_angles(pixels,angles)
end
