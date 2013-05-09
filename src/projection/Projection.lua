local projection_util = util.projection
local Projection = Class()

function Projection:__init(width, height, hfov, vfov, pixel_center_x, pixel_center_y)
   self.width = width or 100
   self.height = height or 100

   -- in radians
   self.hfov = hfov or 1
   self.vfov = vfov or 1

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

   local coords = pixels:clone()

   -- move 0,0 to the center
   -- convert to unit sphere coords
   coords[1]:add(-self.center[1]):mul(self.units_per_pixel_x)
   coords[2]:add(-self.center[2]):mul(self.units_per_pixel_y)

   -- convert unit sphere projection coords to angles
   self:coords_to_angles(coords,angles)

   return angles
end


-- angles - azimuth, elevation from 0,0 center of projection
-- pixels (optional) - pixels from 0,0 in the upper left to width,height in the lower right
function Projection:angles_to_pixels(angles, pixels)
   pixels = pixels or torch.Tensor(angles:size())

   self:angles_to_coords(angles, pixels)

   -- convert from unit sphere coords to pixels
   -- move 0,0 to the upper left corner
   pixels[1]:mul(1/self.units_per_pixel_x):add(self.center[1])
   pixels[2]:mul(1/self.units_per_pixel_y):add(self.center[2])

   return pixels
end

-- grid of pixels equally spaced
-- TODO allow offsets so that the map is not always from 1,width and 1,height
function Projection:pixels_map(scale)
   -- make map of angles
   scale  = scale or 1

   local mapw   = self.width * scale
   local maph   = self.height * scale

   local x = torch.linspace(1,self.width,mapw):resize(1,mapw):expand(maph,mapw)
   local y = torch.linspace(1,self.height,maph):resize(1,maph):expand(mapw,maph)

   local pixels = torch.Tensor(2,maph,mapw)
   pixels[1]:copy(x)
   pixels[2]:copy(y:t())
   return pixels
end

-- grid of angles to equally spaced pixels
-- TODO allow offsets so that the map is not always from 1,width and 1,height
function Projection:angles_map(scale)
   return self:pixels_to_angles(self:pixels_map(scale))
end

--
function Projection:pixels_to_index1D_and_mask(pixels)
   -- make mask for out of bounds values
   local mask = projection_util.make_mask(pixels,self.width,self.height)
   log.tracef("Masking %d/%d pixel locations (out of bounds)", mask:sum(),mask:nElement())

   -- convert the x and y values into a single 1D offset (y * stride + x)
   local index1D = projection_util.make_index(pixels,mask,self.width)

   return {
      index1D      = index1D,   -- LongTensor for fast lookups
      mask         = mask,      -- ByteTensor invalid locations marked with 1
      stride       = self.width -- stride (width) of input image
          }
end

function Projection:angles_to_index1D_and_mask(angles)
   return self:pixels_to_index1D_and_mask(self:angles_to_pixels(angles))
end

function Projection:index1D_and_mask_to_pixels(map)
   local index1D = map.index1D
   local stride  = map.stride

   local pixels = torch.Tensor(2,index1D:size(1),index1D:size(2))
   local xmap   = pixels[1]
   local ymap   = pixels[2]

   xmap:copy(index1D):apply(function (x) return math.mod(x,stride) end)
   xmap[xmap:eq(0)] = stride
   ymap:copy(index1D):mul(1/stride):ceil()

   return pixels

end
