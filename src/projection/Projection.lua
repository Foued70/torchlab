local projection_util = util.projection
local Projection = Class()

function Projection:__init(width, height, hfov, vfov, pixel_center_x, pixel_center_y)
   self.width = width or 100
   self.height = height or 100
   
   -- in radians
   self.hfov = hfov or 1
   self.vfov = vfov or 1

   if not pixel_center_x then 
      pixel_center_x = self.width/2
   end

   if not pixel_center_y then 
      pixel_center_y = self.height/2
   end

   self.center = {pixel_center_x, pixel_center_y}

   -- default to equirectangular (where units = radians)
   self.units_per_pixel_x = self.hfov/self.width
   self.units_per_pixel_y = self.vfov/self.height

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
   return self:coords_to_angles(coords)
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

   angles = torch.Tensor(2,maph,mapw)
   angles[1]:copy(lambda)
   angles[2]:copy(phi:t())
   return angles
end

function Projection:pixels_map(scale)
   -- make map of angles
   scale  = scale or 1

   mapw   = self.width * scale
   maph   = self.height * scale


   x = torch.linspace(1,mapw,mapw):resize(1,mapw):expand(maph,mapw)
   y = torch.linspace(1,maph,maph):resize(1,maph):expand(mapw,maph)

   pixels = torch.Tensor(2,mapw,maph)
   pixels[1]:copy(x)
   pixels[2]:copy(y:t())
   return pixels
end

function Projection:angles_map_to_pixels(scale,hfov,vfov)
   return self:angles_to_pixels(self:angles_map(scale,hfov,vfov))
end

function Projection:pixels_to_lookup(pixels)
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
      lookup_table = index_map, -- LongTensor for fast lookups
      mask         = mask,      -- ByteTensor invalid locations marked with 1
      stride       = self.width -- stride (width) of input image
          }
end

function Projection:angles_map_to_lookup(scale,hfov,vfov)

   return self:pixels_to_lookup(self:angles_map_to_pixels(scale,hfov,vfov))

end

function Projection:lookup_to_pixels(map)
   index_map = map.lookup_table
   stride    = map.stride

   pixels = torch.Tensor(2,index_map:size(1),index_map:size(2))
   xmap = pixels[1]
   ymap = pixels[2]

   xmap:copy(index_map):apply(function (x) return math.mod(x,stride) end)
   xmap[xmap:eq(0)] = stride
   ymap:copy(index_map):mul(1/stride):ceil()

return pixels

end
