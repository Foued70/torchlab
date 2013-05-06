local EquirectangularProjection = Class(data.Projection)

function EquirectangularProjection:__init(width, height, hfov, vfov, pixel_center_x, pixel_center_y)
  __super__.__init(self, width, height)

  self.pixels_per_unit_x = width/hfov
  self.pixels_per_unit_y = height/vfov

  self.center = {pixel_center_x, pixel_center_y}
end

-- pixels - pixels from 0,0 in the upper left to width,height in the lower right
-- angles (optional) - azimuth, elevation from 0,0 center of projection
function EquirectangularProjection:pixels_to_angles(pixels, angles)
  angles = angles or torch.Tensor(pixels:size(1), 2)

  local coords = pixels:clone()

  -- move 0,0 to the center
  -- convert to unit sphere coords
  coords[{{},1}]:add(-self.center[1]):div(self.pixels_per_unit_x)
  coords[{{},2}]:add(-self.center[2]):div(self.pixels_per_unit_y)

  -- convert unit sphere projection coords to angles
  return coords_to_angles(coords)
end


-- angles - azimuth, elevation from 0,0 center of projection
-- pixels (optional) - pixels from 0,0 in the upper left to width,height in the lower right
function EquirectangularProjection:angles_to_pixels(angles, pixels)
  pixels = pixels or torch.Tensor(angles:size(1), 2)

  angles_to_coords(angles, pixels)

  -- convert from unit sphere coords to pixels
  -- move 0,0 to the upper left corner
  pixels[{{},1}]:mul(self.pixels_per_unit_x):add(self.center[1])
  pixels[{{},2}]:mul(self.pixels_per_unit_y):add(self.center[2])

  return pixels
end


-- coords - coords in the unit sphere projection, 0,0 center
-- angles (optional) - azimuth, elevation from 0,0 center of projection
function coords_to_angles(coords, angles)
  angles = angles or torch.Tensor(coords:size(1), 2)
  angles:copy(coords)

  return angles
end


-- angles - azimuth, elevation from 0,0 center of projection
-- coords (optional) - coords in the unit sphere projection
function angles_to_coords(angles, coords)
  coords = coords or torch.Tensor(angles:size(1), 2)
  coords:copy(angles)

  return coords
end