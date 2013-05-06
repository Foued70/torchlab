local RectilinearProjection = Class(data.Projection)

function RectilinearProjection:__init(width, height, vfov)
  __super__.__init(self, width, height)

  local vfov_angles = torch.Tensor({{0, vfov/2}})
  local vfov_coords = angles_to_coords(vfov_angles)
  self.pixels_per_unit = (height/2) / vfov_coords[{1, 2}]

  self.center = {(width+1)/2, (height+1)/2}
end

-- pixels - pixels from 0,0 in the upper left to width,height in the lower right
-- angles (optional) - azimuth, elevation from 0,0 center of projection
function RectilinearProjection:pixels_to_angles(pixels, angles)
  angles = angles or torch.Tensor(pixels:size(1), 2)

  local coords = pixels:clone()

  -- move 0,0 to the center
  coords[{{},1}]:add(-self.center[1])
  coords[{{},2}]:add(-self.center[2])

  -- convert to unit sphere coords
  coords:div(self.pixels_per_unit)

  -- convert unit sphere projection coords to angles
  return coords_to_angles(coords)
end


-- angles - azimuth, elevation from 0,0 center of projection
-- pixels (optional) - pixels from 0,0 in the upper left to width,height in the lower right
function RectilinearProjection:angles_to_pixels(angles, pixels)
  pixels = pixels or torch.Tensor(angles:size(1), 2)

  angles_to_coords(angles, pixels)

  -- convert from unit sphere coords to pixels
  pixels:mul(self.pixels_per_unit)

  -- move 0,0 to the upper left corner
  pixels[{{},1}]:add(self.center[1])
  pixels[{{},2}]:add(self.center[2])

  return pixels
end


-- coords - coords in the unit sphere projection, 0,0 center
-- angles (optional) - azimuth, elevation from 0,0 center of projection
function coords_to_angles(coords, angles)
  angles = angles or torch.Tensor(coords:size(1), 2)

  local azimuth = torch.atan(coords[{{}, 1}])
  angles[{{}, 1}] = azimuth
  angles[{{}, 2}] = torch.cmul(coords[{{}, 2}], torch.cos(azimuth)):atan() 

  return angles
end


-- angles - azimuth, elevation from 0,0 center of projection
-- coords (optional) - coords in the unit sphere projection
function angles_to_coords(angles, coords)
  coords = coords or torch.Tensor(angles:size(1), 2)

  local azimuth = angles[{{}, 1}]
  local elevation = angles[{{}, 2}]

  coords[{{}, 1}] = torch.tan(azimuth)
  coords[{{}, 2}] = torch.tan(elevation):cdiv(torch.cos(azimuth))

  return coords
end