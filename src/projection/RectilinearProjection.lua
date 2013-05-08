local RectilinearProjection = Class(projection.Projection)

function RectilinearProjection:__init(width, height, 
                                      hfov, vfov, 
                                      pixel_center_x, pixel_center_y)

  __super__.__init(self, 
                   width, height,
                   hfov, vfov, 
                   pixel_center_x, pixel_center_y)

  -- How to get to normalized coordinates
  self.units_per_pixel_x = math.tan(self.hfov*0.5)/self.center[1]
  self.units_per_pixel_y = math.tan(self.vfov*0.5)/self.center[2]
end


-- coords - coords in normalized coordinates, 0,0 center
-- angles (optional) - azimuth, elevation from 0,0 center of projection
function RectilinearProjection:coords_to_angles(coords, angles)
  angles = angles or torch.Tensor(coords:size())

  local azimuth = torch.atan(coords[1])
  angles[1] = azimuth
  angles[2] = torch.cmul(coords[2], torch.cos(azimuth)):atan() 

  return angles
end


-- angles - azimuth, elevation from 0,0 center of projection
-- coords (optional) - coords in normalized coordinates
function RectilinearProjection:angles_to_coords(angles, coords)
   coords = coords or torch.Tensor(angles:size())

  local azimuth   = angles[1]
  local elevation = angles[2]

  coords[1] = torch.tan(azimuth)
  coords[2] = torch.tan(elevation):cdiv(torch.cos(azimuth))

  return coords
end