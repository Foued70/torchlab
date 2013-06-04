local RectilinearProjection = Class(projection.Projection)

-- CAREFUL: The Projection class stores the indexes for x and y in the
-- same height,width order as the underlying torch.Tensors()

function RectilinearProjection:__init(width, height,
                                      hfov, vfov,
                                      pixel_center_x, pixel_center_y)

   __super__.__init(self,
                    width, height,
                    hfov, vfov,
                    pixel_center_x, pixel_center_y)

   -- How to get to normalized coordinates
   self.units_per_pixel_y = math.tan(self.vfov*0.5)/self.center[1]
   self.units_per_pixel_x = math.tan(self.hfov*0.5)/self.center[2]

end


-- coords - coords in normalized coordinates, 0,0 center
-- angles (optional) - azimuth, elevation from 0,0 center of projection
function RectilinearProjection:coords_to_angles(coords, angles)
   angles = angles or torch.Tensor(coords:size())

   local azimuth = torch.atan(coords[2])
   angles[2] = azimuth
   angles[1] = torch.cmul(coords[1], torch.cos(azimuth)):atan()

   return angles
end


-- angles - azimuth, elevation from 0,0 center of projection
-- coords (optional) - coords in normalized coordinates
function RectilinearProjection:angles_to_coords(angles, coords)
   coords = coords or torch.Tensor(angles:size())

   local elevation = angles[1]
   local azimuth   = angles[2]

   coords[1] = torch.tan(elevation):cdiv(torch.cos(azimuth))
   coords[2] = torch.tan(azimuth)

   return coords
end