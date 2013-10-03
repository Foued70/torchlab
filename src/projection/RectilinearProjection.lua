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

end

function RectilinearProjection:units_per_pixel_y()
   return math.tan(self.vfov*0.5)/self.center[1]
end

function RectilinearProjection:units_per_pixel_x()
   return math.tan(self.hfov*0.5)/self.center[2]
end


-- normalized_coords - normalized_coords in normalized coordinates, 0,0 center
-- angles (optional) - azimuth, elevation from 0,0 center of projection
function RectilinearProjection:normalized_coords_to_angles(normalized_coords, angles)
   angles = angles or torch.Tensor(normalized_coords:size())

   local azimuth = torch.atan(normalized_coords[2])
   angles[2] = azimuth
   angles[1] = torch.cmul(normalized_coords[1], torch.cos(azimuth)):atan()

   return angles
end


-- angles - azimuth, elevation from 0,0 center of projection
-- normalized_coords (optional) - normalized_coords in normalized coordinates
function RectilinearProjection:angles_to_normalized_coords(angles, normalized_coords)
   normalized_coords = normalized_coords or torch.Tensor(angles:size())

   local elevation = angles[1]
   local azimuth   = angles[2]

   normalized_coords[1] = torch.tan(elevation):cdiv(torch.cos(azimuth))
   normalized_coords[2] = torch.tan(azimuth)

   return normalized_coords
end
