local EquirectangularProjection = Class(data.Projection)

-- coords - coords in the unit sphere projection, 0,0 center
-- angles (optional) - azimuth, elevation from 0,0 center of projection
function coords_to_angles(coords, angles)
   angles = angles or torch.Tensor(coords:size())
   angles:copy(coords)
   return angles
end


-- angles - azimuth, elevation from 0,0 center of projection
-- coords (optional) - coords in the unit sphere projection
function angles_to_coords(angles, coords)
   coords = coords or torch.Tensor(angles:size())
   coords:copy(angles)

   return coords
end