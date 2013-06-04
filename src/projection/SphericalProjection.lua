local SphericalProjection = Class(projection.Projection)

-- CAREFUL: The Projection class stores the indexes for x and y in the
-- same height,width order as the underlying torch.Tensors()

function SphericalProjection:__init(width, height, 
                                      hfov, vfov, 
                                      pixel_center_x, pixel_center_y)

  __super__.__init(self, 
                   width, height,
                   hfov, vfov, 
                   pixel_center_x, pixel_center_y)

  -- How to get to normalized coordinates in this case just radians per pixel
  self.units_per_pixel_x = self.hfov/self.width
  self.units_per_pixel_y = self.vfov/self.height
end

-- override the angles_map
function SphericalProjection:angles_map(scale,hfov,vfov, hoffset, voffset)
   --   make map of angles
   scale  = scale or 1
   
   hfov   = hfov or self.hfov
   vfov   = vfov or self.vfov

   hoffset = hoffset or 0
   voffset = voffset or 0

   local mapw   = self.width * scale
   local maph   = self.height * scale
      
   local half_hfov = hfov * 0.5
   local half_vfov = vfov * 0.5
   
   local lambda = torch.linspace(hoffset-half_hfov,hoffset+half_hfov,mapw):resize(1,mapw):expand(maph,mapw)
   local phi    = torch.linspace(voffset-half_vfov,voffset+half_vfov,maph):resize(1,maph):expand(mapw,maph)
                                                                                                                                                     
   local angles = torch.Tensor(2,maph,mapw)

   angles[1]:copy(phi:t())
   angles[2]:copy(lambda)

   return angles

end


-- coords - coords in the unit sphere projection, 0,0 center
-- angles (optional) - azimuth, elevation from 0,0 center of projection
function SphericalProjection:coords_to_angles(coords, angles)
   angles = angles or torch.Tensor(coords:size())
   angles:copy(coords)
   return angles
end


-- angles - azimuth, elevation from 0,0 center of projection
-- coords (optional) - coords in the unit sphere projection
function SphericalProjection:angles_to_coords(angles, coords)
   coords = coords or torch.Tensor(angles:size())
   coords:copy(angles) 
   return coords
end