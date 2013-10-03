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

end

function SphericalProjection:units_per_pixel_y()
   return self.vfov/self.height
end

function SphericalProjection:units_per_pixel_x()
   return self.hfov/self.width
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

   -- Images are stored row major so the image projections index
   -- height then width therefore the 2D angles are elevation,
   -- azimuth. Furthermore images have 0,0 (or 1,1) in the upper left
   -- corner, so z (elevation is inverted w/respect to our 3D z up.

   angles[1]:copy(phi:t()) -- elevation
   angles[2]:copy(lambda)  -- azimuth

   return angles

end


-- normalized_coords - normalized_coords in the unit sphere projection, 0,0 center
-- angles (optional) - azimuth, elevation from 0,0 center of projection
function SphericalProjection:normalized_coords_to_angles(normalized_coords, angles)
   angles = angles or torch.Tensor(normalized_coords:size())
   angles:copy(normalized_coords)
   return angles
end


-- angles - azimuth, elevation from 0,0 center of projection
-- normalized_coords (optional) - normalized_coords in the unit sphere projection
function SphericalProjection:angles_to_normalized_coords(angles, normalized_coords)
   normalized_coords = normalized_coords or torch.Tensor(angles:size())
   normalized_coords:copy(angles) 
   return normalized_coords
end
