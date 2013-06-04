local CalibratedProjection = Class(projection.Projection)

-- uses parameters from opencv calibration
function CalibratedProjection:__init(width, height,
                                     fx, fy,
                                     pixel_center_x, pixel_center_y,
                                     radial_coeff, tangential_coeff)

   -- some careful work arounds to reuse the Projection class
   __super__.__init(self,
                    width, height,
                    nil, nil,
                    pixel_center_x+1, pixel_center_y+1)

   -- Need to compute fov  from the "intrinsic parameters" we get from opencv
   self.vfov = 2 * math.atan2(self.center[1],fy)
   self.hfov = 2 * math.atan2(self.center[2],fx)

   -- How to get to normalized coordinates
   self.units_per_pixel_x = 1/fx
   self.units_per_pixel_y = 1/fy


   self.radial_coeff     = radial_coeff
   self.tangential_coeff = tangential_coeff
end


-- normalized_coords - coords in normalized coordinates, 0,0 center
-- angles (optional) - azimuth, elevation from 0,0 center of projection
function CalibratedProjection:normalized_coords_to_angles(normalized_coords, angles)
   -- TODO: for now we have no reverse function from the opencv
   --       coefficients
   error("Not implemented")
end


-- angles - azimuth, elevation from 0,0 center of projection
-- normalized_coords (optional) - coords in normalized coordinates
function CalibratedProjection:angles_to_normalized_coords(angles, normalized_coords)
   normalized_coords = normalized_coords and normalized_coords:copy(angles) or angles:clone()

   -- the variables are named as they are in the opencv documentation:
   -- http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html

   -- opencv input is a perspective projection from world coordinates,
   -- so to get from angles (spherical coordinates) to perspective
   -- coordinates we apply a rectilinear transform.


   local elevation = angles[1]
   local azimuth   = angles[2]

   normalized_coords[1] = torch.tan(elevation):cdiv(torch.cos(azimuth))
   normalized_coords[2] = torch.tan(azimuth)

   -- then we apply the opencv distortion from perfect rectilinear to
   -- actual image plane rectilinear.
   local yp = normalized_coords[1]
   local xp = normalized_coords[2]
   
   local xpyp = torch.cmul(xp,yp)

   local xp_sqr = torch.cmul(xp,xp)
   local yp_sqr = torch.cmul(yp,yp)

   local r_sqr = xp_sqr:clone()
   r_sqr:add(yp_sqr)

   local r_pow = r_sqr:clone()

   local radial_distortion = torch.ones(r_sqr:size())

   -- don't use these variable names anymore
   xp = nil 
   yp = nil

   local ypp = normalized_coords[1]
   local xpp = normalized_coords[2]

   for ri = 1,self.radial_coeff:size(1)-1 do
      radial_distortion:add(r_pow * self.radial_coeff[ri])
      r_pow:cmul(r_sqr)
   end
   radial_distortion:add(r_pow * self.radial_coeff[-1])

   

   -- x'( 1 + k1*r^2 + k2*r^4 + k3*r^6)
   xpp:cmul(radial_distortion)
   -- + 2 * p1 * x'y'
   xpp:add(xpyp * ( 2 * self.tangential_coeff[1]))
   -- + p2 * (r^2 + 2*x'^2)
   xp_sqr:mul(2):add(r_sqr):mul(self.tangential_coeff[2])
   xpp:add(xp_sqr)

   -- x'( 1 + k1*r^2 + k2*r^4 + k3*r^6)
   ypp:cmul(radial_distortion)
   -- + 2 * p2 * x'y'
   ypp:add(xpyp * ( 2 * self.tangential_coeff[2]))
   -- + p1 * (r^2 + 2*y'^2)
   yp_sqr:mul(2):add(r_sqr):mul(self.tangential_coeff[1])
   ypp:add(yp_sqr)

   -- getting out of memory errors so make sure the locals are collected
   r_pow = nil
   radial_distortion = nil
   r_sqr = nil
   xp_sqr = nil
   yp_sqr = nil
   collectgarbage()

   return normalized_coords
end