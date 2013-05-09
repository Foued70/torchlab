local CalibratedProjection = Class(projection.Projection)

-- uses parameters from opencv calibration
function CalibratedProjection:__init(width, height,
                                     fx, fy,
                                     pixel_center_x, pixel_center_y,
                                     radial_coeff, tangential_coeff)

   __super__.__init(self,
                    width, height,
                    nil, nil,
                    pixel_center_x, pixel_center_y)


   -- How to get to normalized coordinates
   self.units_per_pixel_x = 1/fx
   self.units_per_pixel_y = 1/fy

   -- Need to compute fov  from the "intrinsic parameters" we get from opencv
   self.hfov = 2 * math.atan2(pixel_center_x,fx)
   self.vfov = 2 * math.atan2(pixel_center_y,fy)

   self.radial_coeff     = radial_coeff
   self.tangential_coeff = tangential_coeff
end


-- coords - coords in normalized coordinates, 0,0 center
-- angles (optional) - azimuth, elevation from 0,0 center of projection
function CalibratedProjection:coords_to_angles(coords, angles)
   -- TODO: for now we have no reverse function from the opencv
   --       coefficients
   error("Not implemented")
end


-- angles - azimuth, elevation from 0,0 center of projection
-- coords (optional) - coords in normalized coordinates
function CalibratedProjection:angles_to_coords(angles, coords)
   coords = coords and coords:copy(angles) or angles:clone()

   -- the variables are named as they are in the opencv documentation:
   -- http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html

   -- opencv input is a perspective projection from world coordinates,
   -- so to get from angles (spherical coordinates) to perspective
   -- coordinates we apply a rectilinear transform.

   local azimuth   = angles[1]
   local elevation = angles[2]

   coords[1] = torch.tan(azimuth)
   coords[2] = torch.tan(elevation):cdiv(torch.cos(azimuth))

   local xpp = coords[1]
   local ypp = coords[2]


   local xp = angles[1]
   local yp = angles[2]

   local xp_sqr = torch.cmul(xp,xp)
   local yp_sqr = torch.cmul(yp,yp)

   local r_sqr = xp_sqr:clone()
   r_sqr:add(yp_sqr)

   local r_pow = r_sqr:clone()

   local radial_distortion = torch.ones(r_sqr:size())

   for ri = 1,self.radial_coeff:size(1)-1 do
      radial_distortion:add(r_pow * self.radial_coeff[ri])
      r_pow:cmul(r_sqr)
   end
   radial_distortion:add(r_pow * self.radial_coeff[-1])

   local xpyp = torch.cmul(xp,yp)


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
   xpp:add(yp_sqr)

   -- getting out of memory errors so make sure the locals are collected
   r_pow = nil
   radial_distortion = nil
   r_sqr = nil
   xp_sqr = nil
   yp_sqr = nil
   collectgarbage()

   return coords
end