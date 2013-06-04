local StereographicProjection = Class(projection.Projection)

-- CAREFUL: The Projection class stores the indexes for x and y in the
-- same height,width order as the underlying torch.Tensors()

-- The Stereographic Projection is a conformal projection from a point
-- on a sphere to a plane tangent to the sphere.  The default
-- projection is from a point on the back of a sphere to a plane
-- tangent touching the opposite side of the sphere.

-- see: http://mathworld.wolfram.com/StereographicProjection.html

function StereographicProjection:__init(width, height,
                                        hfov, vfov,
                                        pixel_center_x, pixel_center_y,
                                        lambda, phi)

   __super__.__init(self,
                    width, height,
                    hfov, vfov,
                    pixel_center_x, pixel_center_y)
   
   -- How to get to normalized coordinates
   self.units_per_pixel_x = math.tan(self.hfov*0.5)/width
   self.units_per_pixel_y = math.tan(self.vfov*0.5)/height
      
   self:set_lambda_phi(lambda, phi)

end

function StereographicProjection:set_lambda_phi(lambda,phi)
   self.tangent_point = self.tangent_point or torch.Tensor(2)

   self.tangent_point[1] = phi or 0
   self.tangent_point[2] = lambda or 0 

   -- make sure that the tangent point is expressed betwee -pi and pi
   projection.util.recenter_angles(self.tangent_point)
   
   self.phi1             = self.tangent_point[1]
   self.lambda0          = self.tangent_point[2]

   self.sin_lambda0 = math.sin(self.lambda0)
   self.cos_lambda0 = math.cos(self.lambda0)
   self.sin_phi1    = math.sin(self.phi1)
   self.cos_phi1    = math.cos(self.phi1)

end

-- coords - coords in normalized coordinates, 0,0 center
-- angles (optional) - azimuth, elevation from 0,0 center of projection
function StereographicProjection:coords_to_angles(coords, angles)
   angles = angles or torch.Tensor(coords:size())

   local y = coords[1]
   local x = coords[2]

   -- rho = sqrt(x^2 + y^2)
   local rho = x:clone()
   rho:cmul(x)  -- x^2
   local temp = y:clone()
   temp:cmul(y) -- y^2
   rho:add(temp):sqrt()

   -- c = 2*tan^-1(rho/2)
   local c    = rho:clone():mul(0.5):atan():mul(2)
   local cosc = c:clone():cos()
   local sinc = c:sin()
   c = nil

   -- latitude, phi, northing, how far north, south
   local elevation = angles[1]
   -- y * sinc * cos_phi / rho
   elevation:copy(y):cmul(sinc):mul(self.cos_phi1):cdiv(rho)
   -- cosc * sin_phi1
   temp:copy(cosc):mul(self.sin_phi1)
   -- asin ( (cosc * sin_phi) + ((y * sinc * cos_phi)/ rho) )
   elevation:add(temp):asin()

   -- longitude, lambda, easting, how far east west
   local azimuth = angles[2]
   -- x * sinc
   azimuth:copy(x):cmul(sinc)
   -- not going to use rho after this
   -- rho * cos_phi1 * cosc
   rho:mul(self.cos_phi1):cmul(cosc)
   -- - y * sin_phi1 * sinc
   temp:copy(y):mul(-self.sin_phi1):cmul(sinc)

   -- (rho * cos_phi * cosc ) + ( - y * sin_phi * sinc)
   rho:add(temp)
   -- lambda + atan((x * sinc) / (rho * cos_phi * cosc ) + ( - y * sin_phi * sinc))
   azimuth:atan2(rho):add(self.lambda0)

   -- resets angles to lie between -pi and pi
   projection.util.recenter_angles(angles)

   return angles
end


-- angles - azimuth, elevation from 0,0 center of projection
-- coords (optional) - coords in normalized coordinates
function StereographicProjection:angles_to_coords(angles, coords)
   coords = coords or torch.Tensor(angles:size())

   -- Elevation: latitude, phi, northing, how far north, south
   local elevation = angles[1]
   -- Azimuth: longitude, lambda, easting, how far east west
   local azimuth   = angles[2]

   local y = coords[1]
   local x = coords[2]

   local cos_phi = torch.cos(elevation)
   local sin_phi = torch.sin(elevation)

   -- sin(phi1) * sin(phi)
   local cosc = sin_phi:clone()
   cosc:mul(self.sin_phi1)

   -- cos_phi * cos(azimuth - lambda0)
   y:copy(azimuth)
   y:add(-self.lambda0)
   y:cos():cmul(cos_phi)


   -- 1 + (sin(phi1) * sin(phi)) + ( cos_phi * cos(azimuth - lambda0))
   cosc:add(torch.mul(y,self.cos_phi1)):add(1)

   x:copy(azimuth):add(-self.lambda0):sin():cmul(cos_phi):cdiv(cosc):mul(2)

   y:mul(-self.sin_phi1)
   y:add(sin_phi:mul(self.cos_phi1))
   y:cdiv(cosc):mul(2)

   return coords
end