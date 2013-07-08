local projection = projection.util

Class()

local pi = math.pi
local piover2 = math.pi * 0.5
local r2d = 180 / pi
local d2r = pi / 180

-- thoby constants
local k1 = 1.47
local k2 = 0.713

-- rectilinear max fov in radians
local max_rad_rectilinear = 1

-- stores fixed information about lens+sensors which we glean from spec sheets and elsewhere.
local LensSensor = Class()

LensSensor.default_lens_sensor = require 'projection.lens_sensor_types'
--LensSensor.default_lens_sensor = projection.lens_sensor_types

-- combine image and lens data into a single object

function LensSensor:__init(lens_sensor,...)

   if not lens_sensor then
      error("Pass lens + sensor type as first arg")
   end
   if type(lens_sensor) == "string" then
      lens_sensor = self.default_lens_sensor[lens_sensor]
   end
   if (not lens_sensor) or (type(lens_sensor) ~= "table") then
      error("LensSensorType not recognized")
   end

   -- copy static data from lens_sensor
   for key, val in pairs(lens_sensor) do
      self[key] = val
   end

   -- FIXME clean this up. Special case for calibration
   if (self.lens_type:gmatch("scaramuzza")()) then
      self.cal_focal_px = self.focal * (self.cal_height / self.sensor_h)
      -- revert all calibration coeffs to normalized coordinates
      self.invpol:mul(1/self.cal_focal_px)
      self.cal_xc     = self.cal_xc / self.cal_focal_px
      self.cal_yc     = self.cal_yc / self.cal_focal_px
      self.cal_width  = self.cal_width / self.cal_focal_px
      self.cal_height = self.cal_height / self.cal_focal_px
   else 
      self.cal_focal_px = 1
   end

   if ... then
      self:add_image(...)
   end

   return self

end

function LensSensor:add_image(...)
   
   -- recompute almost everything based on the image pixels as this is
   -- better than basing measurements on spec sheets.
   local imgw,imgh
   local camw = 0
   local camh = 0

   local args = {...}
   local nargs = #args
   if (nargs == 1) and (type(args[1]) == "userdata") then 
      local img = args[1]
      imgw = img:size(3)
      imgh = img:size(2)
   elseif (nargs == 2) then 
      imgw = args[1]
      imgh = args[2]
   else 
      error("Don't understand arguments")
   end

   -- handle vertical images
   if imgw > imgh then
      camw = self.sensor_w
      camh = self.sensor_h
   else
      camh = self.sensor_w
      camw = self.sensor_h
   end
   
   -- offset to optical center of image (px)
   local cx = (imgw + 1) * 0.5
   local cy = (imgh + 1) * 0.5

   -- horizontal and vertical resolution (px / mm)
   local hres = imgw / camw -- px/mm
   local vres = imgh / camh -- px/mm

   local aspect_ratio = camw / camh

   -- focal length in pixels
   local hfocal_px = self.focal * hres
   local vfocal_px = self.focal * vres

   -- normalized coordinates (px / px ==> dimensionless)
   local horz_norm = cx / hfocal_px
   local vert_norm = cy / vfocal_px

   -- maximum distance in normalized image coordinates which we want
   -- to project from self (default is the diagonal).  Note this is
   -- on the image plane therefore the normal euclidean pythagorean
   -- theorem works.
   local diag_norm = math.sqrt(horz_norm*horz_norm +
                               vert_norm*vert_norm)

   local dfov,hfov,vfov
   
   log.tracef(" -- normalized : diag: %f h: %f v: %f",
      diag_norm, horz_norm, vert_norm)

   -- TODO clean this up. Special case for calibrated lenses.
   -- reset center based on the calibration
   if self.lens_type:gmatch("scaramuzza")() then   
      -- calibrated center in raw image space
      cx = imgw * (self.cal_xc / self.cal_width)
      cy = imgh * (self.cal_yc / self.cal_height)
   end

   if self.lens_type:gmatch("opencv")() then
      -- TODO: check that we keep track of the different focal points
      -- all the way through
      if imgw > imgh then
         hfocal_px = self.fx
         vfocal_px = self.fy
         horz_norm = self.cx / self.fx
         vert_norm = self.cy / self.fy
         hfov = math.atan(horz_norm)
         vfov = math.atan(vert_norm)
      else -- image is vertical
         hfocal_px = self.fy
         vfocal_px = self.fx
         horz_norm = self.cy / self.fy
         vert_norm = self.cx / self.fx
         hfov = math.atan(horz_norm)
         vfov = math.atan(vert_norm)
      end
      diag_norm = math.sqrt(cx*cx + cy*cy)
      dfov = math.atan(diag_norm)
         
   else
      if (self.lens_type == "scaramuzza_r2t") then
         dfov = self:radial_distance_to_angle(diag_norm*self.cal_focal_px)
      else
         dfov = self:radial_distance_to_angle(diag_norm)
         
      end
      
      vfov,hfov = projection.diagonal_to_height_width(dfov,aspect_ratio)
   end
   
   log.tracef(" -- degress: d: %2.4f h: %2.4f v: %2.4f",
      dfov*r2d, hfov*r2d,vfov*r2d)

   self.image_w      = imgw -- px
   self.image_h      = imgh -- px
   self.inv_image_w  = 1/imgw
   self.inv_image_h  = 1/imgh
   self.aspect_ratio = aspect_ratio

   self.center_x = cx -- px
   self.center_y = cy -- px
   self.hfocal_px = hfocal_px
   self.vfocal_px = vfocal_px

   self.fov  = dfov
   self.hfov = hfov
   self.vfov = vfov
   self.inv_hfov = 1/hfov
   self.inv_vfov = 1/vfov
   
   -- resolution dependent version of hfov, vfov, inv_hfov,
   -- inv_fov. used in azimuth and elevation calcs TODO: rewrite
   -- azimuth and elevation to use hfov, vfov, etc instead
   self.rad_per_px_x = 2*hfov/imgw
   self.rad_per_px_y = 2*vfov/imgh
   self.px_per_rad_x = 1/self.rad_per_px_x
   self.px_per_rad_y = 1/self.rad_per_px_y

   self.diagonal_normalized   = diag_norm
   self.horizontal_normalized = horz_norm
   self.vertical_normalized   = vert_norm
end

function LensSensor:radial_distance_to_angle(radial_normalized,debug)
   --,lens_type,params,debug)
   local angle
   local convert = false
   local lens_type = self.lens_type

   if (type(radial_normalized) == "number") then 
      radial_normalized = torch.Tensor({radial_normalized})
      convert = true
   end
   if (lens_type == "rectilinear") then
      angle = torch.atan(radial_normalized)   -- diag in rad
   elseif (lens_type == "scaramuzza_r2t") then
      if debug then
         log.trace(" -- using scaramuzza calibration")
      end
      local pol = self.pol
      local d2 = radial_normalized:clone()
      angle = torch.Tensor(d2:size()):fill(pol[-1])
      for i = pol:size(1)-1,1,-1 do 
         angle = angle + d2 * pol[i]
         d2:cmul(radial_normalized)
      end
      -- using ideal thoby to compute fov for scaramuzza's original
      -- code which does not solve for theta. Can remove this eventually.
   elseif (lens_type == "thoby") or (lens_type == "scaramuzza") then
      if debug then 
         log.trace(" -- using lens type: thoby")
      end
      -- thoby : theta = asin((r/f)/(k1 * f))/k2
      if (radial_normalized:max() > k1) then 
         log.error("diagonal too large for thoby")
      else
         angle = torch.asin(radial_normalized/k1)/k2
      end
   elseif (lens_type == "equal_angle") then
      angle = radial_normalized

   elseif (lens_type =="equal_area") then
      if( radial_normalized:max() <= 2 ) then
         angle = 2 * torch.asin( 0.5 * radial_normalized )
      end
      if( torch.sum(angle:eq(0)) > 0) then
         error( "equal-area FOV too large" )
      end

   elseif (lens_type == "stereographic") then
      angle = 2 * torch.atan(radial_normalized*0.5 )

   elseif (lens_type == "orthographic") then
      if( radial_normalized:max() <= 1 ) then
         angle = torch.asin( radial_normalized )
      end
      if( torch.sum(angle:eq(0)) > 0) then
         error( "orthographic FOV too large" )
      end

   else
      error("don't understand self lens model requested")
   end
   if convert then 
      angle = angle[1]
   end
   return angle
end

-- theta = f(r) functions 
-- 
-- take equal steps in 2 dimensions of the output map r
-- 
-- Based on final projection type create a quantized map of angles
-- (dimensions mapw x maph) which need to be looked up in the original
-- image.
-- 
-- Creates four tensors:
-- 
--    self.lambda      : angles row
--    sel.phi          : angles col
--    self.theta_map   : 2D map of diagonal distances in angles
--    self.output_map  : 2D map of diagonals in output space
-- 
function LensSensor:output_to_equirectangular_map(proj_type,maph,mapw)

   -- TODO: add other projection types
   -- TODO this should be broken out into separate classes as Dustin
   -- is doing.  Totally see it now.
   --    if (self.lens_type == "rectilinear") then
   --       -- rectilinear : (1/f) * r' = tan(theta)
   --       r_map:tan()
   --    elseif (self.lens_type == "stereographic") then
   --       --  + stereographic             : r = 2 * f * tan(theta/2)
   --       r_map:mul(0.5):tan():mul(2)
   --    elseif (self.lens_type == "orthographic") then
   --       --  + orthographic              : r = f * sin(theta)
   --       r_map:sin()
   --    elseif (self.lens_type == "equisolid") then
   --       --  + equisolid                 : r = 2 * f * sin(theta/2)
   --       r_map:mul(0.5):sin():mul(2)
      
   local lambda, phi, output_map, theta_map
   local fov, vfov, hfov, drange, hrange, vrange
   if (proj_type == "rectilinear") then
      fov = self.fov
      -- limit the fov to roughly 120 degrees
      if fov > max_rad_rectilinear then
         fov = max_rad_rectilinear
         vfov,hfov = projection.diagonal_to_height_width(fov,self.aspect_ratio)
      end
      -- set up size of the output table
      drange = torch.tan(fov)
      vrange,hrange = projection.diagonal_to_height_width(drange,self.aspect_ratio)
      -- equal steps in normalized coordinates
      lambda     = torch.linspace(-hrange,hrange,mapw)
      phi        = torch.linspace(-vrange,vrange,maph)
      output_map = projection.make_pythagorean_map(lambda,phi)
      theta_map  = output_map:clone():atan()
   else
      -- Default projection : equirectangular or "plate carree". Create
      -- horizontal (lambda) and vertical angles (phi) x,y lookup in
      -- equirectangular map indexed by azimuth and elevation from
      -- -radians,radians at resolution mapw and maph.  Equal steps in
      -- angles.
      lambda     = torch.linspace(-self.hfov,self.hfov,mapw)
      phi        = torch.linspace(-self.vfov,self.vfov,maph)
      output_map = projection.make_pythagorean_map(lambda,phi)
      theta_map  = output_map
   end

   -- this is the output (stored in LensSensor)
   self.lambda     = lambda
   self.phi        = phi
   self.theta_map  = theta_map
   self.output_map = output_map

end

-- r = f(theta) functions
-- Note: a calibrated camera is a type of projection
function LensSensor:equirectangular_map_to_input()
   if not self.theta_map then 
      error("call output_to_equirectangular_map first")
   end

   local r_map = self.theta_map:clone() -- copy

   if (self.lens_type == "thoby") then
      -- thoby       : (1/f) * r' = k1 * sin(k2*theta)
      r_map:mul(k2):sin():mul(k1)
   elseif (self.lens_type == "scaramuzza") then
      local theta     = self.theta_map:clone()
      -- we use a positive angle from optical center, but scaramuzza
      -- uses a negative offset from focal point.
      theta:add(-piover2) 
      local theta_pow = theta:clone()
      local invpol    = self.invpol
      r_map:fill(invpol[-1]) -- rho = invpol[0]
      
      for i = params:size(1)-1,1,-1 do          
         r_map:add(theta_pow * invpol[i]) -- coefficients of inverse poly.
         theta_pow:cmul(theta)            -- powers of theta
      end 
   elseif self.lens_type:gmatch("scaramuzza_r2t") then
      local theta_pow = self.theta_map:clone()
      local invpol = self.invpol
      r_map:fill(invpol[-1]) -- rho = invpol[0]
      
      for i = invpol:size(1)-1,1,-1 do          
         r_map:add(theta_pow * invpol[i]) -- coefficients of inverse poly.
         theta_pow:cmul(self.theta_map)   -- powers of theta
      end 
   else
      log.error("don't understand lens_type")
      return nil
   end
   return r_map
end

-- Maps a camera image + lens to equirectangular map (azimuth and elevation)
-- or through this map to other projections.
-- 
-- Output is a map structure with a lookup table which can be used to
-- generate the projection from the original image.
-- 
-- Lens types:
-- 
-- - Ideal
--  + rectilinear (perspective) : r = f * tan(theta)
--  + stereographic             : r = 2 * f * tan(theta/2)
--  + orthographic              : r = f * sin(theta)
--  + equal_area (equisolid)    : r = 2 * f * sin(theta/2)
--  + thoby (fisheye)           : r = k1 * f * sin(k2*theta)
--  + equal_angle (eqidistant)  : r = f * theta  (for unit sphere f = 1)
-- 
-- - Calibrated
--  + scaramuzza (OCamLib)
--  + scaramuzza_r2t (modified to return theta's directly)
-- 
-- Projection types:
-- 
--  + equirectangular
--  + rectilinear
function LensSensor:make_projection_map (proj_type,scale,debug)

   if not proj_type then
      proj_type = "equirectangular"
   end

   if not scale then
      scale = 1
   end

   local lens_type = self.lens_type

   -- +++++
   -- (0.1) image dimensions
   -- +++++
   local imgw = self.image_w
   local imgh = self.image_h

   local fov  = self.fov
   local hfov = self.hfov
   local vfov = self.vfov
   local aspect_ratio = self.aspect_ratio -- w/h

   -- +++++
   -- (0.2) find dimension of the map
   -- +++++
   local mapw = math.floor(imgw * scale)
   local maph = math.floor(mapw / aspect_ratio)


   -- +++++
   -- (1) create map of diagonal angles from optical center, this is
   --     the output projection. 
   -- +++++
   
   self:output_to_equirectangular_map(proj_type,maph,mapw)

   -- +++++
   -- (2) create map from equirectangular to the input pixels
   -- +++++

   local map, xmap, ymap
      
   -- +++++
   -- (2) create 2D map of x and y indices into the original image for
   --     each location in the output image.
   -- +++++
   map  = torch.Tensor(2,maph,mapw)
   map[1]:copy(self.lambda:reshape(1,mapw):expand(maph,mapw))
   map[2]:copy(self.phi:repeatTensor(mapw,1):t())
      
   xmap = map[1]
   ymap = map[2]
   
   
   if self.lens_type:gmatch("opencv")() then

      -- opencv produces the perfect rectilinear projection so map the
      -- equirectangular lookup table to rectilinear before further
      -- processing.
      -- self.lambda:tan()
      -- self.phi:tan()

      -- recompute the theta's
      -- self.theta_map = projection.make_pythagorean_map(self.lambda,self.phi)

      -- opencv uses x and y terms explicitly as well as the diagonal
   
      -- r in opencv equations is 
      local theta_sqr = self.theta_map:clone():cmul(self.theta_map)

      local temp = theta_sqr:clone()
      
      local radial_distortion = torch.ones(maph,mapw)

      for ri = 1,self.radial_coeff:size(1)-1 do 
         radial_distortion:add(temp * self.radial_coeff[ri])
         temp:cmul(theta_sqr)
      end
      radial_distortion:add(temp * self.radial_coeff[-1])

      local xpyp = torch.cmul(xmap,ymap)

      local temp = torch.cmul(xmap,xmap)

      -- x'( 1 + k1*r^2 + k2*r^4 + k3*r^6)
      xmap:cmul(radial_distortion)
      -- + 2 * p1 * x'y'
      xmap:add(xpyp * ( 2 * self.tangential_coeff[1]))
      -- + p2 * (r^2 + 2*x'^2)
      temp:mul(2):add(theta_sqr):mul(self.tangential_coeff[2])
      xmap:add(temp)

      local temp = torch.cmul(ymap,ymap)

      -- x'( 1 + k1*r^2 + k2*r^4 + k3*r^6)
      ymap:cmul(radial_distortion)
      -- + 2 * p2 * x'y'
      xmap:add(xpyp * ( 2 * self.tangential_coeff[2]))
      -- + p1 * (r^2 + 2*y'^2)
      temp:mul(2):add(theta_sqr):mul(self.tangential_coeff[1])
      xmap:add(temp)


      temp = nil
      radial_distortion = nil
      theta_sqr = nil
      collectgarbage()
      
   else
      -- most projections express the distortion along the diagonal


      -- +++++ 
      -- (3) transform xmap and ymap to lookups through camera distortion
      -- +++++

      -- (3.1) replace theta (diagonal angle) for each entry in map with r 
      local r_map = self:equirectangular_map_to_input()
      
      -- (3.2) make the ratio (new divided by old) by which we scale the
      --       pixel offsets (in normalized coordinates) in the output
      --       image to coordinates in the original image.
      r_map:cdiv(self.output_map)
      
      -- (3.3) put xmap and ymap from normalized coordinates to pixel
      --       coordinates 
      xmap:cmul(r_map)
      ymap:cmul(r_map)
   end
   
   -- +++++++ 
   -- (4) Apply the camera matrix to return normalized coordinates to
   -- image pixel coordinates.
   -- +++++++ 
   xmap:mul(self.hfocal_px):add(self.center_x)
   ymap:mul(self.vfocal_px):add(self.center_y)
   
   -- +++++++
   -- (5) make mask for out of bounds values
   -- +++++++ 
   local mask = projection.make_mask(map,imgw,imgh)
   log.tracef("Masking %d/%d lookups (out of bounds)", mask:sum(),mask:nElement())

   -- +++++++
   -- (6) convert the x and y index into a single 1D offset (y * stride + x)
   --     Note: stride is the final dimension of the original image.
   -- +++++++
   local index_map = projection.make_index(map,mask,imgw)

   -- +++++++
   -- (7) output map object
   -- +++++++

   projection_map = {
      lookup_table     = index_map, -- 1D LongTensor for fast lookups
      mask             = mask,      -- ByteTensor invalid locations marked with 1
      radial_distance  = theta_map, -- map of distances from optical center (theta)
      height           = maph,      -- height of the output map
      width            = mapw,      -- width of the output map
      dfov             = fov,       -- diagonal field of view
      hfov             = hfov,      -- horizontal field of view
      vfov             = vfov       -- vertical field of view
   }

   return projection_map

end

-- Input: 
-- 
--   + img_pts: Nx2 tensor of points in image space
-- 
--   + pt_type: type of points.
-- 
--   - "uv" as output from opengl center of image at 0,0 x and y between -1,1
-- 
--   - "pixel_space" the offsets in pixels of raw image, (0,0) in
--      upper left corner.
-- 
-- Output: world 
-- 
--     - angles (default) azimuth and elevation
-- 
--     - "uc" or "unit_cartesian" unit cartesian vectors 
-- 
function LensSensor:image_coords_to_angles (img_pts, pt_type, out_type)
   local normalized_pts = img_pts:clone()

   -- put points in normalized coordinates. 
   if (pt_type == "uv") then
      -- put image space -1,1 in x and -1,1 in y into normalized
      -- coordinates computed when calibrating the lens.
      -- 1) apply the calibrated center to adjust position of 0,0
      local xoff = -1 + (2 * self.center_x / self.image_w)
      local yoff = -1 + (2 * self.center_y / self.image_h)
      -- 2) scale
      normalized_pts[{{},1}]:add(xoff):mul(self.horizontal_normalized)
      normalized_pts[{{},2}]:add(yoff):mul(self.vertical_normalized)
      
    -- FIXME -- the following is placeholder
   elseif (pt_type == "pixel_space") then
      -- 1) move 0,0 to optical center of image
      normalized_pts[{{},1}]:add(-self.center_x)
      normalized_pts[{{},2}]:add(-self.center_y)
      -- 2) transform into normalized coordinates
      normalized_pts[{{},1}]:mul(self.horizontal_normalized / self.image_w)
      normalized_pts[{{},2}]:mul(self.vertical_normalized / self.image_h)
      
   else -- normalized
      -- do nothing
      if debug then 
         print("-- points already in normalized coordinates") 
      end
   end

   -- compute diagonal distances
   local xsqr = normalized_pts[{{},1}]:clone()
   xsqr:cmul(xsqr)
   local ysqr = normalized_pts[{{},2}]:clone()
   ysqr:cmul(ysqr)
   local d = torch.add(xsqr,ysqr)
   d:sqrt()
   if self.cal_focal_px then 
      d:mul(self.cal_focal_px) -- put d in pixel coords
   end
   -- keep track of the sign
   normalized_pts:sign()

   -- apply the lens transform
   local diagonal_angles = 
      self:radial_distance_to_angle(d, self.lens_type, self.pol)

   -- convert diagonal angle to equirectangular (spherical) angles
   local elevation,azimuth = projection.diagonal_to_height_width(diagonal_angles,self.aspect_ratio) 
   local spherical_angles = torch.Tensor(elevation:size(1),2)
   spherical_angles[{{},1}] = azimuth
   spherical_angles[{{},2}] = elevation
   spherical_angles:cmul(normalized_pts) -- put the sign

   if (out_type == "uc") or (out_type == "unit_cartesian") then
      return geom.util.spherical_angles_to_unit_cartesian(spherical_angles)
   else
      return spherical_angles
   end
end