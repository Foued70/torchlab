local projection = util.projection
local geom = util.geom

local pi = math.pi
local piover2 = math.pi * 0.5
local r2d = 180 / pi
local d2r = pi / 180

-- stores fixed information about lens+sensors which we glean from spec sheets and elsewhere.
local LensSensor = Class()

LensSensor.default_lens_sensor = require "util.lens_sensor_types"

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

   -- Currently unused, crop_radius would allow us to look at a region
   -- inside image boundaries such as when the image circle is less
   -- than the sensor width or height (not full frame)
   if self.crop_radius then
      -- Divide by the vertical fov b/c that is the shorter dimension
      -- and closer to the circle in a rectangle, but does raise a
      -- question about the aspect ratio.
      diag_norm   = self.crop_radius / vfocal_px
   end

   log.tracef(" -- normalized : diag: %f h: %f v: %f",
      diag_norm, horz_norm, vert_norm)

   -- FIXME clean this up. Special case for calibration
   -- reset center based on the calibration
   if (self.lens_type:gmatch("scaramuzza")()) then   
      -- calibrated center in raw image space
      cx = imgw * (self.cal_xc / self.cal_width)
      cy = imgh * (self.cal_yc / self.cal_height)
   end

   local dfov 
   if (self.lens_type == "scaramuzza_r2t") then
      dfov = 
         projection.compute_diagonal_fov(diag_norm*self.cal_focal_px,
                                         self.lens_type, self.pol)
   else
      dfov = 
         projection.compute_diagonal_fov(diag_norm,
                                         self.lens_type, self.pol)
   end

   local vfov,hfov = projection.derive_hw(dfov,aspect_ratio)

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

-- Maps a camera image + lens to a unit sphere (azimuth and elevation)
-- or through sphere to other projections.
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
--  + sphere
--  + rectilinear
--  + cylindrical
--  + cylindrical_vert
function LensSensor:make_projection_map (proj_type,scale,debug)

   if not proj_type then
      proj_type = "sphere"
   end

   if not scale then
      scale = 1
   end

   local lens_type = self.lens_type

   -- +++++
   -- (0).a image dimensions
   -- +++++
   local imgw = self.image_w
   local imgh = self.image_h

   local fov  = self.fov
   local hfov = self.hfov
   local vfov = self.vfov
   local aspect_ratio = self.aspect_ratio

   -- +++++
   -- find dimension of the map
   -- +++++
   local mapw = math.floor(imgw * scale)
   local maph = math.floor(mapw * (vfov/hfov))

   -- +++++
   -- (1) create map of diagonal angles from optical center
   -- +++++

   local lambda, phi, 
   theta_map, output_map, 
   mapw, maph = projection.projection_to_sphere(fov,hfov,vfov, 
                                                mapw,maph,aspect_ratio, 
                                                proj_type)
      
   -- +++++ 
   -- (2) replace theta for each entry in map with r 
   -- +++++
   local r_map =
      projection.sphere_to_camera(theta_map, lens_type, self.invpol)
   
   -- +++++
   -- (3) make the ratio (new divided by old) by which we scale the
   --     pixel offsets (in normalized coordinates) in the output
   --     image to coordinates in the original image.
   -- +++++
   r_map:cdiv(output_map)
   
   -- +++++
   -- (4) create map of x and y indices into the original image for
   --     each location in the output image.
   -- +++++
   local xmap = lambda:repeatTensor(maph,1)
   local ymap = phi:repeatTensor(mapw,1):t():contiguous()

   -- +++++++
   -- (4) put xmap and ymap from normalized coordinates to pixel coordinates
   -- +++++++
   xmap:cmul(r_map):mul(self.hfocal_px):add(self.center_x)
   ymap:cmul(r_map):mul(self.vfocal_px):add(self.center_y)

   -- +++++++
   -- (5) make mask for out of bounds values
   -- +++++++

   local mask = projection.make_mask(xmap,ymap,imgw,imgh)
   log.tracef("Masking %d/%d lookups (out of bounds)", mask:sum(),mask:nElement())
   -- +++++++
   -- (6) convert the x and y index into a single 1D offset (y * stride + x)
   --     Note: stride is the final dimension of the original image.
   -- +++++++

   local index_map = projection.make_index(xmap,ymap,mask,imgw)

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
      
   elseif (pt_type == "pixel_space") then
      
      normalized_pts[{{},1}]:add(-self.center_x)
      normalized_pts[{{},2}]:add(-self.center_y)
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
      projection.compute_diagonal_fov(d, self.lens_type, self.pol)

   -- convert diagonal angle to spherical coordinates
   local elevation,azimuth = projection.derive_hw(diagonal_angles,self.aspect_ratio) 
   local spherical_angles = torch.Tensor(elevation:size(1),2)
   spherical_angles[{{},1}] = azimuth
   spherical_angles[{{},2}] = elevation
   spherical_angles:cmul(normalized_pts) -- put the sign

   if (out_type == "uc") or (out_type == "unit_cartesian") then
      return geom.spherical_coords_to_unit_cartesian(spherical_angles)
   else
      return spherical_angles
   end
end