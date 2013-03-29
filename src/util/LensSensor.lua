local projection = require "util.projection"

local pi = math.pi
local piover2 = math.pi * 0.5
local r2d = 180 / pi
local d2r = pi / 180

-- stores fixed information about lens+sensors which we glean from spec sheets and elsewhere.
local LensSensor = torch.class("LensSensor")

LensSensor.default_lens_sensor = require "util.lens_sensor_types"

-- combine image and lens data into a single object

function LensSensor:__init(lens_sensor,img)

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

   if img then
      self:add_image(img)
   else
      -- compute some values off sensor data
      local w = self.sensor_w
      local h = self.sensor_h
      local aspect_ratio = w/h
      local diag_norm = math.sqrt(w*w + h*h) / (2 * self.focal)
      local horz_norm, vert_norm = projection.derive_hw(diag_norm,aspect_ratio)
      printf(" -- normalized : diag: %f h: %f v: %f", diag_norm, horz_norm, vert_norm)

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

      printf(" -- degress: d: %2.4f h: %2.4f v: %2.4f",
         dfov*r2d, hfov*r2d,vfov*r2d)
      self.aspect_ratio = aspect_ratio

      self.fov  = dfov
      self.hfov = hfov
      self.vfov = vfov

      self.diagonal_normalized   = diag_norm
      self.horizontal_normalized = horz_norm
      self.vertical_normalized   = vert_norm

   end

   return self

end

function LensSensor:add_image(img)

   -- recompute almost everything based on the image pixels as this is
   -- better than basing measurements on spec sheets.
   local imgw = img:size(3)
   local imgh = img:size(2)

   local camw = 0
   local camh = 0

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
   -- than the sensor width or height.
   if self.crop_radius then
      -- I don't understand why we divide by the vertical fov (in most
      -- cases we have square pixels, aspect ratio is 1).
      diag_norm   = self.crop_radius / vfocal_px
   end

   printf(" -- normalized : diag: %f h: %f v: %f",
      diag_norm,
      horz_norm, vert_norm)

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

   printf(" -- degress: d: %2.4f h: %2.4f v: %2.4f",
      dfov*r2d, hfov*r2d,vfov*r2d)
   self.image_w      = imgw -- px
   self.image_h      = imgh -- px
   self.aspect_ratio = aspect_ratio

   self.center_x = cx -- px
   self.center_y = cy -- px
   self.hfocal_px = hfocal_px
   self.vfocal_px = vfocal_px

   self.fov  = dfov
   self.hfov = hfov
   self.vfov = vfov

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
   printf("r_map: max: %f min: %f", r_map:max(), r_map:min())
   
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
   printf("Masking %d/%d lookups (out of bounds)", mask:sum(),mask:nElement())

   -- +++++++
   -- (6) convert the x and y index into a single 1D offset (y * stride + x)
   -- +++++++

   local index_map = projection.make_index(xmap,ymap,mask)

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


return LensSensor
