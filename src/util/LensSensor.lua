local projection = require "util.projection"

local pi = math.pi
local piover2 = math.pi * 0.5
local r2d = 180 / pi
local d2r = pi / 180

-- thoby constants
local k1 = 1.47
local k2 = 0.713


-- stores fixed information about lens+sensors which we glean from spec sheets and elsewhere.
local LensSensor = torch.class("LensSensor")

LensSensor.default = {
   nikon_D800E_w18mm = {

      name     = "Nikon D800E with 18mm",
      
      -- copied from exif info
      sensor_w = 35.9, -- mm
      sensor_h = 24.0, -- mm
      focal    = 18, -- mm

      lens_type = "rectilinear",
      
   },

   nikon_D5100_w10p5mm = {
      
      name     = "Nikon D5100 with 10.5mm",
      
      -- copied from exif info
      sensor_w = 23.6, -- mm
      sensor_h = 15.6, -- mm
      
      -- from http://michel.thoby.free.fr/Blur_Panorama/Nikkor10-5mm_or_Sigma8mm/Sigma_or_Nikkor/Comparison_Short_Version_Eng.html
      
      -- From my own experimental measurement, the mapping function of
      -- the Nikkor 10,5mm AF DX f/2.8 is R = 1.47 x f x sin (0.713 x
      -- Omega). The measured focal length is f = 10.58mm. This may be
      -- the farthest from the theoretical formula amongst all
      -- commercially available circular fisheye lenses that have been
      -- tested so far. It is therefore nearly abusive to put it in the
      -- so-called Equi-Solid Angle projection class.
      
      focal    = 10.58, -- mm

      lens_type = "thoby"
   }
}

-- combine image and lens data into a single object

function LensSensor:__init(lens_sensor,img)

   if not lens_sensor then
      error("Pass lens + sensor type as first arg")
   end
   if type(lens_sensor) == "string" then
      lens_sensor = self.default[lens_sensor]
   end
   if (not lens_sensor) or (type(lens_sensor) ~= "table") then
      error("LensSensorType not recognized")
   end

   -- copy static data from lens_sensor
   for key, val in pairs(lens_sensor) do
      self[key] = val
   end

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
   local horizontal_normalized = cx / hfocal_px 
   local vertical_normalized   = cy / vfocal_px

   -- maximum distance in normalized image coordinates which we want
   -- to project from self (default is the diagonal).  Note this is
   -- on the image plane therefore the normal euclidean pythagorean
   -- theorem works.
   local diagonal_normalized = 
      math.sqrt(horizontal_normalized*horizontal_normalized + 
                vertical_normalized*vertical_normalized)

   -- Currently unused, crop_radius would allow us to look at a region
   -- inside image boundaries such as when the image circle is less
   -- than the sensor width or height.
   if self.crop_radius then
      -- I don't understand why we divide by the vertical fov (in most
      -- cases we have square pixels, aspect ratio is 1).
      diagonal_normalized   = self.crop_radius / vfocal_px
   end

   printf(" -- normalized : diag: %f h: %f v: %f",
          diagonal_normalized, 
          horizontal_normalized, vertical_normalized)

   local dfov = projection.compute_diagonal_fov(diagonal_normalized,self.lens_type)

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

   self.diagonal_normalized   = diagonal_normalized
   self.horizontal_normalized = horizontal_normalized
   self.vertical_normalized   = vertical_normalized


   return self

end

-- Maps a camera image + lens to a unit sphere (azimuth and elevation)
-- or through sphere to other projections.
--
-- Lens types:
--
--  + rectilinear (perspective) : r = f * tan(theta)
--  + stereographic             : r = 2 * f * tan(theta/2)
--  + orthographic              : r = f * sin(theta)
--  + equal_area (equisolid)    : r = 2 * f * sin(theta/2)
--  + thoby (fisheye)           : r = k1 * f * sin(k2*theta)
--  + equal_angle (eqidistant)  : r = f * theta  (for unit sphere f = 1)
-- 
-- Proj types: 
-- 
--  + sphere
--  + rectilinear
--  + cylindrical
--  + cylindrical_vert
function LensSensor:to_projection (proj_type,scale,debug)

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
   local lambda,phi,theta_map, output_map
   if (proj_type == "rectilinear") then
      -- limit the fov to roughly 120 degrees
      if fov > 1 then 
         fov = 1 
         vfov,hfov = projection.derive_hw(fov,aspect_ratio)
      end
      -- set up size of the output table
      local drange = torch.tan(fov)
      local vrange,hrange = projection.derive_hw(drange,aspect_ratio)
      -- equal steps in normalized coordinates
      lambda   = torch.linspace(-hrange,hrange,mapw)
      phi      = torch.linspace(-vrange,vrange,maph)
      output_map = projection.make_map_of_diag_dist(lambda,phi)
      theta_map = output_map:clone():atan()
   elseif (proj_type == "cylindrical") then
      -- limit the vfov to roughly 120 degrees
      if vfov > 1 then 
         vfov = 1 
         maph = mapw * (vfov / hfov)
      end
      -- set up size of the output table
      local vrange = torch.tan(vfov)
      local hrange = hfov
      -- equal steps in normalized coordinates
      lambda   = torch.linspace(-hrange,hrange,mapw)
      phi      = torch.linspace(-vrange,vrange,maph)
      output_map = projection.make_map_of_diag_dist(lambda,phi)
      lambda:atan()
      theta_map = projection.make_map_of_diag_dist(lambda,phi)
   elseif (proj_type == "cylindrical_vert") then
      -- limit the hfov to roughly 120 degrees
      if hfov > 1 then 
         hfov = 1 
         mapw = maph * (hfov/vfov)
      end
      -- set up size of the output table
      local vrange = vfov
      local hrange = torch.tan(hfov)
      -- equal steps in normalized coordinates
      lambda   = torch.linspace(-hrange,hrange,mapw)
      phi      = torch.linspace(-vrange,vrange,maph)
      output_map = projection.make_map_of_diag_dist(lambda,phi)
      phi:atan()
      theta_map = projection.make_map_of_diag_dist(lambda,phi)
   else
      lambda = torch.linspace(-hfov,hfov,mapw)
      phi    = torch.linspace(-vfov,vfov,maph)
      -- default is to project to sphere
      -- create horizontal (lambda) and vertical angles (phi) x,y lookup
      --  in spherical map from -radians,radians at resolution mapw and
      --  maph.  Equal steps in angles.
      output_map = projection.make_map_of_diag_dist(lambda,phi)
      theta_map  = output_map
   end
   
   -- +++++
   -- (2) replace theta for each entry in map with r
   -- +++++
   local r_map = theta_map:clone() -- copy

   if (lens_type == "rectilinear") then
      -- rectilinear : (1/f) * r' = tan(theta)
      r_map:tan()
   elseif (lens_type == "thoby") then
      -- thoby       : (1/f) * r' = k1 * sin(k2*theta)
      r_map:mul(k2):sin():mul(k1)
   elseif (lens_type == "stereographic") then
      --  + stereographic             : r = 2 * f * tan(theta/2)
      r_map:mul(0.5):tan():mul(2)
   elseif (lens_type == "orthographic") then
      --  + orthographic              : r = f * sin(theta)
      r_map:sin()
   elseif (lens_type == "equisolid") then
      --  + equisolid                 : r = 2 * f * sin(theta/2)
      r_map:mul(0.5):sin():mul(2)
   else
      print("ERROR don't understand lens_type")
      return nil
   end

   -- +++++
   -- (3) x,y index (map unit sphere to normlized pixel coords)
   -- +++++
   -- make the ratio (new divided by old)
   r_map:cdiv(r_map,output_map)
   
   -- x and y are scaled by this ratio. 
   local xmap = lambda:repeatTensor(maph,1)
   xmap:cmul(r_map):mul(self.hfocal_px):add(self.center_x)
   
   local ymap = phi:repeatTensor(mapw,1):t():contiguous() 
   ymap:cmul(r_map):mul(self.vfocal_px):add(self.center_y)
   
   -- +++++++
   -- (4) make mask for out of bounds values
   -- +++++++
 
   local mask = projection.make_mask(xmap,ymap,imgw,imgh)
   printf("Masking %d/%d lookups (out of bounds)", mask:sum(),mask:nElement())

   -- +++++++
   -- (5) convert the x and y index into a single 1D offset (y * stride + x)
   -- +++++++
   
   local index_map = projection.make_index(xmap,ymap,mask)

   -- +++++++
   -- (6) output sphere object
   -- +++++++

   sphere_map = {
      lookup_table     = index_map, -- 1D LongTensor for fast lookups
      mask             = mask,      -- ByteTensor invalid locations marked with 1
      radial_distance  = theta_map, -- map of distances from optical center (theta)
      height           = maph,      -- height of the output map
      width            = mapw,      -- width of the output map
      dfov             = fov,       -- diagonal field of view
      hfov             = hfov,      -- horizontal field of view
      vfov             = vfov       -- vertical field of view
   }

   return sphere_map

end


return LensSensor
