
local pi = math.pi
local piover2 = math.pi * 0.5
local r2d = 180 / pi
local d2r = pi / 180

-- thoby constants
local k1 = 1.47
local k2 = 0.713

-- treat angles with spherical geometry 
local noneuclidean = false

-- stores fixed information about lens+sensors which we glean from spec sheets and elsewhere.
local LensSensor = torch.class("LensSensor")

LensSensor.default = {
   nikon_D800E_w18mm = {

      name     = "Nikon D800E with 18mm",
      
      -- copied from exif info
      sensor_w = 35.9, -- mm
      sensor_h = 24.0, -- mm
      focal    = 18, -- mm

      type = "rectilinear",
      
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

      type = "thoby"
   }
}

-- combine image and lens data into a single object

function LensSensor:__init(lenstype,img)

   if not lenstype then
      error("Pass lens + sensor type as first arg")
   end
   if type(lenstype) == "string" then
      lenstype = self.default[lenstype]
   end
   if (not lenstype) or (type(lenstype) ~= "table") then
      error("LensType not recognized")
   end

   -- copy static data from lenstype
   for key, val in pairs(lenstype) do
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

   local dfov = 0

   -- We don't have most of these types of lenses but it is easy
   -- enough to put here.  Perhaps we will include a universal model
   -- as per Scaramuzza's calibration.

   if (self.type == "rectilinear") then
      dfov = torch.atan(diagonal_normalized)   -- diag in rad

   elseif (self.type == "thoby") then
      print(" -- using lens type: thoby")
      -- thoby : theta = asin((r/f)/(k1 * f))/k2
      if (diagonal_normalized > k1) then 
         error("diagonal too large for thoby")
      else
         dfov = torch.asin(diagonal_normalized/k1)/k2
      end
   elseif (self.type == "equal_angle") then
      dfov = diagonal_normalized

   elseif (self.type =="equal_area") then
      if( diagonal_normalized <= 2 ) then
         dfov = 2 * torch.asin( 0.5 * diagonal_normalized )
      end
      if( dfov == 0 ) then
         error( "equal-area FOV too large" )
      end

   elseif (self.type == "stereographic") then
      dfov = 2 * atan( 0.5 * diagonal_normalized );

   elseif (self.type == "orthographic") then
      if( diagonal_normalized <= 1 ) then
         dfov = torch.asin( diagonal_normalized )
      end
      if( dfov == 0 ) then
         error( "orthographic FOV too large" );
      end;

   else
      error("don't understand self lens model requested")
   end

   -- horizontal and diagonal fov's are useful for size of our lookup
   -- table.  Using normal (euclidean) pythagorean theorem to compute
   -- w and h from the aspect ratio and the diagonal which doesn't
   -- feel right but gives the expected result as opposed to the
   -- non-euclidean cos(c) = cos(a)cos(b)
   local vfov = dfov/math.sqrt(aspect_ratio*aspect_ratio + 1)
   local hfov = aspect_ratio*vfov

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

-- Maps a camera image + lens to a unit sphere (azimuth and elevation).
--
-- Lens types:
--
--  + rectilinear (perspective) : r = f * tan(theta)
--  + stereographic             : r = 2 * f * tan(theta/2)
--  + orthographic              : r = f * sin(theta)
--  + equal_area (equisolid)    : r = 2 * f * sin(theta/2)
--  + thoby (fisheye)           : r = k1 * f * sin(k2*theta)
--  + equal_angle (eqidistant)  : r = f * theta  (for unit sphere f = 1)

function LensSensor:to_sphere (scale,debug)

   if not scale then
      scale = 1
   end

   local projection = self.type

   -- +++++
   -- (0).a image dimensions
   -- +++++
   local imgw = self.image_w
   local imgh = self.image_h
   local hlfw = self.center_x
   local hlfh = self.center_y

   local proj = self.projection
   local fov  = self.fov
   local hfov = self.hfov
   local vfov = self.vfov

   -- +++++
   -- (0).b find dimension of the map
   -- +++++
   local mapw = imgw * scale
   local maph = mapw * (vfov/hfov)

   local max_imgw = hlfw
   local max_imgh = hlfh

   -- +++++
   -- (1) create map of diagonal angles from optical center
   -- +++++

   -- create horizontal (lambda) and vertical angles (phi)
   -- (equirectangular) x,y lookup in spherical map from
   -- -radians,radians at resolution mapw and maph
   local lambda = torch.linspace(-hfov,hfov,mapw)
   local phi    = torch.linspace(-vfov,vfov,maph)

   if noneuclidean then
      -- non-euclidean pythagorean
      local cosx = lambda:clone():cos():resize(1,mapw):expand(maph,mapw)
      local cosy = phi:clone():cos():resize(maph,1):expand(maph,mapw)
      -- pythagorean theorem on a unit sphere is cos(c) = cos(a)cos(b)
      -- c = arccos(cos(a)cos(b))
      theta_map = torch.cmul(cosx,cosy):acos()
      cosx = nil
      cosy = nil
      collectgarbage() -- not such a big deal as we are using expand
   else
      -- normal pythagorean theorem
      local xsqr = lambda:clone():cmul(lambda):resize(1,mapw):expand(maph,mapw) 
      local ysqr = phi:clone():cmul(phi):resize(maph,1):expand(maph,mapw)
      theta_map = torch.add(xsqr,ysqr)
      theta_map:sqrt()
      xsqr = nil
      ysqr = nil
      collectgarbage() -- not such a big deal as we are using expand
   end

   -- +++++
   -- (2) replace theta for each entry in map with r
   -- +++++
   local r_map = theta_map:clone() -- copy

   if (projection == "rectilinear") then
      -- rectilinear : (1/f) * r' = tan(theta)
      r_map:tan()
   elseif (projection == "thoby") then
      -- thoby       : (1/f) * r' = k1 * sin(k2*theta)
      r_map:mul(k2):sin():mul(k1)
   elseif (projection == "stereographic") then
      --  + stereographic             : r = 2 * f * tan(theta/2)
      r_map:mul(0.5):tan():mul(2)
   elseif (projection == "orthographic") then
      --  + orthographic              : r = f * sin(theta)
      r_map:sin()
   elseif (projection == "equisolid") then
      --  + equisolid                 : r = 2 * f * sin(theta/2)
      r_map:mul(0.5):sin():mul(2)
   else
      print("ERROR don't understand projection")
      return nil
   end


   -- +++++
   -- (3) x,y index (map unit sphere to normlized pixel coords)
   -- +++++

   -- try simplistic map from Hugin lens correction which seems
   -- completely wrong to me.
   -- make the ratio (new divided by old)
   r_map:cdiv(r_map,theta_map)
   
   local xmap = lambda:repeatTensor(maph,1)
   xmap:cmul(r_map):mul(self.hfocal_px):add(self.center_x)
   
   local ymap = phi:repeatTensor(mapw,1):t():contiguous() 
   ymap:cmul(r_map):mul(self.vfocal_px):add(self.center_y)
   
   -- know r, the diagonal, in normalized
   -- coordinates.  x and y are scaled by this ratio and the ratio x
   -- to y so use euclidean pythagorean theorem to get x and y in
   -- normalized image coordinates for each location.

   -- +++++++
   -- (4) make mask for out of bounds values
   -- +++++++
   local mask = xmap:ge(1) + xmap:lt(imgw)  -- out of bound in xmap
   mask = mask + ymap:ge(1) + ymap:lt(imgh) -- out of bound in ymap
   -- reset mask to 0 and 1 (valid parts of mask must pass all 4
   -- tests).  We need the mask to reset bad pixels so the 1s are the
   -- out of bound pixels we want to replace
   mask[mask:ne(4)] = 1  -- out of bounds
   mask[mask:eq(4)] = 0  -- in bounds
   printf("Masking %d/%d lookups (out of bounds)", mask:sum(),mask:nElement())

   -- +++++++
   -- (5) convert the x and y index into a single 1D offset (y * stride + x)
   -- +++++++

   -- CAREFUL must floor before multiplying by stride or does not make sense.
   -- -0.5 then floor is equivalient to adding 0.5 -> floor -> -1 before multiply by stride.
   local outmap = ymap:clone():add(-0.5):floor()

   -- ymap -1 so that multiply by stride makes sense (imgw is the stride)
   -- to map (1,1) ::  y = 0  * stride + x = 1 ==> 1
   -- to map (2,1) ::  y = 1  * stride + x = 1 ==> stride + 1 etc.

   outmap:mul(imgw):add(xmap + 0.5)
   -- remove spurious out of bounds from output
   outmap[mask] = 1
   local index_map = outmap:long() -- round (+0.5 above) and floor
   -- make 1D
   index_map:resize(index_map:nElement())

   -- only needed if we reuse the xmap and ymap
   xmap = xmap:floor()
   xmap[mask] = 1
   ymap = ymap:floor()
   ymap[mask] = 1


   if debug then
      printf(" x map from %d to %d (max: %d)",xmap:min(),xmap:max(),imgw)
      printf(" y map from %d to %d (max: %d)",ymap:min(),ymap:max(),imgh)
      printf("1D map from %d to %d (max: %d)",index_map:min(),index_map:max(),index_map:size(1))
   end

   -- +++++++
   -- (6) output sphere object
   -- +++++++

   sphere_map = {
      lookup_table     = index_map, -- 1D LongTensor for fast lookups
      lookup_x         = xmap,      -- 2D lookup for x
      lookup_y         = ymap,      -- 2D lookup for y
      mask             = mask,      -- ByteTensor invalid locations marked with 1
      radial_distance  = theta_map, -- map of distances from optical center (theta)
      fov_width        = fovw,      -- field of view in radians (width)
      fov_height       = fovh,      -- field of view in radians (height)
      width            = mapw,      -- dimensions to display map in 2D. width
      height           = maph       -- and height.
   }

   return sphere_map

end

return LensSensor
