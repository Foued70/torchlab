require 'image'

local util = require 'util'
local geom = util.geom

local pi = math.pi
local piover2 = math.pi * 0.5
local r2d = 180 / pi
local d2r = pi / 180


-- thoby constants
local k1 = 1.47
local k2 = 0.713

-- image.rotate does not do what we want
function img_rot (img)
   return  image.vflip(img:transpose(2,3))
end

-- top level filenames

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Compute depth maps')
cmd:text()
cmd:text('Options')
cmd:option('-imagesdir',
           'images/',
           'directory with the images to load')
cmd:option('-outdir','output/')
cmd:text()

-- parse input params
params = cmd:parse(arg)

imagesdir  = params.imagesdir
outdir     = params.outdir .. "/"


-- load images
if not images then
   images = {}
   imgfiles = paths.files(imagesdir)
   imgfiles()
   imgfiles()
   local cnt = 1
   for f in imgfiles do
      if f == ".DS_Store" then -- exclude OS X automatically-created backup files
         printf("--- Skipping .DS_Store file")
      else
         local imgfile = imagesdir.."/"..f
         printf("Loading : %s", imgfile)
         images[cnt] = image.load(imgfile)
         cnt = cnt + 1
      end
   end
end

-- start of camera object (extension of pose object...)

-- FIXME write .ini loader...

nikon_D800E_w18mm = {

   name     = "Nikon D800E with 18mm",

   -- copied from exif info
   sensor_w = 35.9, -- mm
   sensor_h = 24.0, -- mm
   focal    = 18, -- mm

   type = "rectilinear",

   -- copied from .ini computed in hugin
   a       =  0.0762999,
   b       = -0.167213,
   c       =  0.061329
}

nikon_D5100_w10p5mm = {

   name     = "Nikon D5100 with 10.5mm",

   -- copied from exif info
   sensor_w = 23.6, -- mm
   sensor_h = 15.6, -- mm
   focal    = 10.5, -- mm

   type = "thoby",

   -- computed with opencv
   a        = -0.3889126043367629,
   b        =  0.1473722957858515,
   c        = -0.02577248837247648,
   p_one    =  0.001583298478238885,
   p_two    =  0.01061379290496793
}

camera = nikon_D5100_w10p5mm


-- add image data to camera

function update_imagedata(camera,image)

   local w = image:size(3)
   local h = image:size(2)


   -- offset to optical center of image (px)
   local cx = (w + 1) * 0.5
   local cy = (h + 1) * 0.5


   -- horizontal and vertical resolution (px / mm)
   local hres = w / camera.sensor_w -- px/mm
   local vres = h / camera.sensor_h -- px/mm

   local hfocal_px = camera.focal * hres
   local vfocal_px = camera.focal * vres

   local dx = cx / hfocal_px -- (px / px ==> dimensionless)
   local dy = cy / vfocal_px

   -- maximum radius (in px) (equivalent to fov) which we want to
   -- project from camera (default is the diagonal)
   local max_trad = math.sqrt(dx*dx + dy*dy) -- diag in px / mm

   -- not sure if we need this
   if not camera.crop_radius then
      camera.crop_radius = math.sqrt(w*w + h*h)
   end

   local trad = camera.crop_radius / vfocal_px

   printf("trad: %f max_trad: %f",trad,max_trad)

   if ((trad == 0) or (trad > max_trad)) then
      trad = max_trad
   end

   local tfov = 0

   if (camera.type == "rectilinear") then
      tfov = torch.atan2(trad) -- diag in rad
   elseif (camera.type == "thoby") then
      -- thoby : theta = asin((r/f)/(k1 * f))/k2 
      tfov = torch.asin(trad/k1)/k2
   else
      print("don't understand camera lens model requested")
   end


   camera.image_w  = w -- px
   camera.image_h  = h -- px

   camera.center_x = cx -- px
   camera.center_y = cy -- px

   camera.fov = tfov

   return camera

end


-- FIXME the undistort functions need to be rewritten to generate the
-- lookup tables and

-- -- Standard lens correction (pinhole + water)

-- -- distortion = 1 + a*r^2 + b*r^4 + c*r^6
-- -- xcorr = x * distortion
-- -- ycorr = y * distortion


-- -- 
-- -- r = 1 in radial undistort is at the shorter focal length.
-- -- 
-- function radial_undistort(x,y,cam)
--    local r_squared = x^2 + y^2 -- * inv_radius

--    -- a*r^2 + b*r^4 + c*r^6
--    local scale = 1 + ((cam.c*r_squared + cam.b)*r_squared + cam.a)*r_squared

--    -- rescale -1,1 to px coords in image space
--    return x*scale,y*scale
-- end

-- function tangential_undistort(x, y, camera)
--    local r = x^2 + y^2 -- * inv_radius   -- note: r is radius squared

--    -- x_corrected = x + [2 * p1 * x * y + p2(r + 2 * x^2)]
--    -- y_corrected = y + [p1(r + 2 * y^2) + 2 * p2 * x * y]

--    local x_corrected = (camera.p_two * (r + 2*x^2) + 2 * camera.p_one * x * y)
--    local y_corrected = (camera.p_one * (r + 2*y^2) + 2 * camera.p_two * x * y)

--    return x_corrected, y_corrected
end

-- Maps a camera image to a unit cartesian sphere.
--
-- Lens types:
-- 
--  + rectilinear (perspective) : r = f * tan(theta)
--  + stereographic             : r = 2 * f * tan(theta/2)
--  + orthographic              : r = f * sin(theta)
--  + equal_area (equisolid)    : r = 2 * f * sin(theta/2)
--  + thoby (fisheye)           : r = k1 * f * sin(k2*theta)
--  + equal_angle (eqidistant)  : r = f * theta  (for unit sphere f = 1)

function camera_to_sphere (camera,scale,debug)

   if not scale then
      scale = 1
   end

   -- +++++
   -- (0).a image dimensions
   -- +++++

   local imgw = camera.image_w
   local imgh = camera.image_h
   local hlfw = camera.center_x
   local hlfh = camera.center_y

   local proj = camera.projection
   local fov  = camera.fov

   -- +++++
   -- (0).b find dimension of the map
   -- +++++

   local mapw = imgw * scale

   if (projection == "rectilinear") then
      -- rectilinear : theta = r' = tan(theta)
      max_diag = camera.focal_px * torch.tan(fov)
   elseif (projection == "thoby") then
      -- thoby       : (1/f) * r' = k1 * sin(k2*theta)
      max_diag = k1 * camera.focal_px * torch.sin(k2*fov)
   elseif (projection == "stereographic") then
      --  + stereographic             : r = 2 * f * tan(theta/2)
      max_diag = 2 * camera.focal_px * torch.tan(0.5*fov)
   elseif (projection == "orthographic") then
      --  + orthographic              : r = f * sin(theta)
      max_diag = torch.sin(fov)
   elseif (projection == "equisolid") then
      --  + equisolid                 : r = 2 * f * sin(theta/2)
      max_diag = 2 * camera.focal_px * torch.sin(0.5*fov)
   else
      print("ERROR don't understand projection")
      return nil
   end

   local maph = imgh * scale
   local max_imgw = hlfw
   local max_imgh = hlfh

   -- +++++
   -- (1) create map of diagonal angles from optical center
   -- +++++

   -- create horizontal and vertical angles (equirectangular)
   local xrow = torch.linspace(-map_fovw,map_fovw,mapw)
   local ycol = torch.linspace(-map_fovh,map_fovh,maph)

   -- pythagorean theorem on a unit sphere is cos(c) = cos(a)cos(b)
   local r_map = xrow:repeatTensor(maph,1):cos() -- cos(a)
   r_map:cmul(ycol:repeatTensor(mapw,1):t():cos()) -- * cos(b)
   r_map:acos() -- c = arccos(cos(a)cos(b)

   -- +++++
   -- (2) find the ratio r' / r for each entry in map
   -- +++++

   local rprime_map = r_map:clone() -- copy

   if (projection == "rectilinear") then
      -- rectilinear : (1/f) * r' = tan(theta)
      rprime_map:tan()
   elseif (projection == "thoby") then
      -- thoby       : (1/f) * r' = k1 * sin(k2*theta)
      rprime_map:mul(k2):sin():mul(k1)
   elseif (projection == "stereographic") then
      --  + stereographic             : r = 2 * f * tan(theta/2)
      rprime_map:mul(0.5):tan():mul(2)
   elseif (projection == "orthographic") then
      --  + orthographic              : r = f * sin(theta)
      rprime_map:sin()
   elseif (projection == "equisolid") then
      --  + equisolid                 : r = 2 * f * sin(theta/2)
      rprime_map:mul(0.5):sin():mul(2)
   else
      print("ERROR don't understand projection")
      return nil
   end

   -- ratio (dimensions cancel so we can just use unit sphere coordinates)
   local ratio_map = rprime_map:clone()
   ratio_map:cdiv(r_map)           -- ratio = r' / r

   -- +++++
   -- (3) x,y index (map unit sphere to pixel coords)
   -- +++++

   -- **** check this **
   -- we assume that the ratio of r' to r corresponds to x' to x and y' to y

   -- torch indexes from 1,size_x
   local xval = hlfw - 0.5
   local yval = hlfh - 0.5

   -- start with old x and y values centered at zero
   local xindex = torch.linspace(-max_imgw,max_imgw,mapw)
   local yindex = torch.linspace(-max_imgh,max_imgh,maph)

   local xmap = xindex:repeatTensor(maph,1)
   xmap:cmul(ratio_map)

   -- NEW we need to use pythagorean theorem to find y
   --   (cf. white board proof w/ MLC [MS])
   -- y = math.sqrt(r^2 - x^2)

   -- reuse rprime_map as the r' values are there.
   ymap = rprime_map:mul(camera.focal_px)
   ymap:mul(camera.focal_px):pow(2):add(-1,torch.pow(xmap,2)):sqrt()

   -- recenter the maps to image coordinates
   ymap:add(hlfh + 0.5)
   xmap:add(hlfw + 0.5)

   -- +++++++
   -- (4) make mask for out of bounds values
   -- +++++++
   local mask = xmap:gt(1) + xmap:lt(imgw+1)  -- out of bound in xmap
   mask = mask + ymap:gt(1) + ymap:lt(imgh+1) -- out of bound in ymap
   -- reset mask to 0 and 1 (valid parts of mask must pass all 4
   -- tests).  We need the mask to reset bad pixels so the 1s are the
   -- out of bound pixels we want to replace
   mask[mask:ne(4)] = 1
   mask[mask:eq(4)] = 0
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
      mask             = mask,      -- ByteTensor invalid locations marked with 1
      radial_distance  = r_map,     -- map of distances from optical center (theta)
      fov_width        = map_fovw,  -- field of view in radians (width)
      fov_height       = map_fovh,  -- field of view in radians (height)
      width            = mapw,      -- dimensions to display map in 2D. width
      height           = maph       -- and height.
   }

   return sphere_map

end

-- takes a sphere map and a projection type creates a new map
function sphere_to_projection(sphere_map, projection)
   local thetas = sphere_map.radial_distance:clone()
   if (projection == "rectangular") then
      -- R = f * tan(theta)
      thetas:tan()
   end
end

-- image is 3 x map dims
-- now map is 1D (this is faster)
function remap(img, map)
   local out    = torch.Tensor()
   local lookup = map.lookup_table
   local mask   = map.mask
   local nelem  = lookup:nElement()
   -- fast 1D remap
   if (lookup:nDimension() ~= 1) then
      print("ERROR map should be 1D Tensor")
      return nil
   end

   local ndim     = img:nDimension()

   if (ndim == 2) then
      -- single channel 2D

      local imgh = img:size(1)
      local imgw = img:size(2)

      img:resize(imgh * imgw)
      out = img[lookup] -- uses new indexing feature
      out[mask] = 0  -- erase out of bounds

      img:resize(imgh,imgw)
      out:resize(map.height,map.width)

   elseif (ndim == 3) then
      -- n channel (RGB) (n x h x w)
      printf("output image size: %d x %d", img:size(1), nelem)
      out:resize(img:size(1),nelem)

      for d = 1,img:size(1) do -- loop through channels
         local imgd = img[d]
         imgd:resize(imgd:size(1)*imgd:size(2))

         out[d] = imgd[lookup]
         out[d][mask] = 0

      end
      -- make output 3 x H x W
      out:resize(img:size(1),map.height,map.width)
   end
   return out
end


img = images[1]
update_imagedata(camera,img)

sys.tic()
map = projection_to_sphere(camera,"thoby")
printf(" + build look up table: %2.4fs",sys.toc())

sys.tic()
collectgarbage()
printf(" + (collect garbage) : %2.4fs",sys.toc())

sys.tic()
output_image = remap(img,map)
printf(" + reproject: %2.4fs",sys.toc())

sys.tic()
collectgarbage()
printf(" + (collect garbage) : %2.4fs",sys.toc())

image.display{image={image.scale(output_image, output_image:size(2)*0.5, output_image:size(3)*0.5)}}
image.display{image=img}
