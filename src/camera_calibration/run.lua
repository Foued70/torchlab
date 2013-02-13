require 'image'

local util = require 'util'
local geom = util.geom

local r2d = 180 / math.pi
local d2r = math.pi / 180

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
      local imgfile = imagesdir.."/"..f
      printf("Loading : %s", imgfile)
      images[cnt] = image.load(imgfile)
      cnt = cnt + 1
   end
end

-- start of camera object (extension of pose object...)

-- FIXME write .ini loader...


camera = {
   name     = "nikon_d800E_w18mm",
   sensor_w = 35.9, -- mm
   sensor_h = 24.0, -- mm
   focal    = 18,   -- mm
   -- copied from .ini computed in hugin
   a       =  0.0762999,
   b       = -0.167213,
   c       =  0.061329
}

-- FIXME we have a problem when loading vertical or horizontal images.

-- sensor is always horizontal (as in libpanotools)
if images[1]:size(2) > images[1]:size(3) then
   camera.image_w  = images[1]:size(2) -- px
   camera.image_h  = images[1]:size(3) -- px
else
   camera.image_h  = images[1]:size(2) -- px
   camera.image_w  = images[1]:size(3) -- px
end

-- same as in pose
camera.center_x = camera.image_w*0.5
camera.center_y = camera.image_h*0.5

camera.px_per_mm_x = camera.sensor_w / camera.image_w
camera.px_per_mm_y = camera.sensor_h / camera.image_h

camera.fovw     = r2d * 2 * torch.atan((camera.sensor_w*0.5)/camera.focal) -- deg
camera.fovh     = r2d * 2 * torch.atan((camera.sensor_h*0.5)/camera.focal) -- deg

camera.degree_per_px_x = camera.fovw/camera.image_w
camera.degree_per_px_y = camera.fovh/camera.image_h

-- focal length in pixels
camera.focal_px_x = (camera.sensor_w * 0.5)/(camera.center_x * camera.focal)
camera.focal_px_y = (camera.sensor_h * 0.5)/(camera.center_y * camera.focal)


-- Standard lens correction (pinhole + water)

-- distortion = 1 + a*r^2 + b*r^4 + c*r^6
-- xcorr = x * distortion
-- ycorr = y * distortion


-- from x,y (-1,1) in camera local to distorted raw px coordinates 
-- 
-- FIXME To keep things sane, we would rather express radial
-- distortion in angle from optical axis which would correspond to ray
-- angles in the 3D world. dimension of x and y needs to be clear.
-- 
-- r = 1 in radial undistort is at the shorter focal length.
-- 
function radial_undistort(x,y,cam)
   local r = x^2 + y^2 -- * inv_radius

   -- a*r^2 + b*r^4 + c*r^6
   local scale = 1 + ((cam.c*r + cam.b)*r + cam.a)*r

   -- rescale -1,1 to px coords in image space
   return x*scale,y*scale
   -- return x,y
end



-- attempt a distortion
-- make forward and backward lookup table


function make_map (camera,scale)

   if not scale then
      scale = 1
   end

   local w = camera.image_w
   local h = camera.image_h

   local aspect_ratio = w / h

   -- these are if we want to change the resolution of the lookup table.
   local map_h = h*scale
   local map_w = w*scale

   map = torch.Tensor(map_h,map_w,2)

   -- We only need to compute one quadrant and copy to the others
   -- map of even steps between -1,0 1/2 camera resolution 
   out_row = torch.linspace(-aspect_ratio,aspect_ratio,map_w)
   out_col = torch.linspace(-1,1,map_h) -- assumes that out_col is the short edge

   for yi = 1,map_h do
      for xi = 1,map_w do
         local xlin  = out_row[xi]
         local ylin  = out_col[yi]

         local xp,yp = radial_undistort(xlin,ylin,camera)

         map[{yi,xi,1}] = (xp + aspect_ratio)/(2*aspect_ratio) * map_w
         map[{yi,xi,2}] = (yp + 1)/2 * map_h
      end
   end

   return map
end

function remap(img, map)
   local out = torch.Tensor(img:size())

   for yi = 1, map:size(1) do
      for xi = 1, map:size(2) do
         local coord = map[{yi, xi, {}}]

         if (coord[2] > 0 and coord[2] < 683 and coord[1] > 0 and coord[1] < 1024) then
            out[{{}, yi, xi}] = img[{{}, coord[2], coord[1]}]
         end
      end
   end

   return out
end

-- image.rotate does not do what we want
function img_rot (img)
   return  image.vflip(img:transpose(2,3))
end

map = make_map(camera)
output_image = remap(images[1], map)
image.display{image={img_rot(output_image),img_rot(images[1])}}
