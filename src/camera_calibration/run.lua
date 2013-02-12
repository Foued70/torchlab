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
   -- printf("x': %f y': %f x'': %f y'': %f",x,y,x*scale,y*scale)

   -- rescale -1,1 to px coords in image space
   return x*scale,y*scale
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
   local out_h = h*scale
   local out_w = w*scale
   local half_image_w = out_w * 0.5
   local half_image_h = out_h * 0.5

   local map = torch.Tensor(h*scale,w*scale,2)

   local half_camera_w = w * 0.5
   local half_camera_h = h * 0.5

   -- We only need to compute one quadrant and copy to the others
   -- map of even steps between -1,0 1/2 camera resolution 
   out_row = torch.linspace(0,aspect_ratio,half_camera_w)
   out_col = torch.linspace(0,1,half_camera_h) -- assumes that out_col is the short edge

   for yi = 1,half_camera_h do
      for xi = 1,half_camera_w do
         local xlin  = out_row[xi]
         local ylin  = out_col[yi]

         local xp,yp = radial_undistort(xlin,ylin,camera)

         local xsrc1 =   xlin
         local ysrc1 =   ylin
         local xsrc2 = - xlin
         local ysrc2 = - ylin

         local xdst1 =   xp
         local ydst1 =   yp
         local xdst2 = - xp
         local ydst2 = - yp

         printf("x: %f y:%f -> x':%f y':%f",xsrc1,ysrc1,xdst1,ydst1)

         -- top-right quadrant
         if (xdst1 > -aspect_ratio and xdst1 < aspect_ratio and ydst1 > -1 and ydst1 < 1) then
            map[{half_camera_h + yi, half_camera_w + xi,1}] = (xdst1 + aspect_ratio)/(2*aspect_ratio) * out_w
            map[{half_camera_h + yi, half_camera_w + xi,2}] = (ydst1 + 1)/2 * out_h
         end

         

         -- bottom-right quadrant
         if (xdst2 > -aspect_ratio and xdst2 < aspect_ratio and ydst1 > -1 and ydst1 < 1) then
            map[{yi + half_camera_h, -xi + half_camera_w, 1}] = (xdst2 + aspect_ratio)/(2*aspect_ratio) * out_w
            map[{yi + half_camera_h, -xi + half_camera_w, 2}] = (ydst1 + 1)/2 * out_h
         end

         -- bottom-left quadrant
         if (xdst2 > -aspect_ratio and xdst2 < aspect_ratio and ydst1 > -1 and ydst1 < 1) then
            map[{-yi + half_camera_h, -xi + half_camera_w, 1}] = (xdst2 + aspect_ratio)/(2*aspect_ratio) * out_w
            map[{-yi + half_camera_h, -xi + half_camera_w, 2}] = (ydst2 + 1)/2 * out_h
         end

         -- printf("x: %f y:%f -> x':%f y':%f",xsrc2,ysrc1,xdst2,ydst1)
         -- out[{{},ydst1,xdst2}] = images[1][{{},ysrc1,xsrc2}]

         -- printf("x: %f y:%f -> x':%f y':%f",xsrc1,ysrc2,xdst1,ydst2)
         -- out[{{},ydst2,xdst1}] = images[1][{{},ysrc2,xsrc1}]

         -- printf("x: %f y:%f -> x':%f y':%f",xsrc2,ysrc2,xdst2,ydst2)
         -- out[{{},ydst2,xdst2}] = images[1][{{},ysrc2,xsrc2}]
      end
   end
   return map
end

function remap(img, map)
   local out = torch.Tensor(img:size())

   for yi = 1, map:size(1) do
      for xi = 1, map:size(2) do
         local coord = map[{yi, xi, {}}]

         out[{{}, yi, xi}] = img[{{}, coord[2] + 1, coord[1] + 1}]
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
