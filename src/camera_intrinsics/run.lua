require 'image'

local util = require "util" -- need printf
local LensSensor = require "util.LensSensor"
local projection = require "util.projection"

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
cmd:text()

-- parse input params
params = cmd:parse(arg)

imagesdir  = params.imagesdir


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

img = images[1]

mylens  = LensSensor.new("nikon_D5100_w10p5mm",img)

sys.tic()
rmap = mylens:to_projection("rectilinear")
printf(" + (rectilinear) build look up table: %2.4fs",sys.toc())

sys.tic()
cmap = mylens:to_projection("cylindrical")
printf(" + (cylindrical) build look up table: %2.4fs",sys.toc())

sys.tic()
vmap = mylens:to_projection("cylindrical_vert")
printf(" + (cylindrical vert) build look up table: %2.4fs",sys.toc())

sys.tic()
smap = mylens:to_projection()
printf(" + (spherical) build look up table: %2.4fs",sys.toc())

sys.tic()
collectgarbage()
printf(" + (collect garbage) : %2.4fs",sys.toc())

for i = 1,#images do
   img = images[i]

   sys.tic()
   routput_image = projection.remap(img,rmap)
   printf(" + (rectilinear) reproject: %2.4fs",sys.toc())
   
   sys.tic()
   coutput_image = projection.remap(img,cmap)
   printf(" + (cylinder) reproject: %2.4fs",sys.toc())
   
   sys.tic()
   voutput_image = projection.remap(img,vmap)
   printf(" + (cylinder vert) reproject: %2.4fs",sys.toc())
   
   sys.tic()
   soutput_image = projection.remap(img,smap)
   printf(" + (spherical ) reproject: %2.4fs",sys.toc())
   
   sys.tic()
   collectgarbage()
   printf(" + (collect garbage) : %2.4fs",sys.toc())
   
   rimg_scale = image.scale(routput_image, 
                            routput_image:size(3)*0.2, 
                            routput_image:size(2)*0.2)
   
   cimg_scale = image.scale(coutput_image, 
                            coutput_image:size(3)*0.2, 
                            coutput_image:size(2)*0.2)
   
   vimg_scale = image.scale(voutput_image, 
                            voutput_image:size(3)*0.2, 
                            voutput_image:size(2)*0.2)
   
   simg_scale = image.scale(soutput_image, 
                            soutput_image:size(3)*0.2, 
                            soutput_image:size(2)*0.2)
   
   image.display{image={rimg_scale,simg_scale}}
   image.display(cimg_scale)
   image.display(vimg_scale)
end

