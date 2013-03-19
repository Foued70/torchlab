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

img = images[1]

mylens  = LensSensor.new("nikon_D5100_w10p5mm",img)

sys.tic()
rmap = mylens:to_projection("rectilinear")
printf(" + build look up table: %2.4fs",sys.toc())

sys.tic()
smap = mylens:to_projection()
printf(" + build look up table: %2.4fs",sys.toc())

sys.tic()
collectgarbage()
printf(" + (collect garbage) : %2.4fs",sys.toc())

sys.tic()
routput_image = projection.remap(img,rmap)
printf(" + reproject: %2.4fs",sys.toc())

sys.tic()
soutput_image = projection.remap(img,smap)
printf(" + reproject: %2.4fs",sys.toc())

sys.tic()
collectgarbage()
printf(" + (collect garbage) : %2.4fs",sys.toc())

rimg_scale = image.scale(routput_image, 
                         routput_image:size(3)*0.2, 
                         routput_image:size(2)*0.2)

simg_scale = image.scale(soutput_image, 
                         soutput_image:size(3)*0.2, 
                         soutput_image:size(2)*0.2)

image.display{image={rimg_scale,simg_scale}}


