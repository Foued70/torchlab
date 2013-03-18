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

-- takes a sphere map and a projection type creates a new map
function sphere_to_projection(sphere_map, projection)
   local thetas = sphere_map.radial_distance:clone()
   if (projection == "rectangular") then
      -- R = f * tan(theta)
      thetas:tan()
   end
end



img = images[1]

mylens  = LensSensor.new("nikon_D5100_w10p5mm",img)

sys.tic()
map = mylens:to_sphere()
printf(" + build look up table: %2.4fs",sys.toc())

sys.tic()
collectgarbage()
printf(" + (collect garbage) : %2.4fs",sys.toc())

sys.tic()
output_image = projection.remap(img,map)
printf(" + reproject: %2.4fs",sys.toc())

sys.tic()
collectgarbage()
printf(" + (collect garbage) : %2.4fs",sys.toc())

image.display{image={image.scale(output_image, output_image:size(3)*0.2, output_image:size(2)*0.2)}}
--image.display{image=img}


