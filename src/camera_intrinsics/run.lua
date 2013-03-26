require 'image'

local util = require "util" -- need printf
local LensSensor = require "util.LensSensor"
local projection = require "util.projection"

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
cachedir = "cache/"
sys.execute("mkdir -p " .. cachedir)

-- load images
if not images then
   images = {}
   if not paths.dirp(imagesdir) then 
      error("Must set a valid path to directory of images to process default -imagesdir images/")
   end
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
         local img = image.load(imgfile)
         -- rough start of memory mapping image files
         -- imgfile = cachedir .. imgfile:gsub("/","_")
         -- torch.save(imgfile,img:storage())
         -- images[cnt] = torch.Tensor(torch.DoubleStorage(imgfile))
         -- images[cnt]:resize(img:size(1),img:size(2),img:size(3))
         images[cnt] = img
         cnt = cnt + 1
         
      end
   end
end
collectgarbage()

img = images[1]


lenses  = { LensSensor.new("nikon_10p5mm_calibrated",img),
             LensSensor.new("nikon_D5100_w10p5mm",img) 
}
for i = 1,#lenses do
   mylens = lenses[i]

   for i = 1,#images do
      img = images[i]
      if (i == 1) or imgsize ~= img:size() then
         -- once computed, we only need to recompute projection map if
         -- the image size changes.
         mylens:add_image(img) -- only need to do this if the image dimensions change
         sys.tic()
         rmap = mylens:make_projection_map("rectilinear")
         printf(" + (rectilinear) build look up table: %2.4fs",sys.toc())
         
         sys.tic()
         smap = mylens:make_projection_map()
         printf(" + (spherical) build look up table: %2.4fs",sys.toc())
      
         sys.tic()
         collectgarbage()
         printf(" + (collect garbage) : %2.4fs",sys.toc())
         imgsize = img:size()
      end

      sys.tic()
      routput_image = projection.remap(img,rmap)
      printf(" + (rectilinear) reproject: %2.4fs",sys.toc())

      sys.tic()
      soutput_image = projection.remap(img,smap)
      printf(" + (spherical ) reproject: %2.4fs",sys.toc())

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
   end

end