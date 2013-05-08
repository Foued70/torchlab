require 'image'

local LensSensor = util.LensSensor
local projection = util.projection

-- top level filenames

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Compute image projections')
cmd:text()
cmd:text('Options')
cmd:option('-imagesdir', 'images/', 'directory with the images to load')
cmd:option('-lenstype', 'sigma_10_20mm', 'lenstype as collected in lens_sensor_types eg. nikon_10p5mm_r2t_full')
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
   imgfiles() -- .
   imgfiles() -- ..
   for f in imgfiles do
      if f == ".DS_Store" then -- exclude OS X automatically-created backup files
         printf("--- Skipping .DS_Store file")
         
      elseif (f:gmatch("jpg$")() or f:gmatch("png$")()) then
         local imgfile = imagesdir.."/"..f
         table.insert(images, imgfile)
         printf("Found : %s", imgfile)
      end
   end
end
collectgarbage()

lenstype = params.lenstype
lens     = LensSensor.new(lenstype) 

for i = 1,#images do
   imgfname = images[i]
   img = image.load(imgfname)
   if ((i == 1) or (imgsize ~= img:size())) then
      -- once computed, we only need to recompute projection map if
      -- the image size changes.
      lens:add_image(img) -- only need to do this if the image dimensions change
      sys.tic()
      rmap = lens:make_projection_map("rectilinear")
      printf(" + (rectilinear) build look up table: %2.4fs",sys.toc())
      
      sys.tic()
      smap = lens:make_projection_map("equirectangular") -- plate carree
      printf(" + (equirectangular) build look up table: %2.4fs",sys.toc())
      
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
   
   display_h = img:size(2)*0.2
   display_w = img:size(3)*0.2
            
   img_scale  = image.scale(img,display_w,display_h)

   rimg_scale = image.scale(routput_image,display_w,display_h)
   
   simg_scale = image.scale(soutput_image,display_w,display_h)
   
   image.display{image={img_scale,rimg_scale,simg_scale}}
end
