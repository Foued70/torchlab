local projection_util = util.projection
require 'image'

local pi = math.pi
local pi2 = pi * 0.5

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Compute image projections')
cmd:text()
cmd:text('Options')
cmd:option('-imagesdir', 'images/', 'directory with the images to load')
cmd:text()

-- parse input params
params = cmd:parse(arg)

imagesdir  = params.imagesdir

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

img = image.load(images[1])

local width = img:size(3)
local height = img:size(2)

-- images are vertical
local vfov = (97/180) * pi 
local hfov = (74/180) * pi
 
proj_from = projection.RectilinearProjection.new(width,height,hfov,vfov)
proj_to   = projection.SphericalProjection.new(width,height,hfov,vfov)

local scale  = 1/5

p("Testing Image Projection")

sys.tic()
angle_map      = proj_to:angles_map(scale)
index_and_mask = proj_from:angles_to_index1D_and_mask(angle_map)

local perElement = index_and_mask.index1D:nElement()

time = sys.toc()
printf(" - make map %2.4fs %2.4es per px", time, time*perElement)
sys.tic()

for i = 1,#images do 
   img = image.load(images[i])
   img_out = projection_util.remap(img,index_and_mask)
   time = sys.toc()
   printf(" - reproject %2.4fs %2.4es per px", time, time*perElement)
   sys.tic()

   img_scale = img_out:clone()
   image.scale(img,img_scale)
   image.display{image={img_scale,img_out}}
end
