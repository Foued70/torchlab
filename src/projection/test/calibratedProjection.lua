local projection_util = util.projection
require 'image'

lens_data = require 'util.lens_sensor_types'

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
 
lens = lens_data.sigma_10_20mm 

if width < height then
   fx = lens.fy
   fy = lens.fx
   cx = lens.cy
   cy = lens.cx
else
   fx = lens.fx
   fy = lens.fy
   cx = lens.cx
   cy = lens.cy
end

proj_from  = projection.CalibratedProjection.new(width,height,
                                                 fx, fy,
                                                 cx, cy,
                                                 lens.radial_coeff,
                                                 lens.tangential_coeff
                                                )

proj_to_sphere   = projection.SphericalProjection.new(width,height,proj_from.hfov,proj_from.vfov)
proj_to_rect     = projection.RectilinearProjection.new(width,height,proj_from.hfov,proj_from.vfov)

local scale  = 1/5

p("Testing Calibrated Image Projection")

sys.tic()
sphere_map     = proj_to_sphere:angles_map(scale)
rect_map       = proj_to_rect:angles_map(scale)
sphere_index_and_mask = proj_from:angles_to_index1D_and_mask(sphere_map)
rect_index_and_mask   = proj_from:angles_to_index1D_and_mask(rect_map)
local perElement = sphere_index_and_mask.index1D:nElement()

time = sys.toc()
printf(" - make map %2.4fs %2.4es per px", time, time*perElement)
sys.tic()

for i = 1,#images do 
   img = image.load(images[i])
   simg_out = projection_util.remap(img,sphere_index_and_mask)
   rimg_out = projection_util.remap(img,rect_index_and_mask)
   time = sys.toc()
   printf(" - reproject %2.4fs %2.4es per px", time, time*perElement)
   sys.tic()

   img_scale = simg_out:clone()
   image.scale(img,img_scale)
   image.display{image={img_scale,rimg_out,simg_out}}
end
