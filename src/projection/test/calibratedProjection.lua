require 'image'

local lens_data = require 'util.lens_sensor_types'

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Compare calibrated to rectilinear projections')
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

width = img:size(3)
height = img:size(2)
 
lens = lens_data.sigma_10_20mm 

-- handle the vertical images correctly
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

proj_cal  = projection.CalibratedProjection.new(width,height,
                                                 fx, fy,
                                                 cx, cy,
                                                 lens.radial_coeff,
                                                 lens.tangential_coeff
                                                )

proj_sphere   = projection.SphericalProjection.new(width,height,
                                                   proj_cal.hfov,proj_cal.vfov)
proj_rect     = projection.RectilinearProjection.new(width,height,
                                                     proj_cal.hfov,proj_cal.vfov)

local scale  = 1/5

p("Testing Calibrated Image Projection")

sys.tic()

cal_to_sphere         = projection.Remap.new(proj_cal,proj_sphere)
rect_to_sphere        = projection.Remap.new(proj_rect,proj_sphere)
-- do not need to call get_index_and_mask explicitly as it will be
-- called when needed on the first call to remap, but by calling it
-- here we can compute the timing information.
index1D_sphere = cal_to_sphere:get_index_and_mask(scale)
index1D_rect   = rect_to_sphere:get_index_and_mask(scale)

perElement = index1D_sphere:nElement()

time = sys.toc()
printf(" - make map %2.4fs %2.4es per px", time, time*perElement)
sys.tic()

for i = 1,#images do 
   img = image.load(images[i])
   cal_img_out = cal_to_sphere:remap(img)
   rect_img_out = rect_to_sphere:remap(img)
   time = sys.toc()
   printf(" - reproject %2.4fs %2.4es per px", time, time*perElement)
   sys.tic()

   image.display{image={cal_img_out,rect_img_out}}
end
