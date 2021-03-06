Class()

lens_data = require '../lens_sensor_types'

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Compare calibrated to rectilinear projections')
cmd:text()
cmd:text('Options')
cmd:option('-imagesdir', 'images/', 'directory with the images to load')
cmd:option('-scale', '0.25', 'downsample')
cmd:text()

-- parse input params
params = cmd:parse(process.argv)

imagesdir  = params.imagesdir

-- load images
images = util.fs.glob(imagesdir,"jpg")
images = util.fs.glob(imagesdir,"png",images)
images = util.fs.glob(imagesdir,"JPG",images)
images = util.fs.glob(imagesdir,"PNG",images)

img = image.load(images[1])

width = img:size(3)
height = img:size(2)
 
scale  = tonumber(params.scale) or 0.25

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
proj_rect     = 
   projection.RectilinearProjection.new(width,height,
                                        proj_cal.hfov,proj_cal.vfov)

proj_sphere   = 
   projection.SphericalProjection.new(scale*width,scale*height,
                                      proj_cal.hfov,proj_cal.vfov,
                                      cx,cy)


p("Testing Calibrated Image Projection")

log.tic()

cal_to_sphere         = projection.Remap.new(proj_cal,proj_sphere)
rect_to_sphere        = projection.Remap.new(proj_rect,proj_sphere)
-- do not need to call get_index_and_mask explicitly as it will be
-- called when needed on the first call to remap, but by calling it
-- here we can compute the timing information.
index1D_sphere = cal_to_sphere:get_offset_and_mask()
index1D_rect   = rect_to_sphere:get_offset_and_mask()

perElement = index1D_sphere:nElement()

time = log.toc()
printf(" - make map %2.4fs %2.4es per px", time, time*perElement)
log.tic()

for i = 1,#images do 
   img = image.load(images[i])
   cal_img_out = cal_to_sphere:remap(img)
   rect_img_out = rect_to_sphere:remap(img)
   time = log.toc()
   printf(" - reproject %2.4fs %2.4es per px", time, time*perElement)
   log.tic()

   image.display{image={cal_img_out,rect_img_out}}
end
