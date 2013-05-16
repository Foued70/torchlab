Class()

sys.tic()
require 'image'
require 'nn'

saliency  = require 'saliency'
lens_data = require 'util.lens_sensor_types'

d2r = math.pi / 180

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Align images in a sweep')
cmd:text()
cmd:text('Options')
cmd:option('-scan_dir',
           "../data/test/96_spring_kitchen/",
           'base directory for scan')
cmd:option('-matter_dir',
           "raw_scan/",
           "Directory for matterport data (relative to scan_dir)")

cmd:option('-sweep_prefix',
           "sweep_",
           "Directory prefix for DSLR image sweeps (relative to scan_dir)")

cmd:option("-lens_type", "sigma_10_20mm", "data for image dewarping")

cmd:text()

-- parse input params
params = cmd:parse(arg)

scan_dir = params.scan_dir

matter_dir = scan_dir .. params.matter_dir

matter_pose_fname = util.util.file_match(matter_dir,"texture_info.txt")

if #matter_pose_fname > 0 then
   matter_pose_fname = matter_pose_fname[1]
   printf("using : %s", matter_pose_fname)
end

poses = util.mp.load_poses(matter_pose_fname)

sweep_dir  = scan_dir .. params.sweep_prefix

-- load lens calibration
lens = lens_data.sigma_10_20mm

preprocess = nn.Sequential()
preprocess:add(nn.SpatialContrastiveNormalization(1,image.gaussian(9)))

-- Could loop over sweeps here.

-- load sweeps
sweep_no = 1

-- matterport texture
matter_texture_fname = util.util.file_match(matter_dir,poses[sweep_no].name)

if #matter_texture_fname > 0 then
   matter_texture_fname = matter_texture_fname[sweep_no]
   printf("using : %s", matter_texture_fname)
end

matter_texture = image.load(matter_texture_fname)
mp_out  = preprocess:forward(image.rgb2lab(matter_texture):narrow(1,1,1))

mp_sal  = saliency.high_entropy_features(mp_out)

mp_width      = matter_texture:size(3)
mp_height     = matter_texture:size(2)

mp_hfov = poses[1].degrees_per_px_x * mp_width * d2r
mp_vfov = poses[1].degrees_per_px_y * mp_height * d2r
mp_cx   = mp_width  * poses[1].center_u
mp_cy   = mp_height * poses[1].center_v

image.display(matter_texture)
image.display(mp_sal)

-- load DSLR image

images = util.util.file_match(sweep_dir .. sweep_no, ".jpg")

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
proj_rect = projection.RectilinearProjection.new(width,height,
                                                 proj_cal.hfov,proj_cal.vfov)

-- best guess at aligning the two images.
out_height = proj_cal.vfov / (d2r * poses[1].degrees_per_px_y)
out_width  = proj_cal.hfov / (d2r * poses[1].degrees_per_px_x)

proj_sphere = projection.SphericalProjection.new(out_width, out_height,
                                                 proj_cal.hfov,proj_cal.vfov,
                                                 cx,cy)


time_prep = sys.toc()
printf(" - load image in %2.4fs", time_prep)
sys.tic()

p("Testing Calibrated Image Projection")

sys.tic()

cal_to_sphere         = projection.Remap.new(proj_cal,proj_sphere)
-- rect_to_sphere        = projection.Remap.new(proj_rect,proj_sphere)
-- do not need to call get_index_and_mask explicitly as it will be
-- called when needed on the first call to remap, but by calling it
-- here we can compute the timing information.
index1D_cal    = cal_to_sphere:get_index_and_mask()
-- index1D_rect   = rect_to_sphere:get_index_and_mask()

perElement = index1D_cal:nElement()

time = sys.toc()
printf(" - make map %2.4fs %2.4es per px", time, time*perElement)
sys.tic()



-- DSLR images

for i = 1,#images do
   img = image.load(images[i])

   cal_img_out  = cal_to_sphere:remap(img)
   -- rect_img_out = rect_to_sphere:remap(img)

   time = sys.toc()
   printf(" - reproject %2.4fs %2.4es per px", time, time*perElement)
   sys.tic()

   cal_pp = preprocess:forward(image.rgb2lab(cal_img_out):narrow(1,1,1))
   -- rect_pp = preprocess:forward(image.rgb2lab(rect_img_out):narrow(1,1,1))

   cal_sal = saliency.high_entropy_features(cal_pp)
   -- rect_sal = saliency.high_entropy_features(rect_pp)

   time = sys.toc()
   printf(" - convert to lab %2.4fs %2.4es per px", time, time*perElement)
   sys.tic()

   -- image.display(rect_img_out)
   -- image.display(rect_sal)
   image.display(cal_img_out)
   image.display(cal_sal)

   collectgarbage()
end
