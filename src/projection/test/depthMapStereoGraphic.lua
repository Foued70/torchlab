-- Class()
local path = require 'path'

log.tic()

d2r = math.pi / 180
pi = math.pi
pi2 = pi/2

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Compute image projections')
cmd:text()
cmd:text('Options')
cmd:option('-scandir', 'images/', 'directory with the images to load')
cmd:option('-depthdir', 'images/', 'directory with the depthmaps to load')
cmd:option('-size', '512', 'size in pixels of side of stereographic')
cmd:text()

-- parse input params
params = cmd:parse(process.argv)

scandir  = params.scandir
out_size = params.size

-- load images
if not util.fs.is_dir(scandir) then
   error("Must set a valid path to directory of images to process default -scandir images/")
end
images = util.util.file_match(scandir,"jpg$",images)
images = util.util.file_match(scandir,"png$",images)
images = util.util.file_match(scandir,"JPG$",images)
images = util.util.file_match(scandir,"PNG$",images)

pose_file = util.util.file_match(scandir,"texture_info.txt$")

poses = util.mp.load_poses(pose_file[1])

img = image.load(images[1])
depth_file = util.util.file_match(params.depthdir,path.basename(images[1]))
depth_img = image.load(depth_file[1])

width      = img:size(3)
height     = img:size(2)

hfov = poses[1].degrees_per_px_x * width * d2r
vfov = poses[1].degrees_per_px_y * height * d2r
cx   = width  * poses[1].center_u
cy   = height * poses[1].center_v

out_fov  = pi/1.05
out_c    = out_size * 0.5

proj_sphere = 
   projection.SphericalProjection.new(width,height, hfov,vfov,cx,cy)
-- forward
proj_stereo = 
   projection.StereographicProjection.new(out_size,out_size,out_fov,out_fov,nil,nil,0,0)
-- little world
proj_little = 
   projection.StereographicProjection.new(out_size,out_size,out_fov,out_fov,nil,nil,0,pi2)

time_prep = log.toc()
printf(" - prep in %2.4fs", time_prep)
log.tic()

p("Testing Stereographic LittleWorld Projection")

log.tic()

sphere_to_stereographic = projection.Remap.new(proj_sphere,proj_stereo)
sphere_to_little = projection.Remap.new(proj_sphere,proj_little)
-- do not need to call get_index_and_mask explicitly as it will be
-- called when needed on the first call to remap, but by calling it
-- here we can compute the timing information.
index1Ds = sphere_to_stereographic:get_index_and_mask()
index1Dl = sphere_to_little:get_index_and_mask()
perElement = index1Ds:nElement()

time_map = log.toc()
printf(" - make map %2.4fs", time_map)
log.tic()

img_stereo = sphere_to_stereographic:remap(img)
img_little = sphere_to_little:remap(img)
depth_stereo = sphere_to_stereographic:remap(depth_img)
depth_little = sphere_to_little:remap(depth_img)
time_reproject = log.toc()
printf(" - reproject %2.4fs", time_reproject)
log.tic()

for c = 1,3 do 
   img_stereo[c]:cmul(depth_stereo)
   img_little[c]:cmul(depth_little)
end
image.display{image={img_stereo,img_little}}

-- image.display{image={depth_stereo,depth_little}}

for i = 2,#images do 
   img = image.load(images[i])
   depth_file = util.util.file_match(params.depthdir,path.basename(images[i]))
   depth_img = image.load(depth_file[1])

   img_stereo = sphere_to_stereographic:remap(img)
   img_little = sphere_to_little:remap(img)
   depth_stereo = sphere_to_stereographic:remap(depth_img)
   depth_little = sphere_to_little:remap(depth_img)

   time_reproject = log.toc()
   printf(" - reproject %2.4fs", time_reproject)
   log.tic()
   for c = 1,3 do 
      img_stereo[c]:cmul(depth_stereo)
      img_little[c]:cmul(depth_little)
   end
   image.display{image={img_stereo,img_little}}
end


printf(" - Total %2.4fs", time_prep + time_map + time_reproject)
