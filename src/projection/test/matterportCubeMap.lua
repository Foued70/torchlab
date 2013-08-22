Class()

log.tic()

d2r = math.pi / 180

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Compute image projections')
cmd:text()
cmd:text('Options')
cmd:option('-scandir', 'images/', 'directory with the images to load')
cmd:option('-outfile', 'skybox', 'basename for 6 output images for skybox')
cmd:option('-size', '512', 'size in pixels of side of skybox cube')
cmd:text()

-- parse input params
params = cmd:parse(process.argv)

scandir  = params.scandir
out_size = params.size
out_file = params.outfile

-- load images
images    = util.fs.glob(scandir,"jpg")
pose_file = util.fs.glob(scandir,"_texture_info.txt")
pose_file = pose_file[1]

poses = model.mp.load_poses(pose_file)

img = image.load(images[1])

width      = img:size(3)
height     = img:size(2)

hfov = poses[1].degrees_per_px_x * width * d2r
vfov = poses[1].degrees_per_px_y * height * d2r
cx   = width  * poses[1].center_u
cy   = height * poses[1].center_v

proj_from = projection.SphericalProjection.new(width,height, hfov,vfov,cx,cy)

time_prep = log.toc()
printf(" - load image in %2.4fs", time_prep)
log.tic()

p("Creating Skybox Projection")
cmap = projection.CubeMap.new(proj_from,out_size)

time_map = log.toc()
printf(" - make map %2.4fs", time_map)
log.tic()

faces = cmap:remap(img)
for name,face in pairs(faces) do 
   outf = out_file .."_"..name..".jpg"
   image.display(face)
   printf(" - saving: %s", outf)
   image.save(outf,face)
end

time_reproject = log.toc()
printf(" - reproject %2.4fs", time_reproject)
log.tic()


printf(" - Total %2.4fs", time_prep + time_map + time_reproject)
