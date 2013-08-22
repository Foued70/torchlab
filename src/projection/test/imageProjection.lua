pi = math.pi
pi2 = pi * 0.5

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Compute image projections')
cmd:text()
cmd:text('Options')
cmd:option('-imagesdir', 'images/', 'directory with the images to load')
cmd:option('-scale', '0.1', 'downsample ratio')
cmd:text()

-- parse input params
params    = cmd:parse(process.argv)
imagesdir = params.imagesdir

-- load images
images = util.fs.glob(imagesdir,{"JPG","PNG","jpg","png"})

img = image.load(images[1])

width  = img:size(3)
height = img:size(2)
scale  = tonumber(params.scale) or 0.1

-- images are vertical
vfov = (97/180) * pi 
hfov = (74/180) * pi
 
-- proj_from = projection.RectilinearProjection.new(width,height,hfov,vfov)
proj_from = projection.GnomonicProjection.new(width,height,hfov,vfov)
proj_to   = projection.SphericalProjection.new(width*scale,height*scale,hfov,vfov)

p("Testing Image Projection")

log.tic()

rect_to_sphere = projection.Remap.new(proj_from,proj_to)

-- do not need to call get_index_and_mask explicitly as it will be
-- called when needed on the first call to remap, but by calling it
-- here we can compute the timing information.
offset,stride,mask = rect_to_sphere:get_offset_and_mask()
perElement = offset:nElement()
time = log.toc()
printf(" - make map %2.4fs %2.4es per px", time, time*perElement)
log.tic()

for i = 1,#images do 
   img = image.load(images[i])
   img_out = rect_to_sphere:remap(img)
   time = log.toc()
   printf(" - reproject %2.4fs %2.4es per px", time, time*perElement)
   log.tic()

   img_scale = img_out:clone()
   image.scale(img,img_scale)
   image.display{image={img_scale,img_out}}
end
