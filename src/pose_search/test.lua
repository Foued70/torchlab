Class()

require 'image'

pi = math.pi
pi2 = pi * 0.5

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
         imgfile = imagesdir.."/"..f
         table.insert(images, imgfile)
         printf("Found : %s", imgfile)
      end
   end
end
collectgarbage()

img = image.load(images[1])

width  = img:size(3)
height = img:size(2)
scale  = 1/5

-- images are vertical
vfov = (97/180) * pi 
hfov = (74.22/180) * pi

-- output full equirectangular
out_width  = 600
out_height = 300
out_hfov   = 2 * pi
out_vfov   = pi

proj_from = projection.GnomonicProjection.new(width,height,hfov,vfov)
proj_to   = projection.SphericalProjection.new(out_width,out_height,out_hfov,out_vfov)

p("Testing Image Projection")

sys.tic()

rect_to_sphere = projection.Remap.new(proj_from,proj_to)
-- do not need to call get_index_and_mask explicitly as it will be
-- called when needed on the first call to remap, but by calling it
-- here we can compute the timing information.
index1D = rect_to_sphere:get_index_and_mask()
perElement = index1D:nElement()
time = sys.toc()
printf(" - make map %2.4fs %2.4es per px", time, time*perElement)
sys.tic()
force  = true 
lambda = 0
phi    = 0
delta  = 2 * pi / 10
for i = 1,#images do 
   img = image.load(images[i])
   lambda = lambda + delta
   proj_from:set_lambda_phi(lambda,phi)
   img_out = rect_to_sphere:remap(img,force)
   time = sys.toc()
   printf(" - reproject %2.4fs %2.4es per px", time, time*perElement)
   sys.tic()

   image.display{image={img_out}}
end
