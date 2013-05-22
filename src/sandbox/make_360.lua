-- Class()

require 'image'

dofile 'util.lua'

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
         
      elseif (f:gmatch("JPG$")() or f:gmatch("png$")()) then
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
out_width  = 2048
out_height = 1024
out_hfov   = 2 * pi
out_vfov   = pi

proj_from = projection.GnomonicProjection.new(width,height,hfov,vfov)
proj_to   = projection.SphericalProjection.new(out_width,out_height,out_hfov,out_vfov)

p("Testing Image Projection")

sys.tic()

indices     = {}
masks       = {}
out_images  = {}

rect_to_sphere = projection.Remap.new(proj_from,proj_to)

time = sys.toc()
printf(" - make map %2.4fs", time)
sys.tic()
force  = true 
lambda = 0
phi    = 0
delta  = 2 * pi / 6

for i = 1,#images do 

   img    = image.load(images[i])

   proj_from:set_lambda_phi(lambda,phi)
   index1D,stride,mask = rect_to_sphere:get_index_and_mask(force)
   img_out = rect_to_sphere:remap(img)

   table.insert(indices,index1D)
   table.insert(masks,mask)
   table.insert(out_images,img_out)

   time = sys.toc()
   printf(" - reproject %2.4fs", time)
   sys.tic()

   lambda = lambda + delta
end

-- blend

allimg = blend(out_images, masks)

image.display(allimg)
