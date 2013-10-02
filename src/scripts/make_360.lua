-- Class()

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
cmd:option('-imageext', 'JPG', 'filename extention of images to load')
cmd:text()

-- parse input params
params = cmd:parse(process.argv)

images_dir  = params.imagesdir
if not util.fs.is_dir(images_dir) then 
   error(string.format("Can't find directory: %s", images_dir))
end

image_ext  = params.imageext

images = util.fs.glob(images_dir,image_ext) 

if #images == 0 then 
   error(string.format("No images with ext %s found", image_ext))
end

img = image.load(images[1])

width  = img:size(3)
height = img:size(2)

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

log.tic()

indices     = {}
masks       = {}
out_images  = {}

rect_to_sphere = projection.Remap.new(proj_from,proj_to)

time = log.toc()
printf(" - make map %2.4fs", time)
log.tic()
force  = true 
lambda = 0
phi    = 0
delta  = 2 * pi / #images

for i = 1,#images do 

   img    = image.load(images[i])

   proj_from:set_lambda_phi(lambda,phi)
   index1D,stride,mask = rect_to_sphere:get_index_and_mask(force)
   img_out = rect_to_sphere:remap(img)

   table.insert(indices,index1D)
   table.insert(masks,mask)
   table.insert(out_images,img_out)

   time = log.toc()
   printf(" - reproject %2.4fs", time)
   log.tic()

   lambda = lambda + delta
   collectgarbage()

end

-- blend

allimg = blend(out_images, masks)

image.display(allimg)
