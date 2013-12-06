-- Class()
io    = require 'io'
os    = require 'os'
blend = projection.util.blend 

pi = math.pi
pi2 = pi * 0.5

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Make 360 from a linear 360 sweep of images')
cmd:text()
cmd:text('Options')
cmd:option('-imagesdir', 'images/', 'directory with the images to load')
cmd:option('-fileglob', '', 'string to match images')
cmd:option('-scale', 1, 'how much to down scale original image')
cmd:option('-enblend', false, 'use enblend to make final image')
cmd:option('-outimage', 'output_360.png', 'filename of image output')
cmd:text()

-- parse input params
params = cmd:parse(process.argv)

images_dir  = params.imagesdir
if not util.fs.is_dir(images_dir) then 
   error(string.format("Can't find directory: %s", images_dir))
end

fileglob = params.fileglob
if fileglob == '' then 
   fileglob = {"tiff","png","jpg","JPG"}
end
images = util.fs.glob(images_dir,fileglob) 

if #images == 0 then 
   error(string.format("No images with ext %s found", fileglob))
end

image_wand   = image.Wand.new(images[1])
height,width = image_wand:size()
if scale ~= 1 then 
   height = math.floor(height * params.scale)
   width = math.floor(width * params.scale)
   image_wand:size(width,height)
end


-- images are vertical
vfov = (97/180) * pi 
hfov = (74.22/180) * pi

-- output full equirectangular
out_hfov   = 2 * pi
out_vfov   = vfov
out_height = height 
out_width  = out_height * out_hfov / out_vfov

proj_from = projection.GnomonicProjection.new(width,height,hfov,vfov)
proj_to   = projection.SphericalProjection.new(out_width,out_height,out_hfov,out_vfov)

log.tic()

_G.masks       = {}
_G.out_images  = {}
_G.out_fnames  = ""
rect_to_sphere = projection.Remap.new(proj_from,proj_to)

time = log.toc()
printf(" - make map %2.4fs", time)
log.tic()
force  = true 
lambda = 0
phi    = 0
delta  = 2 * pi / #images

for i = 1,#images do 

   image_wand:load(images[i])
   image_wand:size(width,height)
   img = image_wand:toTensor('float',"RGBA","DHW")
   proj_from:set_lambda_phi(lambda,phi)
   index1D,stride,mask = rect_to_sphere:get_offset_and_mask(force)
   img_out = rect_to_sphere:remap(img)
   img_out[4]:copy(mask:eq(0):mul(255))
   time = log.toc()
   printf(" - reproject %2.4fs", time)
   log.tic()
   if not params.enblend then 
      table.insert(masks,mask)
      table.insert(out_images,img_out)
   else
      outname = string.format("tmp_%02d.png", i)
      image_wand:fromTensor(img_out,"RGBA","DHW")
      printf(" - saving %s %s",outname, image_wand:imagetype())
      image_wand:save(outname)
      out_fnames = out_fnames .. " " .. outname
   end
   time = log.toc()
   printf(" - save in %2.4fs", time)
   log.tic()

   lambda = lambda + delta
   collectgarbage()

end

-- blend

if (not params.enblend) then 
   allimg = blend(out_images, masks)
   
   image.display(allimg)
   
   printf("saving %s", params.outimage)
   image.save(params.outimage, allimg)
else 
   os.execute(string.format("enblend %s -o %s", out_fnames, params.outimage))
   os.execute(string.format("rm %s", out_fnames))
end
