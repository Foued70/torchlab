blend = projection.util.blend

Class()

pi  = math.pi
pi2 = pi * 0.5

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Compute image projections')
cmd:text()
cmd:text('Options')
cmd:option('-imagesdir', 'matterport_mount', 'directory with the images to load')
cmd:option('-fileglob', '', 'string to match images')
cmd:option('-scale', 1, 'how much to down scale original image')
cmd:option('-enblend', false, 'use enblend to make final image')
cmd:option('-outimage', 'output_360.png', 'filename of image output')

cmd:text()

-- arg = ''

-- parse input params
params = cmd:parse(process.argv)

imagesdir  = params.imagesdir
if not util.fs.is_dir(imagesdir) then
   error("Must set a valid path to directory of images to process default -imagesdir images/")
end


-- images are vertical
vfov = (97/180) * pi
hfov = (74.8/180) * pi

force  = true

fileglob = params.fileglob
if fileglob == '' then
   fileglob = {"tiff","png","jpg","JPG"}
end

_G.image_files = util.fs.glob(imagesdir,fileglob)

if #image_files == 0 then
   error("No images found")
end

-- get size of last image
_G.image_wand   = image.Wand.new(image_files[#image_files])
  w,h           = image_wand:size()
_G.scale        = params.scale
_G.height       = math.floor(h*scale)
_G.width        = math.floor(w*scale)
image_wand:size(width,height)
print("image size:", image_wand:size())

rad_per_pixel = hfov / (2*width)

-- load last image
img_l = image_wand:toTensor('float',"RGB","DHW")

p("Aligning images")

log.tic()

mindist       = torch.Tensor(#image_files);
mindist:fill(math.huge);
precalc            = 2*pi/#image_files
lambda_wiggle_base = hfov / 4 -- in radians (portion of precalc)
phi_wiggle_base    = vfov / 10 -- in radians (portion of precalc)
maxquant           = 2; -- steps of wiggle from -maxquant to maxquant

_G.best_delta = torch.Tensor(2,#image_files)
best_delta[1]:fill(precalc)
best_delta[2]:fill(0)

best_distance = torch.Tensor(#image_files);
best_area     = torch.Tensor(#image_files);
best_quant    = torch.Tensor(#image_files);

proj_from     = projection.GnomonicProjection.new(width,height,hfov,vfov)

canvas_hfov   = hfov + precalc + lambda_wiggle_base
canvas_vfov   = vfov + 2 * phi_wiggle_base
canvas_width  = width  * canvas_hfov/hfov
canvas_height = height * canvas_vfov/vfov

proj_to       = projection.SphericalProjection.new(canvas_width,canvas_height,canvas_hfov,canvas_vfov)

rect_to_sphere = projection.Remap.new(proj_from,proj_to)

done_lambda = {}
-- function find_best_lambda(image_files,best_delta
-- find horizontal offsets
-- go through all images find lambda
for i = 1,#image_files do
   collectgarbage()

   -- left image is at negative the simple pie split.
   proj_from:set_lambda_phi(-precalc,0)
   index1D1,stride1,mask1 = rect_to_sphere:get_offset_and_mask(force)
   mm1 = mask1:eq(0);

   _G.img_out_l = rect_to_sphere:remap(img_l)

   -- load right image and set size
   image_wand:load(image_files[i])
   image_wand:size(width,height)
   img_r = image_wand:toTensor('float',"RGB","DHW")

   lambda_anchor = 0
   phi_anchor    = 0
   lambda_wiggle = lambda_wiggle_base
   done_lambda = {}
   while (lambda_wiggle >= rad_per_pixel) do

      best_lambda = lambda_anchor
      count       = 1
      bestsum     = 0
      bestdiff    = 0

      -- do horizontal
      for dd = -maxquant,maxquant do
         collectgarbage()
         lambda = lambda_anchor + lambda_wiggle * (dd/maxquant);
         if (done_lambda[lambda]) then
            printf(" - skipping lambda: %f = ss %f",lambda,done_lambda[lambda])
         else
            printf(" - testing lambda: %f", lambda)
            
            proj_from:set_lambda_phi(lambda,phi_anchor)
            index1D2,stride2,mask2 = rect_to_sphere:get_offset_and_mask(force)
            
            mm2 = mask2:eq(0)
            overlap_mask = mm2:cmul(mm1)
            
            area = overlap_mask:sum();
            
            _G.img_out_r = rect_to_sphere:remap(img_r)
            
            imdiff = img_out_l - img_out_r
            
            imdist = imdiff:abs():sum(1):squeeze():cmul(overlap_mask:float())
            
            ss = imdist:sum()/area;
            if ss < mindist[i] then
               best_lambda      = lambda
               best_delta[1][i] = precalc + lambda
               best_distance[i] = ss;
               mindist[i]       = ss;
               best_area[i]     = area;
               best_quant[i]    = dd;
               printf("new best lambda found for %d at %d: %d %s %s %s", i, count, dd, lambda, ss, area);
               
            end
            done_lambda[lambda] = ss
            count = count + 1
         end
      end
      -- recenter on best lambda
      lambda_anchor = best_lambda
      -- split the lambda_wiggle
      lambda_wiggle = lambda_wiggle / maxquant
      -- store for debug
   end
   img_l = img_r
end

allmindist = mindist:sum()
best_phi = 0
phi_wiggle = phi_wiggle_base
done_phi = {}
-- do vertical
while (phi_wiggle >= rad_per_pixel) do
   count = 1
   for dd = -maxquant,maxquant do
      collectgarbage()
      lambda = best_delta[1][1]
      best_phi = phi_anchor
      phi = phi_anchor + phi_wiggle * (dd/maxquant);
      if done_phi[phi] then 

         printf(" - skipping phi: %f ss: %f", phi, done_phi[phi])
      else
         printf(" - testing phi: %f", phi)

         -- assume that phi will be the same for all images as this
         -- represents the camera not looking exactly at the plane
         -- of rotation, but slightly above or below.
         ss = 0
         image_wand:load(image_files[1])
         image_wand:size(width,height)
         img_l = image_wand:toTensor('float',"RGB","DHW")
         print("image size:", image_wand:size())
         
         

         for i = 2,#image_files do
            collectgarbage()
            lambda = precalc - best_delta[1][i]
            proj_from:set_lambda_phi(-precalc,phi)
            index1D1,stride1,mask1 = rect_to_sphere:get_offset_and_mask(force)
            mm1 = mask1:eq(0);
        
            _G.img_out_l = rect_to_sphere:remap(img_l)
            image_wand:load(image_files[i])
            image_wand:size(width,height)
            img_r = image_wand:toTensor('float',"RGB","DHW")
            
            proj_from:set_lambda_phi(lambda,phi)
            index1D2,stride2,mask2 = rect_to_sphere:get_offset_and_mask(force)
            _G.img_out_r = rect_to_sphere:remap(img_r)
            
            
            mm2 = mask2:eq(0)
            overlap_mask = mm2:cmul(mm1)
            
            area = overlap_mask:sum();
            
            imdiff = img_out_l - img_out_r
            
            imdist = imdiff:abs():sum(1):squeeze():cmul(overlap_mask:float())
            if area < 0 then
               print("Warning no overlap this should not happen")
            else
               ss = ss + imdist:sum()/area;
            end
            img_l = img_r
            mm1   = mm2
         end
         if ss < allmindist then
            best_phi         = phi
            allmindist       = ss;
            printf("new best phi found at %d: %d %s %s", count, dd, phi, ss);
         end
         done_phi[phi] = ss
         count = count + 1
      end
   end
   -- recenter on best lambda
   phi_anchor = best_phi
   -- split the lambda_wiggle
   phi_wiggle = phi_wiggle / maxquant
end

best_delta[2]:fill(best_phi)

_G.mapped_image_files = {}
_G.masks = {}
_G.out_fnames = ""

-- display

-- output full equirectangular
out_hfov   = 2 * pi + hfov
out_vfov   = vfov + 0.1 * vfov
out_height = height
out_width  = out_height * out_hfov / out_vfov

proj_to   = projection.SphericalProjection.new(out_width,out_height,out_hfov,out_vfov)

rect_to_sphere = projection.Remap.new(proj_from,proj_to)

lambda = best_delta[1][1];
phi    = best_delta[2][1]

for i = 1,#image_files do

   collectgarbage()

   proj_from:set_lambda_phi(lambda,phi)
   index1D,stride,mask = rect_to_sphere:get_offset_and_mask(force)

   image_wand:load(image_files[i])
   image_wand:size(width,height)

   img = image_wand:toTensor("double","RGBA","DHW")

   img_out = rect_to_sphere:remap(img)
   img_out[4]:copy(mask:eq(0):mul(255))

   table.insert(mapped_image_files,img_out)
   table.insert(masks,mask)
   if params.enblend then
      outname = string.format("tmp_%02d.png", i)
      image_wand:fromTensor(img_out,"RGBA","DHW")
      printf(" - saving %s %s",outname, image_wand:imagetype())
      image_wand:save(outname)
      out_fnames = out_fnames .. " " .. outname
   end

   if i<#image_files then
      lambda = lambda + best_delta[1][i+1]
      phi = best_delta[2][i+1]
   end
end

allimg = blend(mapped_image_files, masks)

image.display(allimg)

printf("saving %s", params.outimage)
image.save(params.outimage, allimg)

if (params.enblend) then
   os.execute(string.format("enblend %s -o %s", out_fnames, params.outimage))
   os.execute(string.format("rm %s", out_fnames))
end
