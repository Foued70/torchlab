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

mindist            = torch.Tensor(#image_files);
mindist:fill(math.huge);
precalc            = 2*pi/#image_files
lambda_wiggle_base = hfov / 4 -- in radians (portion of precalc)
phi_wiggle_base    = vfov / 10 -- in radians (portion of precalc)

-- setup best first guess
_G.best_delta = torch.Tensor(2,#image_files)
best_delta[1]:fill(precalc)
best_delta[2]:fill(0)

_G.best_scores = torch.Tensor(#image_files);


proj_from     = projection.GnomonicProjection.new(width,height,hfov,vfov)

canvas_hfov   = hfov + precalc + lambda_wiggle_base
canvas_vfov   = vfov + 2 * phi_wiggle_base
canvas_width  = width  * canvas_hfov/hfov
canvas_height = height * canvas_vfov/vfov

proj_to       = projection.SphericalProjection.new(canvas_width,canvas_height,canvas_hfov,canvas_vfov)

rect_to_sphere = projection.Remap.new(proj_from,proj_to)

-- optimize <func()> with respect to <start_param>, and current best
-- result <best_result> by testing +,- <wiggle> and repeating binary
-- search by halving wiggle until wiggle is smaller that <stop>.
-- Arguments can be passed to <func()> with the args in {...}
function optimize(wiggle,stop,start_param,best_result,func,...)
   print("Optimizing")
   count = 0
   current_best_result = best_result
   best_param          = start_param
   while (wiggle >= stop) do 
      log.tic()
      for _,wig in pairs({ -wiggle,wiggle}) do
         param = best_param + wig
         test_val = func(param,...)
         if test_val < current_best_result then
            best_param          = param
            current_best_result = test_val;
            printf("new best found %2.4f %2.4f", best_param, current_best_result);
         end
      end
      printf(" - [%d] wiggle %2.4f tested in %2.2fs param: %2.4f, score: %2.4f",
             count,wiggle,log.toc()/1000, best_param, current_best_result)
      wiggle = wiggle / 2
      count = count + 1
   end
   return best_param, current_best_result
end

function compute_lambda_score(lambda,phi,image_right,mask_left,projected_image_left)
   proj_from:set_lambda_phi(lambda,phi)
   _,_,mask_right = rect_to_sphere:get_offset_and_mask(force)
            
   mask_right = mask_right:eq(0)
   overlap_mask = mask_right:cmul(mask_left)
            
   area = overlap_mask:sum();
            
   projected_image_right = rect_to_sphere:remap(image_right)
            
   image_diff = projected_image_left - projected_image_right
            
   image_dist = image_diff:abs():sum(1):squeeze():cmul(overlap_mask:float())
            
   return image_dist:sum()/area;
end

function find_best_lambda(image_files,best_delta,wiggle,stop)
   -- find horizontal offsets
   -- go through all images find lambda
   previ = #image_files
   image_wand:load(image_files[previ])
   image_wand:size(width,height)
   image_left = image_wand:toTensor('float',"RGB","DHW")
   
   -- do optimization for each pair
   for i = 1,#image_files do
      collectgarbage()
      -- setup left image is at negative current best offset.
      proj_from:set_lambda_phi(-best_delta[1][i],best_delta[2][previ])
      _,_,mask_left = rect_to_sphere:get_offset_and_mask(force)
      mask_left = mask_left:eq(0);

      projected_image_left = rect_to_sphere:remap(image_left)

      -- load right image and set size
      image_wand:load(image_files[i])
      image_wand:size(width,height)
      image_right = image_wand:toTensor('float',"RGB","DHW")

      -- difference from current best guess lambda
      lambda_delta  = 0
      phi           = best_delta[2][i]

      current_best = 
         compute_lambda_score(lambda_delta, phi,
                              image_right,
                              mask_left,projected_image_left)
      lambda_delta, score = 
         optimize(wiggle,stop,
                  lambda_delta,current_best,
                  compute_lambda_score,
                  phi,image_right,mask_left,projected_image_left)

      best_delta[1][i] = best_delta[1][i] + lambda_delta
      best_scores[i] = score;
      printf("new best lambda found for %d: %2.4f %2.4f", i, best_delta[1][i], score);
      
      image_left = image_right
      previ      = i
   end
end

function compute_phi_score(current_phi, image_files, best_delta)

   -- assume that phi will be the same for all images as this
   -- represents the camera not looking exactly at the plane
   -- of rotation, but slightly above or below.
   score = 0
   image_wand:load(image_files[#image_files])
   image_wand:size(width,height)
   image_left = image_wand:toTensor('float',"RGB","DHW")
   previ = #image_files
   for i = 1,#image_files do
      collectgarbage()
      lambda_left  = -best_delta[1][i]
      lambda_right = 0 
      proj_from:set_lambda_phi(lambda_left,current_phi)
      _,_,mask_left = rect_to_sphere:get_offset_and_mask(force)
      mask_left = mask_left:eq(0);
        
      projected_image_left = rect_to_sphere:remap(image_left)
      image_wand:load(image_files[i])
      image_wand:size(width,height)
      image_right = image_wand:toTensor('float',"RGB","DHW")
            
      proj_from:set_lambda_phi(lambda_right,current_phi)
      _,_,mask_right = rect_to_sphere:get_offset_and_mask(force)
      projected_image_right = rect_to_sphere:remap(image_right)
                        
      mask_right = mask_right:eq(0)
      overlap_mask = mask_right:cmul(mask_left)
      area = overlap_mask:sum()

      image_diff = projected_image_left - projected_image_right
            
      image_dist = image_diff:abs():sum(1):squeeze():cmul(overlap_mask:float())
      if area <= 0 then
         print("Warning no overlap this should not happen")
      else
         score = score + image_dist:sum()/area;
      end
      image_left = image_right
      mask_left  = mask_right
   end
   return score
end


function find_best_phi(image_files, best_delta, wiggle, stop, current_phi,current_best_score)
   wiggle      = wiggle or phi_wiggle_base
   stop        = stop or rad_per_pixel
   current_phi = current_phi or 0
   best_phi    = current_phi 
   current_best_score = current_best_score or compute_phi_score(current_phi,image_files,best_delta)
   printf("Adjusting global phi: %f", current_phi)
   test_phi, score = 
      optimize(wiggle,stop,
               best_phi,current_best_score,
               compute_phi_score,
               image_files,best_delta)
   
   if score < current_best_score then
      best_phi = test_phi
      current_best_score = score
   end
   best_delta[2]:fill(best_phi)
   printf("new best phi found %2.4f %2.4f", best_phi, current_best_score);
   return best_phi, current_best_score
end

function compute_vfov_score(current_vfov, image_files, best_delta)

   proj_from = 
      projection.GnomonicProjection.new(width,height,hfov,current_vfov)

   score = 0
   image_wand:load(image_files[#image_files])
   image_wand:size(width,height)
   image_left = image_wand:toTensor('float',"RGB","DHW")
   previ = #image_files
   for i = 1,#image_files do
      collectgarbage()
      lambda_left  = -best_delta[1][i]
      phi_left     = best_delta[2][previ]
      lambda_right = 0 
      phi_right    = best_delta[2][i]

      proj_from:set_lambda_phi(lambda_left,phi_left)
      _,_,mask_left = rect_to_sphere:get_offset_and_mask(force)
      mask_left = mask_left:eq(0);
        
      projected_image_left = rect_to_sphere:remap(image_left)
      image_wand:load(image_files[i])
      image_wand:size(width,height)
      image_right = image_wand:toTensor('float',"RGB","DHW")
            
      proj_from:set_lambda_phi(lambda_right,phi_right)
      _,_,mask_right = rect_to_sphere:get_offset_and_mask(force)
      projected_image_right = rect_to_sphere:remap(image_right)
                        
      mask_right   = mask_right:eq(0)
      overlap_mask = mask_right:cmul(mask_left)
      area         = overlap_mask:sum()

      image_diff = projected_image_left - projected_image_right
            
      image_dist = 
         image_diff:abs():sum(1):squeeze():cmul(overlap_mask:float())
      if area <= 0 then
         print("Warning no overlap this should not happen")
      else
         score = score + image_dist:sum()/area;
      end
      image_left = image_right
      mask_left  = mask_right
   end
   return score
end

function find_best_vfov(image_files, best_delta, wiggle, stop, current_vfov,current_best_score)
   wiggle       = wiggle or phi_wiggle_base
   stop         = stop or rad_per_pixel
   current_vfov = current_vfov or vfov
   best_vfov    = current_vfov
   current_best_score = current_best_score or 
      compute_vfov_score(current_vfov,image_files,best_delta)
   printf("Adjusting vfov: %f",current_vfov)
   test_vfov, score = 
      optimize(wiggle,stop,
               current_vfov,current_best_score,
               compute_vfov_score,
               image_files,best_delta)
   
   if score < current_best_score then
      best_vfov = test_vfov
      current_best_score = score
   end
   
   printf("new best vfov found %2.4f %2.4f", best_vfov, current_best_score);
   return best_vfov, current_best_score
end
   
find_best_lambda(image_files, best_delta, lambda_wiggle_base, rad_per_pixel)
best_score = best_scores:sum()
print(best_scores)
printf("current best_score: %f", best_score)
best_phi, best_score = find_best_phi(image_files, best_delta, phi_wiggle_base, rad_per_pixel, 0, best_score)
printf("current best_score: %f", best_score)
best_vfov, best_score = find_best_vfov(image_files, best_delta, vfov*0.05, rad_per_pixel, vfov, best_score)
printf("current best_score: %f", best_score)

-- display
_G.mapped_image_files = {}
_G.masks = {}
_G.out_fnames = ""

-- output full equirectangular
out_hfov   = 2 * pi + hfov
out_vfov   = vfov + 0.1 * vfov
out_height = height
out_width  = out_height * out_hfov / out_vfov

proj_from = projection.GnomonicProjection.new(width,height,hfov,best_vfov)
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
   os.execute(string.format("enblend %s -o enblend_%s", out_fnames, params.outimage))
   os.execute(string.format("rm %s", out_fnames))
end
