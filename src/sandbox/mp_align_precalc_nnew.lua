--Class()

local fs = require 'fs'
saliency = require 'saliency.init'

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Compute image projections')
cmd:text()
cmd:text('Options')
cmd:option('-topdir', '/Users/lihui815/Projects/PICSDIR/SWEEPS/jet-pizza-6780/scanner371_job362006/proces', 'project directory')
cmd:option('-sweepnum', 'sweep_3', 'directory of particular sweep')
cmd:option('-imagesdir', 'JPG', 'directory with the images to load')
cmd:option('-outdir', 'output', 'directory to save images')

cmd:text()

arg = ''

-- parse input params
params = cmd:parse(arg)

pi = math.pi
pi2 = pi * 0.5

--kitchen13
--precalculated = torch.Tensor({0.4811, 0.5103, 0.4748, 0.4692, 0.4767, 0.5064, 0.5348, 0.4686, 0.4599, 0.4968, 0.4655, 0.2238, 0.7154})

--artgal8 1,6-13
best_delta_01 = torch.Tensor({0.7574, 0.7342, 0.7533, 0.7174, 0.7290, 0.7491, 0.7477, 1.0950})
best_delta_06 = torch.Tensor({0.7144, 0.7164, 0.7003, 0.7198, 0.8590, 0.7337, 0.7323, 1.1071})
best_delta_07 = torch.Tensor({0.9978, 0.7348, 0.7294, 0.7324, 0.7404, 0.8715, 0.7314, 0.7454})
best_delta_12 = torch.Tensor({0.7166, 0.7455, 0.7384, 0.7455, 0.7333, 0.7445, 0.7554, 1.1040})
best_delta_13 = torch.Tensor({0.8614, 0.7265, 0.7357, 0.9193, 0.6278, 0.7252, 0.7301, 0.9571})

--artgal8 15-17
best_delta_16 = torch.Tensor({0.6676, 0.6661, 0.6863, 0.9219, 0.7249, 0.7224, 0.7192, 1.1747})

precalculated = (best_delta_01 + best_delta_06 + best_delta_07 + best_delta_12 + best_delta_13)/5;
--precalculated = (best_delta_01)/1;

precalculated = torch.Tensor({0.7395, 0.7415, 0.7564, 0.7519, 0.7329, 0.7298, 0.7294, 1.1017})

precalculated = precalculated * (2 * pi) / precalculated:sum();

--vfov_anchor = (83.0/180)*pi
--hfov_anchor = (63.5/180)*pi
vfov_anchor = (97.0/180)*pi
hfov_anchor = (74.8/180)*pi
phi_anchor = 0;
delta_anchor = precalculated:clone();

vfov_wiggle = (1.0/180)*pi
vfov_quant = 0;

hfov_wiggle = (0.5/180)*pi
hfov_quant = 0;

phi_wiggle = (0.25/180)*pi
phi_quant = 0;

delta_wiggle = 0.25;
delta_quant = 50;

delta_sum_tolerance = (0.1/180)*pi

mindist = math.huge;
best_vfov = vfov_anchor;
best_hfov = hfov_anchor;
best_phi = phi_anchor;
best_delta = precalculated:clone();

imagesdir  = params.topdir..'/'..params.sweepnum..'/'..params.imagesdir
outdir = params.topdir..'/'..params.outdir

lab_images={}
rgb_images={}
smp_images={}

-- load images
if not images then
   images = {}
   fnames = {}
   if not util.fs.is_dir(imagesdir) then 
      error("Must set a valid path to directory of images to process default -imagesdir images/")
   end
   imgfiles = fs.readdirSync(imagesdir)
   imgfiles() -- .
   imgfiles() -- ..
   for f in imgfiles do
      if f == ".DS_Store" then -- exclude OS X automatically-created backup files
         printf("--- Skipping .DS_Store file")

      elseif (f:gmatch("jpg$")() or f:gmatch("png$")() or f:gmatch("JPG$")() or f:gmatch("PNG$")()) then
         imgfile = imagesdir.."/"..f
         table.insert(images, imgfile)
         table.insert(fnames, f);
         local img = image.load(imgfile)
         if img:size(2) < img:size(3) then
            img = img:transpose(2,3)
         end
         local imglab = image.rgb2lab(img);
         local smp = saliency.high_entropy_features(imglab[1],25,25)
         smp = smp / (smp:max() + 0.0001)
         imglab = imglab / (imglab:max() + 0.0001)
         table.insert(lab_images,imglab);
         table.insert(rgb_images,img);
         table.insert(smp_images,smp);
         printf("Found : %s", imgfile)
      end
      collectgarbage()
   end
end
collectgarbage()

img = image.load(images[1])

width  = img:size(3)
height = img:size(2)
scale  = 1/10

force  = true 
lambda = 0

-- images are vertical

p("Testing Image Projection")

log.tic()

for pp = -phi_quant,phi_quant do
  
  local phi = phi_anchor + phi_wiggle * (pp/(phi_quant + 0.0001));

  for vv = -vfov_quant,vfov_quant do
  
    local vfov = vfov_anchor + vfov_wiggle * (vv/(vfov_quant + 0.0001));

    for hh = -hfov_quant,hfov_quant do

      local hfov = hfov_anchor + hfov_wiggle * (hh/(hfov_quant + 0.0001))

      local proj_from1 = projection.GnomonicProjection.new(width,height,hfov,vfov)
      local proj_from2 = projection.GnomonicProjection.new(width,height,hfov,vfov)

      local proj_to1   = projection.SphericalProjection.new(width*2*scale,height*scale,2*hfov,vfov)
      local proj_to2   = projection.SphericalProjection.new(width*2*scale,height*scale,2*hfov,vfov)

      local rect_to_sphere1 = projection.Remap.new(proj_from1,proj_to1)
      local rect_to_sphere2 = projection.Remap.new(proj_from2,proj_to2)
      
      -- create a delta_partition such that sum(delta) = 2*pi
      
      local cost_matrix = torch.zeros(#images, 2 * delta_quant + 1);
      local quant_mat = torch.zeros(#images);
      local dist_mat = torch.ones(#images) * math.huge;

      for i = 1,#images do
      
        local j = (i-1) % #images + 1
        local k = (i+0) % #images + 1

        local img_l = lab_images[j]
        local img_r = lab_images[k]
          
        local smp_l = smp_images[j]
        local smp_r = smp_images[k]

        for d=-delta_quant,delta_quant do
        
          collectgarbage();

          local delta = delta_anchor[i] + delta_wiggle * (d/(delta_quant + 0.0001));

          proj_from1:set_lambda_phi(lambda-delta/2,phi)
          index1D1,stride1,mask1 = rect_to_sphere1:get_offset_and_mask(force)

          proj_from2:set_lambda_phi(lambda+delta/2,phi)
          index1D2,stride2,mask2 = rect_to_sphere2:get_offset_and_mask(force)

          local mm1 = mask1-1;
          local mm2 = mask2-1;
          local overlap_mask = mm1:cmul(mm2);
          local overlap_mask=overlap_mask:repeatTensor(3,1,1);
          local overlap_mask=overlap_mask:type('torch.DoubleTensor');

          local ss = 0
          local area = overlap_mask:sum();

          collectgarbage()
   
          local img_out_l = rect_to_sphere1:remap(img_l)
          local img_out_r = rect_to_sphere2:remap(img_r)
          
          local smp_out_l = rect_to_sphere1:remap(smp_l)
          local smp_out_r = rect_to_sphere2:remap(smp_r)

          local imo_l = img_out_l:clone():cmul(overlap_mask);
          local imo_r = img_out_r:clone():cmul(overlap_mask);
          
          local smo_l = smp_out_l:clone():cmul(overlap_mask[1]);
          local smo_r = smp_out_r:clone():cmul(overlap_mask[1]);

          local imdiff = imo_l-imo_r;
          local smdiff = smo_l-smo_r;
          local imdist = smdiff:cmul(smdiff);
          ---imdist = torch.zeros(imdiff:size(2), imdiff:size(3))
          for c=1,imdiff:size(1) do
            imdist = imdist + imdiff[c]:cmul(imdiff[c])
          end
          imdist:sqrt();
    
          ss = imdist:sum()/(area+0.0001);
          
          if area == 0 then
            ss = 1.0
          end
          
          cost_matrix[i][d+delta_quant+1] = ss;
          
          if ss < dist_mat[i] then
            quant_mat[i] = d;
            dist_mat[i]=ss;
            printf("for image %d, the best [lambda, phi, delta, vfov, hfov, d] is [%s, %s, %s, %s, %s, %s]", i, lambda, phi, delta, vfov, hfov, d)
          end

        end
        
        collectgarbage()

      end
      
      local dd = quant_mat:clone()
      local cnt = 0;
      local dd_sum = dd:sum()

      local deltas_partition = delta_anchor + (dd * (delta_wiggle / (delta_quant + 0.0001)));
      
      while not(dd_sum == 0) do
      
        local next_cost = math.huge
        local next_dd = 1
        local next_im = 0
        
        if dd_sum < 0 then
        
          for i = 1,#images do
            if dd[i] < delta_quant then
              local cd = dd[i]
              local nd = dd[i]+1
              local nc = cost_matrix[i][nd+delta_quant+1]-cost_matrix[i][cd+delta_quant+1]
              printf("235 - i: %d, cd: %s, nd: %s, nc %s", i, cd, nd, nc)  
              if nc < next_cost then
                next_cost = cost_matrix[i][nd+delta_quant+1]
                next_dd = nd
                next_im = i
              end
            end
          end

        else
          
          for i = 1,#images do
            if dd[i] > -delta_quant then
              local cd = dd[i]
              local nd = dd[i]-1
              local nc = cost_matrix[i][nd+delta_quant+1]-cost_matrix[i][cd+delta_quant+1]
              printf("251 - i: %d, cd: %s, nd: %s, nc %s", i, cd, nd, nc)  
              if nc < next_cost then
                next_cost = cost_matrix[i][nd+delta_quant+1]
                next_dd = nd
                next_im = i
              end
            end
          end

        end
        
        collectgarbage()
        
        printf("tochange - changed_image: %d, dd_sum: %d, next_dd: %d, cost_change: %s", next_im, dd_sum, next_dd, next_cost)
        
        print(dd)
        dd[next_im] = next_dd
        print(dd)
        dd_sum = dd:sum()
        
      end
      
      collectgarbage()
      
      local ss = 0
      for i = 1, #images do
        ss = ss + cost_matrix[i][dd[i]+delta_quant+1]
      end
      
      if ss < mindist then
        best_delta = delta_anchor + dd * (delta_wiggle/(delta_quant + 0.0001));
        best_vfov = vfov
        best_hfov = hfov
        best_phi = phi
      end
  
    end -- phi
  
    collectgarbage()

  end --hfov
  
  collectgarbage()

end --vfov

collectgarbage()

printf('mindist: %s, [vfov, hfov, phi]: [%s, %s, %s]', mindist, best_vfov, best_hfov, best_phi);
print(best_delta);

best_delta_orig = best_delta:clone();
adj = (2 * pi/best_delta:sum())
best_delta = best_delta * adj;

hfov = best_hfov;
vfov = best_vfov;
phi = best_phi;

collectgarbage()

mapped_images = {}
masks = {}

delta = 0;
sc = 1/5;

local proj_from = projection.GnomonicProjection.new(width,height,hfov,vfov)

local proj_to   = projection.SphericalProjection.new(width*((2*pi+hfov)/hfov)*sc,height*sc,2*pi+hfov,vfov)

local rect_to_sphere = projection.Remap.new(proj_from,proj_to)

for i = 1,#images do
    
    collectgarbage()

    proj_from:set_lambda_phi(lambda+delta,phi)
    local index1D,stride,mask = rect_to_sphere:get_offset_and_mask(force)

    local mm = mask:repeatTensor(3,1,1);
    mm = mm:type('torch.DoubleTensor');

    local img = rgb_images[i]

    local img_out = rect_to_sphere:remap(img)

    table.insert(mapped_images,img_out)
    table.insert(masks,mm)
    
    delta = delta + best_delta[i]
    
end

collectgarbage()

big_image = mapped_images[1]
big_mask = -masks[1]+1

for i = 2,#images do
    big_image = big_image + mapped_images[i]
    big_mask = big_mask + 1 - masks[i]
end

collectgarbage()

big_mask = big_mask + 0.00000001
big_image = big_image:cdiv(big_mask);

image.save(outdir.."/big_img_"..params.sweepnum..".jpg",big_image);
