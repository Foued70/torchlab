--Class()

local fs = require 'fs'

pi = math.pi
pi2 = pi * 0.5

--kitchen13
--precalc = torch.Tensor({0.4811, 0.5103, 0.4748, 0.4692, 0.4767, 0.5064, 0.5348, 0.4686, 0.4599, 0.4968, 0.4655, 0.2238, 0.7154})

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

vfov_anchor = (83.0/180)*pi
hfov_anchor = (63.5/180)*pi
phi_anchor = 0;
delta_anchor = precalculated:clone();

vfov_wiggle = (0.25/180)*pi
vfov_quant = 0;

hfov_wiggle = (0.25/180)*pi
hfov_quant = 0;

phi_wiggle = (0.5/180)*pi
phi_quant = 0;

delta_wiggle = 0.25;
delta_quant = 12;

delta_sum_tolerance = (0.1/180)*pi

--mindist = torch.Tensor(#images);
--mindist:fill(9999999999999999999999);
--best_delta = precalculated:clone();
--best_distance = torch.Tensor(#images);
--best_area = torch.Tensor(#images);
--best_quant = torch.Tensor(#images);

mindist = 9999999999999999999999;
best_vfov = vfov_anchor;
best_hfov = hfov_anchor;
best_phi = phi_anchor;
best_delta = precalculated:clone();

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Compute image projections')
cmd:text()
cmd:text('Options')
cmd:option('-topdir', '/Users/lihui815/Projects/PICSDIR/Sweeps/NikonD5100_130522', 'project directory')
cmd:option('-sweepnum', 'sweep_2', 'directory of particular sweep')
cmd:option('-imagesdir', 'JPG', 'directory with the images to load')
cmd:option('-outdir', 'output', 'directory to save images')

cmd:text()

arg = ''

-- parse input params
params = cmd:parse(arg)

imagesdir  = params.topdir..'/'..params.sweepnum..'/'..params.imagesdir
outdir = params.topdir..'/'..params.outdir

lab_images={}
rgb_images={}

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
         table.insert(lab_images,imglab);
         table.insert(rgb_images,img);
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
      
      -- create a delta_partition such that sum(delta) = 2*pi
      
      local cost_matrix = torch.zeros(#images, 2 * delta_quant + 1);

      for i = 1,#images do
      
        for d=-delta_quant,delta_quant do
          
          collectgarbage();

          local delta = delta_anchor[i] + delta_wiggle * (d/(delta_quant + 0.0001));

          local proj_from1 = projection.GnomonicProjection.new(width,height,hfov,vfov)
          local proj_from2 = projection.GnomonicProjection.new(width,height,hfov,vfov)

          local proj_to1   = projection.SphericalProjection.new(width*2*scale,height*scale,2*hfov,vfov)
          local proj_to2   = projection.SphericalProjection.new(width*2*scale,height*scale,2*hfov,vfov)

          local rect_to_sphere1 = projection.Remap.new(proj_from1,proj_to1)
          local rect_to_sphere2 = projection.Remap.new(proj_from2,proj_to2)
          
          print(lambda)
          print(phi)
          print(delta)
          print(d)

          proj_from1:set_lambda_phi(lambda-delta/2,phi)
          local index1D1,stride1,mask1 = rect_to_sphere1:get_index_and_mask(force)

          proj_from2:set_lambda_phi(lambda+delta/2,phi)
          local index1D2,stride2,mask2 = rect_to_sphere2:get_index_and_mask(force)

          local mm1 = mask1-1;
          local mm2 = mask2-1;
          local overlap_mask = mm1:cmul(mm2);
          local overlap_mask=overlap_mask:repeatTensor(3,1,1);
          local overlap_mask=overlap_mask:type('torch.DoubleTensor');

          local ss = 0
          local area = overlap_mask:sum();

          collectgarbage()
   
          local j = (i-1) % #images + 1
          local k = (i+0) % #images + 1

          local img_l = lab_images[j]
          local img_r = lab_images[k]
   
          local img_out_l = rect_to_sphere1:remap(img_l)
          local img_out_r = rect_to_sphere2:remap(img_r)

          local imo_l = img_out_l:clone():cmul(overlap_mask);
          local imo_r = img_out_r:clone():cmul(overlap_mask);

          local imdiff = imo_l-imo_r;
          local imdist = imdiff[1]:cmul(imdiff[1]);
          for c=2,imdiff:size(1) do
            imdist = imdist + imdiff[c]:cmul(imdiff[c])
          end
          imdist:sqrt();
    
          ss = imdist:sum()/area;
          
          cost_matrix[i][d+delta_quant+1] = ss;

        end

      end
      
      local dd = torch.zeros(#images)
      local cnt = 0;

      while cnt <= math.pow((2 * delta_quant + 1),#images) do

        local deltas_partition = delta_anchor + (dd * (delta_wiggle / (delta_quant + 0.001)));
      
        if (deltas_partition:sum() > 2 * pi - delta_sum_tolerance) and (deltas_partition:sum() < 2 * pi + delta_sum_tolerance) then

          ss = 0
          for i = 1,#images do
            ss = ss + cost_matrix[i][dd[i]+delta_quant+1]
          end
          
          if ss < mindist then
            print(deltas_partition);
            print(dd);
            print(ss);

            best_delta = deltas_partition:clone();
            best_vfov = vfov;
            best_hfov = hfov;
            best_phi = phi;
            mindist = ss;
            print(ss)
            print(vfov)
            print(hfov)
            print(phi)
            printf("new best found: %s [%s %s %s]", ss, vfov, hfov, phi);
            print(deltas_partition);
      
          end

          collectgarbage()

        end -- if deltasum
      
        collectgarbage()

        local set = 1;
        if cnt % 10000 == 0 then
          print(cnt)
        end
        
        while set <= #images do
          
          if dd[set] >= 0 then
            
            if dd[set] < delta_quant then
              
              dd[set] = -(dd[set] + 1)
              set = #images
            
            else

              dd[set] = 0;
            end
          
          else
          
            dd[set] = -dd[set]
            set = #images
          
          end

          set = set + 1
        end

        cnt = cnt + 1
        
      end -- while for dd
    
      collectgarbage()
  
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

for i = 1,#images do
    
    collectgarbage()

    local proj_from = projection.GnomonicProjection.new(width,height,hfov,vfov)
    
    local proj_to   = projection.SphericalProjection.new(width*((2*pi+hfov)/hfov)*sc,height*sc,2*pi+hfov,vfov)

    local rect_to_sphere = projection.Remap.new(proj_from,proj_to)

    proj_from:set_lambda_phi(lambda+delta,phi)
    local index1D,stride,mask = rect_to_sphere:get_index_and_mask(force)

    local mm = mask:repeatTensor(3,1,1);
    mm = mm:type('torch.DoubleTensor');

    local img = rgb_images[i]

    local img_out = rect_to_sphere:remap(img)

    table.insert(mapped_images,img_out)
    table.insert(masks,mm)
    
    delta = delta + best_delta[i]
    
    local proj_from1 = projection.GnomonicProjection.new(width,height,hfov,vfov)
    local proj_from2 = projection.GnomonicProjection.new(width,height,hfov,vfov)
    local proj_from3 = projection.GnomonicProjection.new(width,height,hfov,vfov)

    local proj_to1   = projection.SphericalProjection.new(width*3*scale,height*scale,hfov*3,vfov)
    local proj_to2   = projection.SphericalProjection.new(width*3*scale,height*scale,hfov*3,vfov)
    local proj_to3   = projection.SphericalProjection.new(width*3*scale,height*scale,hfov*3,vfov)
    
    local rect_to_sphere1 = projection.Remap.new(proj_from1,proj_to1)
    local rect_to_sphere2 = projection.Remap.new(proj_from2,proj_to2)
    local rect_to_sphere3 = projection.Remap.new(proj_from3,proj_to3)

    j = (i-1) % #images + 1
    k = (i+0) % #images + 1
    l = (i+1) % #images + 1

    proj_from1:set_lambda_phi(lambda-best_delta_orig[j],phi)
    local index1D1,stride1,mask1 = rect_to_sphere1:get_index_and_mask(force)
    
    proj_from2:set_lambda_phi(lambda,phi)
    local index1D2,stride2,mask2 = rect_to_sphere2:get_index_and_mask(force)
    
    proj_from3:set_lambda_phi(lambda+best_delta_orig[k],phi)
    local index1D3,stride3,mask3 = rect_to_sphere3:get_index_and_mask(force)
    
    local mm1 = mask1:repeatTensor(3,1,1);
    mm1 = mm1:type('torch.DoubleTensor');
    local mm2 = mask2:repeatTensor(3,1,1);
    mm2 = mm2:type('torch.DoubleTensor');
    local mm3 = mask3:repeatTensor(3,1,1);
    mm3 = mm3:type('torch.DoubleTensor');
    
    local img1 = rgb_images[j]
    local img2 = rgb_images[k]
    local img3 = rgb_images[l]

    local img_out1 = rect_to_sphere1:remap(img1)
    local img_out2 = rect_to_sphere2:remap(img2)
    local img_out3 = rect_to_sphere3:remap(img3)
    
    local bgim = img_out1 + img_out2 + img_out3
    local bgmk = - mm1 - mm2 - mm3 + 3
    local bgf = bgim:clone():cdiv(bgmk)
    
    --image.save(outdir.."/bg_img_"..params.sweepnum.."_"..j.."_"..k.."_"..l..".jpg", bgim)
    image.save(outdir.."/bg_fmg_"..params.sweepnum.."_"..j.."_"..k.."_"..l..".jpg", bgf)
    
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
image.save(outdir.."/bg_img_"..params.sweepnum..".jpg",big_image);
big_image = big_image:cdiv(big_mask);
big_image = big_image - big_image:min();
big_image = big_image/(big_image:max());
--big_image = big_image*2;

image.save(outdir.."/big_img_"..params.sweepnum..".jpg",big_image);
--image.save(outdir.."/big_mask_"..params.sweepnum..".jpg",big_mask);
