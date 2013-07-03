hough = require 'hough'
saliency = require 'saliency'
local fs = require 'fs'

pi = math.pi
pi2 = pi * 0.5
d2r = pi / 180

function load_alignment_images(imagesdir)
  local lab_images={}
  local rgb_images={}
  local smp_images={}
  if not images then
    images = {}
    fnames = {}
    if not util.fs.is_dir(imagesdir) then
      error("Must set a valid path to directory of images to process default -imagesdir images/")
    end
    imgfiles = fs.readdirSync(imagesdir)
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
  return images, lab_images, rgb_images, smp_images, fnames
end

function load_precalculated_deltas(numimages, option)
  local precalculated = torch.zeros(25)
  if option == 13 then
    --local bd = torch.Tensor(4,13)
    --bd[1] = torch.Tensor({0.4861, 0.4753, 0.4698, 0.4542, 0.4667, 0.4514, 0.7198, 0.4736, 0.4499, 0.4668, 0.4605, 0.0088, 0.9004}) --d_off=
    --bd[2] = torch.Tensor({}) --d_off=
    --bd[3] = torch.Tensor({}) --d_off=
    --bd[4] = torch.Tensor({}) --d_off=
    
    precalculated = torch.Tensor({0.4811, 0.5103, 0.4748, 0.4692, 0.4767, 0.5064, 0.5348, 0.4686, 0.4599, 0.4968, 0.4655, 0.2238, 0.7154})
  elseif option == 8 then
    local bd = torch.Tensor(6,8)
    bd[1] = torch.Tensor({0.7395, 0.7415, 0.7594, 0.7519, 0.7299, 0.7298, 0.7294, 1.1017}) --d_off=0.73043687258752
    bd[2] = torch.Tensor({0.8595, 0.9715, 0.7294, 0.7319, 0.7299, 0.7398, 0.7294, 0.7917}) --d_off=0.65731700170692
    bd[3] = torch.Tensor({0.7095, 0.9885, 0.7264, 0.7479, 0.7299, 0.7408, 0.7354, 0.9047}) --d_off=0.66728789319064
    bd[4] = torch.Tensor({0.7455, 0.7425, 0.7324, 0.7299, 0.7419, 0.7288, 0.7294, 1.1327}) --d_off=0.67725878467436
    bd[5] = torch.Tensor({0.7395, 0.7590, 0.8749, 0.7464, 0.7209, 0.7288, 0.6289, 1.0847}) --d_off=0.62740432725576
    bd[6] = torch.Tensor({0.7527, 0.7326, 0.8785, 0.8736, 0.7365, 0.7456, 0.7405, 0.8231}) --d_off=0.65399337121235
    precalculated = bd:mean(1)[1]
  elseif option == 6 then
    local bd = torch.Tensor(4,6)
    bd[1] = torch.Tensor({1.0472, 1.0422, 1.0322, 1.0472, 1.0472, 1.0672}) --d_off=1.0156043690219
    bd[2] = torch.Tensor({1.0472, 1.0497, 1.0472, 1.0597, 1.0547, 1.0247}) --d_off=6.0183196949309
    bd[3] = torch.Tensor({1.0447, 1.0447, 1.0472, 1.0622, 1.0447, 1.0397}) --d_off=5.8524838277737
    bd[4] = torch.Tensor({1.0397, 1.0372, 1.0472, 1.0497, 1.0472, 1.0622}) --d_off=3.5634463280735
    precalculated = bd:mean(1)[1]
  else
    if numimages > 0 then
      precalculated = torch.ones(numimages)
    end
  end
    
  precalculated = precalculated * (2 * pi) / precalculated:sum();
  return precalculated
  
end

function find_best_deltas(lab_imgages, smp_images, precalculated, fbd)
  local vfov_anchor=fbd.vfov_anchor
  local vfov_wiggle=fbd.vfov_wiggle
  local vfov_quant=fbd.vfov_quant

  local hfov_anchor=fbd.hfov_anchor
  local hfov_wiggle=fbd.hfov_wiggle
  local hfov_quant=fbd.hfov_quant

  local phi_anchor=fbd.phi_anchor
  local phi_wiggle=fbd.phi_wiggle
  local phi_quant=fbd.phi_quant

  local delta_anchor=fbd.delta_anchor
  local delta_wiggle=fbd.delta_wiggle
  local delta_quant=fbd.delta_quant
  
  local delta_sum_tolerance = (0.1/180)*pi
  local force  = true
  local lambda = 0
  local mindist = math.huge;
  local best_vfov = vfov_anchor;
  local best_hfov = hfov_anchor;
  local best_phi = phi_anchor;
  local best_delta = precalculated:clone();

  local img = lab_images[1]

  local width  = img:size(3)
  local height = img:size(2)
  local scale  = 1/10
  local numimages = #lab_images
  
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
    
        local cost_matrix = torch.zeros(numimages, 2 * delta_quant + 1);
        local quant_mat = torch.zeros(numimages);
        local dist_mat = torch.ones(numimages) * math.huge;

        for i = 1,numimages do
      
          local j = (i-1) % numimages + 1
          local k = (i+0) % numimages + 1

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
        
            for i = 1,numimages do
              if dd[i] < delta_quant then
                local cd = dd[i]
                local nd = dd[i]+1
                local nc = cost_matrix[i][nd+delta_quant+1]-cost_matrix[i][cd+delta_quant+1]
                if nc < next_cost then
                  next_cost = cost_matrix[i][nd+delta_quant+1]
                  next_dd = nd
                  next_im = i
                end
              end
            end

          else
          
            for i = 1,numimages do
              if dd[i] > -delta_quant then
                local cd = dd[i]
                local nd = dd[i]-1
                local nc = cost_matrix[i][nd+delta_quant+1]-cost_matrix[i][cd+delta_quant+1]
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
        for i = 1, numimages do
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
  
  printf('mindist: %s, [vfov, hfov, phi]: [%s, %s, %s]', mindist, best_vfov, best_hfov, best_phi);
  print(best_delta);

  local adj = (2 * pi/best_delta:sum())
  best_delta = best_delta * adj;
  
  return best_delta, best_vfov, best_hfov, best_phi

end

function score_band_overlap(mptex, mount, offset)
  local ret = 0
  local mptex_1 = mptex:clone():transpose(1,2)[1]
  local mount_1 = mount:clone():transpose(1,2)[1]
  local mount_offset = mount_1:clone()
  local numRad = mount_1:size(1)
  local off = offset % numRad
  if off > 0 then
    mount_offset:sub(off+1,numRad):add(0.000001):cdiv(mount_offset:sub(off+1,numRad)):cmul(mount_1:sub(1,numRad-off))
    mount_offset:sub(1,off):add(0.000001):cdiv(mount_offset:sub(1,off)):cmul(mount_1:sub(numRad-off+1,numRad))
  end
  ret = mptex_1:dist(mount_offset);
  return ret
end

function find_offset(mptex, mount)
  local off = 0
  local scr = math.huge
  local numRad = mptex:size(1)
  for i=0,numRad-1 do
    local score = score_band_overlap(mptex, mount, i)
    if score < scr then
      off = i
      scr = score
      print(off)
    end
    collectgarbage()
  end
  return off
end

function get_hough(img, numrads, numangs, numbest)

  img = img:type('torch.DoubleTensor')
  local lab = image.rgb2lab(img);
  local smp = saliency.high_entropy_features(lab[1],5,5)
  
  print('getting hough transform')

  local ht = hough.get_hough_transform_vertical(smp, numrads,numangs)

  print('applying local contrast normalization')

  ht = hough.local_contrast_normalization(ht);

  print('sorting lines')

  local sorted = hough.find_best_lines_vertical(ht,numbest)
  local sorted_t = sorted:transpose(1,2)
  sorted_t[3]=sorted_t[3]/sorted_t[3]:max()
  sorted = sorted_t:transpose(1,2)
  local in_order = torch.zeros(numbest,2)

  local ht_best = torch.zeros(numrads, numangs)

  for i=1,sorted:size(1) do
  
    local RR = sorted[i][1]
    local VV = sorted[i][3]
    if RR > 0 and RR <= numrads then
      for aa = 1,numangs do
        ht_best[RR][aa]=1.0
      end
      
      local ind = 1;
      while ind <= numbest do
        if in_order[ind][1] > RR or in_order[ind][1] == 0 then
          if ind < numbest then
            local iocl = in_order:clone()
            in_order:sub(ind+1, numbest):add(0.000001):cdiv(in_order:sub(ind+1, numbest)):cmul(iocl:sub(ind, numbest-1))
          end
          in_order[ind] = torch.Tensor({RR,VV})
          break
        end
        ind = ind + 1
      end -- while
      
    else
      numbest = i-1
      in_order=in_order:sub(1,numbest)
      break;
    end -- if RR > 0...
  
    printf('drawing %d th best line RR: %s, val: %s', i,RR/numrads,VV)
    
  end --for
  
  ht_best = blur_bands(in_order, ht_best)
  ht_best=ht_best:type('torch.DoubleTensor')
  
  print(ht_best:size())
  
  return ht_best

end

function blur_bands(in_order, ht_best)
  
  local numbest = in_order:size(1)
  local numrads = ht_best:size(1)
  local numangs = ht_best:size(2)
  
  for i=1,numbest do
    
    local j = (i) % numbest + 1
    local r1 = in_order[i][1]
    local r2 = in_order[j][1]
    
    if r1 > r2 then
      r2 = r2 + numrads
    end
    
    local dist = (r2 - r1 + 1)/2
    local rmid = r1 + dist -1
    
    for cc = r1+1,r2-1 do
      local v = (math.pow(1 - (math.min(cc,rmid)-r1)/dist,2)*in_order[i][2]+math.pow(1 - (r2-math.max(rmid,cc))/dist,2)*in_order[j][2])
      for aa = 1,numangs do
        ht_best[(cc-1)%numrads+1][aa]=v
      end
    end
  end
  
  return ht_best
  
end

function remap_images_to_panorama(rgb_images, best_delta, ritp)

  local lambda = ritp.lambda
  local phi = ritp.phi
  local sc = ritp.sc
  local width_fr = ritp.width_from
  local height_fr = ritp.height_from
  local hfov_fr = ritp.hfov_from
  local vfov_fr = ritp.vfov_from
  local width_to = ritp.width_to
  local height_to = ritp.height_to
  local hfov_to = ritp.hfov_to
  local vfov_to = ritp.vfov_to
  local force = true
  
  local mapped_images_f = {}
  local masks_f = {}

  local delta = 0
  local proj_from = projection.GnomonicProjection.new(width_fr,height_fr,hfov_fr,vfov_fr)
  local proj_to   = projection.SphericalProjection.new(width_to*sc,height_to*sc,hfov_to,vfov_to)
  local rect_to_sphere = projection.Remap.new(proj_from,proj_to)

  for i = 1,#rgb_images do
    
    collectgarbage()

    proj_from:set_lambda_phi(lambda+delta,phi)
    local index1D,stride,mask = rect_to_sphere:get_offset_and_mask(force)

    local mm = mask:repeatTensor(3,1,1);
    mm = mm:type('torch.DoubleTensor');

    local img = rgb_images[i]
    print(i)
    local img_out = rect_to_sphere:remap(img)

    table.insert(mapped_images_f,img_out)
    table.insert(masks_f,mm)
    
    delta = delta + best_delta[i]
    
  end

  collectgarbage()

  local big_image_f = mapped_images_f[1]
  local big_mask_f = -masks_f[1]+1

  for i = 2,#rgb_images do
    big_image_f = big_image_f + mapped_images_f[i]
    big_mask_f = big_mask_f + 1 - masks_f[i]
  end
  
  big_mask_f = big_mask_f + 0.00000001
  big_image_f = big_image_f:cdiv(big_mask_f);

  return big_image_f
  
end

