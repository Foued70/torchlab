--Class()

local fs = require 'fs'

pi = math.pi
pi2 = pi * 0.5

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Compute image projections')
cmd:text()
cmd:text('Options')
--cmd:option('-imagesdir', 'images/', 'directory with the images to load')
--nodal_ninja
cmd:option('-imagesdir', '/Users/lihui815/cloudlab/src/data/test/96_spring_kitchen/matterport_mount', 'directory with the images to load')

--cmd:option('-imagesdir', '/Users/lihui815/Projects/PICSDIR/SWEEPS/NikonD5100_130522', 'directory with the images to load')


cmd:text()

arg = ''

-- parse input params
params = cmd:parse(arg)


sweepnum = '/sweep_4'
outdir = 'images/kitchen'..sweepnum
imagesdir  = params.imagesdir..sweepnum

loaded_images={}
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
         table.insert(loaded_images,imglab);
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
scale  = 1/8

-- images are vertical
vfov = (97/180) * pi
hfov = (74.8/180) * pi

--vfov = (83.15/180) * pi
--hfov = (63.62/180) * pi

-- output full equirectangular
out_width  = 2048
out_height = 1024
out_hfov   = 2 * pi
out_vfov   = pi

p("Testing Image Projection")

sys.tic()

mindist = torch.Tensor(#images);
mindist:fill(9999999999999999999999);
best_delta = torch.Tensor(#images);
best_distance = torch.Tensor(#images);
best_area = torch.Tensor(#images);
best_quant = torch.Tensor(#images);

force  = true 
lambda = 0
phi    = 0
delta_anchor  = 2 * pi / #images

wiggle = pi/6;
maxquant = 30;

for dd = -maxquant,maxquant do

    collectgarbage()
    
    local delta = delta_anchor + wiggle * (dd/maxquant);

    local proj_from1 = projection.GnomonicProjection.new(width,height,hfov,vfov)
    local proj_from2 = projection.GnomonicProjection.new(width,height,hfov,vfov)

    local proj_to1   = projection.SphericalProjection.new(width*2*scale,height*scale,2*hfov,vfov)
    local proj_to2   = projection.SphericalProjection.new(width*2*scale,height*scale,2*hfov,vfov)

    local rect_to_sphere1 = projection.Remap.new(proj_from1,proj_to1)
    local rect_to_sphere2 = projection.Remap.new(proj_from2,proj_to2)

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

    for i = 1,#images do

        collectgarbage()
   
        local j = (i-1) % #images + 1
        local k = (i+0) % #images + 1

        local img_l = loaded_images[j]
        local img_r = loaded_images[k]
   
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
        
        if ss < mindist[i] then
            best_delta[i] = delta;
            best_distance[i] = ss;
            mindist[i] = ss;
            best_area[i] = area;
            best_quant[i] = dd;
        
            printf("new best found for %d: %d %s %s %s", i, dd, delta, ss, area);
        
        end

    end

end

collectgarbage()

best_delta_orig = best_delta:clone();
adj = (2 * pi/best_delta:sum())
best_delta = best_delta * adj;

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
    
    --image.save(outdir.."/bg_img_"..j.."_"..k.."_"..l..".jpg", bgim)
    image.save(outdir.."/bg_fmg_"..j.."_"..k.."_"..l..".jpg", bgf)
    
    
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
image.save(outdir.."/bg_img.jpg",big_image);
big_image = big_image:cdiv(big_mask);
big_image = big_image - big_image:min();
big_image = big_image/(big_image:max());
--big_image = big_image*2;

image.save(outdir.."/big_img.jpg",big_image);
--image.save(outdir.."/big_mask.jpg",big_mask);
