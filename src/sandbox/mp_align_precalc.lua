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
cmd:option('-topdir', '/Users/lihui815/Projects/PICSDIR/Sweeps/scanner371_job362006', 'project directory')
cmd:option('-sweepnum', 'sweep_3', 'directory of particular sweep')
cmd:option('-imagesdir', 'JPG', 'directory with the images to load')
cmd:option('-outdir', 'output', 'directory to save images')

cmd:text()

arg = ''

-- parse input params
params = cmd:parse(arg)

imagesdir  = params.topdir..'/'..params.sweepnum..'/'..params.imagesdir
outdir = params.topdir..'/'..params.outdir

arg = ''

-- parse input params
params = cmd:parse(arg)

lab_images={}
rgb_images={}

-- images are vertical
vfov = (97/180) * pi
hfov = (74.8/180) * pi

--vfov = (83.15/180) * pi
--hfov = (63.62/180) * pi
--hfov = (68/180) * pi

force  = true 
lambda = 0
phi    = 0

--kitchen13
--precalc = torch.Tensor({0.4793, 0.5099, 0.4749, 0.4706, 0.4749, 0.5054, 0.5405, 0.4706, 0.4575, 0.4969, 0.4662, 0.2217, 0.7151})

--artgal8 1,6-13
best_delta_01 = torch.Tensor({0.7574, 0.7342, 0.7533, 0.7174, 0.7290, 0.7491, 0.7477, 1.0950})
best_delta_06 = torch.Tensor({0.7144, 0.7164, 0.7003, 0.7198, 0.8590, 0.7337, 0.7323, 1.1071})
best_delta_07 = torch.Tensor({0.9978, 0.7348, 0.7294, 0.7324, 0.7404, 0.8715, 0.7314, 0.7454})
best_delta_12 = torch.Tensor({0.7166, 0.7455, 0.7384, 0.7455, 0.7333, 0.7445, 0.7554, 1.1040})
best_delta_13 = torch.Tensor({0.8614, 0.7265, 0.7357, 0.9193, 0.6278, 0.7252, 0.7301, 0.9571})

--artgal8 15-17
best_delta_16 = torch.Tensor({0.6676, 0.6661, 0.6863, 0.9219, 0.7249, 0.7224, 0.7192, 1.1747})

precalculated = (best_delta_01 + best_delta_06 + best_delta_07 + best_delta_12 + best_delta_13)/5;

precalculated = best_delta_16

wiggle = 0.25;
maxquant = 25;

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
         local
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

force  = true 
lambda = 0
phi    = 0

p("Testing Image Projection")

log.tic()

mindist = torch.Tensor(#images);
mindist:fill(9999999999999999999999);
best_delta = precalculated:clone();
best_distance = torch.Tensor(#images);
best_area = torch.Tensor(#images);
best_quant = torch.Tensor(#images);


for i = 1,#images do

    collectgarbage()
    
    for dd = -maxquant,maxquant do
    
        collectgarbage()
    
        local delta_anchor =  precalculated[i]

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
        
        if area == 0 then
          ss = 10;
        end
        
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
    
    --image.save(outdir.."/bg_img_"..params.sweepnum..j.."_"..k.."_"..l..".jpg", bgim)
    image.save(outdir.."/bg_fmg_"..params.sweepnum..j.."_"..k.."_"..l..".jpg", bgf)
    
    
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

image.save(outdir.."/big_img_"..params.sweepnum..".jpg",big_image);
--image.save(outdir.."/big_mask_"..params.sweepnum..".jpg",big_mask);
