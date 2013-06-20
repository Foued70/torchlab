Class()

--kitchen13
bstdlt1 = torch.Tensor({0.4847, 0.4747, 0.4697, 0.4522, 0.4672, 0.4522, 0.7197, 0.4722, 0.4472, 0.4672, 0.4622, 0.0071, 0.9072})
bstdlt2 = torch.Tensor({0.4831, 0.4831, 0.4806, 0.4658, 0.4708, 0.6046, 0.4732, 0.4633, 0.4658, 0.4708, 0.4782, 0.2253, 0.7187})
bstdlt3 = torch.Tensor({0.4696, 0.5988, 0.4819, 0.4919, 0.4969, 0.4869, 0.4844, 0.4770, 0.4696, 0.4696, 0.4696, 0.2781, 0.6088})
bstdlt4 = torch.Tensor({0.4870, 0.4845, 0.4670, 0.4670, 0.4720, 0.4820, 0.4620, 0.4620, 0.4570, 0.5794, 0.4520, 0.3845, 0.6269})

precalc = torch.Tensor({0.4811, 0.5103, 0.4748, 0.4692, 0.4767, 0.5064, 0.5348, 0.4686, 0.4599, 0.4968, 0.4655, 0.2238, 0.7154})

require 'image'

pi = math.pi
pi2 = pi * 0.5

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Compute image projections')
cmd:text()
cmd:text('Options')
cmd:option('-topdir', '/Users/lihui815/cloudlab/src/data/test/96_spring_kitchen', 'project directory')
cmd:option('-imagesdir', 'matterport_mount', 'directory with the images to load')
cmd:option('-sweepnum', 'sweep_1', 'directory of particular sweep')
cmd:option('-outdir', 'output', 'directory to save images')

cmd:text()

-- arg = ''

-- parse input params
params = cmd:parse(arg)

imagesdir  = params.topdir..'/'..params.imagesdir..'/'..params.sweepnum
if not util.fs.is_dir(imagesdir) then
   error("Must set a valid path to directory of images to process default -imagesdir images/")
end

outdir = params.topdir..'/'..params.outdir
sys.execute("mkdir -p " .. outdir)

-- images are vertical
vfov = (97/180) * pi
hfov = (74.8/180) * pi

force  = true
lambda = 0
phi    = 0

wiggle = 0.25;
maxquant = 30;

scale  = 1/8

loaded_images={}
rgb_images={}


images = util.fs.glob(imagesdir,"JPG")
images = util.fs.glob(imagesdir,"jpg",images)
images = util.fs.glob(imagesdir,"png",images)

collectgarbage()

img = image.load(images[1])

width  = img:size(3)
height = img:size(2)

p("Testing Image Projection")

sys.tic()

mindist       = torch.Tensor(#images);
mindist:fill(math.huge);
best_delta    = torch.Tensor(#images);
best_distance = torch.Tensor(#images);
best_area     = torch.Tensor(#images);
best_quant    = torch.Tensor(#images);

local proj_from1      = projection.GnomonicProjection.new(width,height,hfov,vfov)
local proj_from2      = projection.GnomonicProjection.new(width,height,hfov,vfov)

local proj_to1        = projection.SphericalProjection.new(width*2*scale,height*scale,2*hfov,vfov)
local proj_to2        = projection.SphericalProjection.new(width*2*scale,height*scale,2*hfov,vfov)

local rect_to_sphere1 = projection.Remap.new(proj_from1,proj_to1)
local rect_to_sphere2 = projection.Remap.new(proj_from2,proj_to2)

for i = 1,#images do
   collectgarbage()
   -- load images individually
   local img = image.load(images[i])
   if img:size(2) < img:size(3) then
      img = img:transpose(2,3)
   end
   local imglab = image.rgb2lab(img);
   local imgprev 
   for dd = -maxquant,maxquant do

      collectgarbage()

      local delta_anchor =  precalc[i]

      local delta = delta_anchor + wiggle * (dd/maxquant);

      proj_from1:set_lambda_phi(lambda-delta/2,phi)
      local index1D1,stride1,mask1 = rect_to_sphere1:get_offset_and_mask(force)

      proj_from2:set_lambda_phi(lambda+delta/2,phi)
      local index1D2,stride2,mask2 = rect_to_sphere2:get_offset_and_mask(force)

      local mm1 = mask1-1;
      local mm2 = mask2-1;
      local overlap_mask = mm1:cmul(mm2);
      local overlap_mask=overlap_mask:repeatTensor(3,1,1);
      local overlap_mask=overlap_mask:type('torch.DoubleTensor');

      local ss = 0
      local area = overlap_mask:sum();

      collectgarbage()

      local j = (i-1) % #images + 1

      local img_l = imgprev or image.rgb2lab(image.load(images[j]))
      local img_r = imglab

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
         best_delta[i]    = delta;
         best_distance[i] = ss;
         mindist[i]       = ss;
         best_area[i]     = area;
         best_quant[i]    = dd;

         printf("new best found for %d: %d %s %s %s", i, dd, delta, ss, area);

      end

   end
   imgprev = imglab
   collectgarbage()

end

collectgarbage()

best_delta_orig = best_delta:clone();
adj = (2 * pi/best_delta:sum())
best_delta = best_delta * adj;

collectgarbage()

mapped_images = {}
masks = {}

delta = 0;
sc = 1/4;

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
print(best_delta)
