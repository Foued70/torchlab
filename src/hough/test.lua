hough = require 'hough'
saliency = require 'saliency'
require 'image'


cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Compute image projections')
cmd:text()
cmd:text('Options')
cmd:option('-image', '/Users/lihui815/Projects/PICSDIR/SWEEPS/jet-pizza-6780/scanner371_job362006/proces/sweep_3/JPG/DSC_0009.jpg', 'image file path')

cmd:text()

-- arg = ''

-- parse input params
params = cmd:parse(arg)
dname = paths.dirname(params.image)
fname = paths.basename(params.image)

img = image.load(params.image):type('torch.DoubleTensor')
img = image.scale(img, 1580,412)
lab = image.rgb2lab(img);
smp = saliency.high_entropy_features(lab[1],5,5)

print('getting hough transform')
ht = hough.get_hough_transform(smp, 1000,1080)
disp1 = ht:clone()/ht:max()
image.save(dname.."/disp_1_"..fname, disp1);
disp1 = image.scale(disp1, 600,600)
image.display(disp1);

print('applying local contrast normalization')
ht = hough.local_contrast_normalization(ht);
disp2 = ht:clone()/ht:max()
image.save(dname.."/disp_2_"..fname, disp2);
disp2 = image.scale(disp2, 600,600)
image.display(disp2);

print('restricting angles')
restA = 5.0
ht1 = hough.restrict_angles(ht, 0.0, math.pi*(restA)/180.0)
ht2 = hough.restrict_angles(ht, math.pi*(180.0-restA)/180.0, math.pi*(180.0+restA)/180.0)
ht3 = hough.restrict_angles(ht, math.pi*(360.0-restA)/180.0, 2.0*math.pi)
ht = ht1+ht2+ht2
disp3 = ht:clone()/ht:max()
disp3 = image.scale(disp3, 600,600)
image.display(disp3);

print('sorting lines')
sorted = hough.find_best_lines(ht,10)
smp = smp/(smp:max()+0.00000001)
for i=1,sorted:size(1) do
  local RR = sorted[i][1]
  local AA = sorted[i][2]
  local VV = sorted[i][3]
  printf('drawing %d th best line RR: %s, AA: %s, val: %s', i,RR/600,AA*180/(1080*math.pi),VV)
  --local dispi = smp:clone():repeatTensor(3,1,1)
  local dispi = img:clone()
  hough.draw_line(dispi, RR, AA, 1000, 1080)
  image.display(dispi);
end
