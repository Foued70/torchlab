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
img = image.rgb2lab(img);
smp = saliency.high_entropy_features(img[1],5,5)
print('getting hough transform')
ht = hough.get_hough_transform(smp, 2500,1080)
disp1 = ht:clone()/ht:max()
image.save(dname.."/disp_1_"..fname, disp1);
disp1 = image.scale(disp1, 300,600)
image.display(disp1);
print('applying local contrast normalization')
ht = hough.local_contrast_normalization(ht);
disp2 = ht:clone()/ht:max()
image.save(dname.."/disp_2_"..fname, disp2);
disp2 = image.scale(disp2, 300,600)
image.display(disp2);