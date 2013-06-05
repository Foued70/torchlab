hough = require 'hough'
require 'image'
img = image.load('/Users/lihui815/Projects/PICSDIR/SWEEPS/jet-pizza-6780-small/DSC_0012_smp.jpg')[1]:type('torch.DoubleTensor')
print('getting hough transform')
ht = hough.get_hough_transform(img, 480,480)
image.display(ht:clone());
print('applying local contrast normalization')
ht = hough.local_contrast_normalization(ht);
image.display(ht);