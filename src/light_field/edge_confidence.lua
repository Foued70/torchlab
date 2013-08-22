require 'image'
require 'sys'

-- pretty much saliency in image space to weight the epi lines
-- TODO just try saliency
i1        = image.load("mansion_image_0000.jpg")
i2        = image.load("mansion_image_0001.jpg")
win_size  = 32
height    = i1:size(2)
width     = i1:size(3)
dimg      = torch.Tensor(win_size,height,width+win_size)
win_width = width - win_size
tmp_win   = torch.Tensor(i1:size(1),height,width+win_size)

sys.tic()
for i = 1,win_size do 
   invi = win_size - i + 1
   w1 = i1:narrow(3,   i,win_width)
   w2 = i2:narrow(3,invi,win_width)
   wd = dimg[{i,{},{i,i+win_width-1}}]
   torch.add(tmp_win,w1,-1,w2) 
   wd:copy(tmp_win:abs():sum(1):squeeze())
   collectgarbage()
   print(i,invi, sys.toc())
   sys.tic()
end

val,idx = dimg:min(1)
-- Edge confidence high confidence (val less than mean)
conf = dimg:mean(1) - val
