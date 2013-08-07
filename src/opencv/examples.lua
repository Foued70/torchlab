libopencv = require './libopencv'

-- adds some funcs to move between torch.Tensor userdata and cdata
ctorch = util.ctorch

-- from Tensor to Mat and back

t = torch.randn(3,5,5)
print(t)
m = libopencv.C.THDoubleTensor_toMat(torch.cdata(t))
libopencv.C.Mat_info(m)
t2 = torch.Tensor()
libopencv.C.THDoubleTensor_fromMat(m, torch.cdata(t2))

t2[1][1][1] = 100
t2[2][2][2] = 100
t2[3][3][3] = 100

print(t2)
print(t)

-- from Mat to Tensor and back
img = libopencv.C.Mat_loadImage("DSC_0130.png")
timg = torch.ByteTensor()
libopencv.C.THByteTensor_fromMat(img,torch.cdata(timg))
img2 = libopencv.C.THByteTensor_toMat(torch.cdata(timg))

timg = timg:transpose(1,3):transpose(2,3)
print(timg:size())
image.display(timg)

-- opencv conversions
