libopencv = require "../libopencv"

-- low level from Tensor to Mat and back

t = torch.randn(3,5,5)
print(t)
m = libopencv.THDoubleTensor_toMat(torch.cdata(t))
libopencv.Mat_info(m)
t2 = torch.Tensor()
libopencv.THDoubleTensor_fromMat(m, torch.cdata(t2))

t2[1][1][1] = 100
t2[2][2][2] = 100
t2[3][3][3] = 100

print(t2)
print(t)

-- from Mat to Tensor and back
img = libopencv.Mat_loadImage(CLOUDLAB_SRC.."/image/test/lena.jpg")
timg = torch.ByteTensor()
libopencv.THByteTensor_fromMat(img,torch.cdata(timg))
img2 = libopencv.THByteTensor_toMat(torch.cdata(timg))

-- clone the data
cvtMat = libopencv.Mat_clone(img2)

-- convert BGR (opencv) to RGB (torch)
libopencv.Mat_convert(img2,cvtMat,opencv.Mat.conversion.BGR2RGB)
tcvt = torch.ByteTensor()
libopencv.THByteTensor_fromMat(cvtMat,torch.cdata(tcvt))

print(" +++ orig")
libopencv.Mat_info(img)

print(" +++ fromT")
libopencv.Mat_info(img2)

print(" +++ clone")
libopencv.Mat_info(cvtMat)

timg = timg:transpose(1,3):transpose(2,3)
image.display(timg)

tcvt = tcvt:transpose(1,3):transpose(2,3)
image.display(tcvt)

libopencv.Mat_showImage(img2,"mat2torch2mat")
libopencv.Mat_showImage(cvtMat,"cvtMat")

-- opencv conversions
