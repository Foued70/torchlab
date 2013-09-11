opencv_ffi = require "../opencv_ffi"

-- low level from Tensor to Mat and back

t = torch.randn(3,5,5)
print(t)
m = opencv_ffi.THDoubleTensor_toMat(torch.cdata(t))
opencv_ffi.Mat_info(m)
t2 = torch.Tensor()
opencv_ffi.THDoubleTensor_fromMat(m, torch.cdata(t2))

t2[1][1][1] = 100
t2[2][2][2] = 100
t2[3][3][3] = 100

print(t2)
print(t)

-- from Mat to Tensor and back
img = opencv_ffi.Mat_loadImage(CLOUDLAB_SRC.."/image/test/lena.jpg")
timg = torch.ByteTensor()
opencv_ffi.THByteTensor_fromMat(img,torch.cdata(timg))
img2 = opencv_ffi.THByteTensor_toMat(torch.cdata(timg))

-- clone the data
cvtMat = opencv_ffi.Mat_clone(img2)

-- convert BGR (opencv) to RGB (torch)
opencv_ffi.Mat_convert(img2,cvtMat,opencv.Mat.conversion.BGR2RGB)
tcvt = torch.ByteTensor()
opencv_ffi.THByteTensor_fromMat(cvtMat,torch.cdata(tcvt))

print(" +++ orig")
opencv_ffi.Mat_info(img)

print(" +++ fromT")
opencv_ffi.Mat_info(img2)

print(" +++ clone")
opencv_ffi.Mat_info(cvtMat)

timg = timg:transpose(1,3):transpose(2,3)
image.display(timg)

tcvt = tcvt:transpose(1,3):transpose(2,3)
image.display(tcvt)

opencv_ffi.Mat_showImage(img2,"mat2torch2mat")
opencv_ffi.Mat_showImage(cvtMat,"cvtMat")

-- opencv conversions
