opencv = require '../init'

-- adds some funcs to move between torch.Tensor userdata and cdata
ctorch = util.ctorch

-- low level from Tensor to Mat and back

t = torch.randn(3,5,5)
print(t)
m = opencv.C.THDoubleTensor_toMat(torch.cdata(t))
opencv.C.Mat_info(m)
t2 = torch.Tensor()
opencv.C.THDoubleTensor_fromMat(m, torch.cdata(t2))

t2[1][1][1] = 100
t2[2][2][2] = 100
t2[3][3][3] = 100

print(t2)
print(t)

-- from Mat to Tensor and back
img = opencv.C.Mat_loadImage("../image/test/lena.jpg")
timg = torch.ByteTensor()
opencv.C.THByteTensor_fromMat(img,torch.cdata(timg))
img2 = opencv.C.THByteTensor_toMat(torch.cdata(timg))

-- clone the data
cvtMat = opencv.C.Mat_clone(img2)

-- convert BGR (opencv) to RGB (torch)
opencv.C.Mat_convert(img2,cvtMat,opencv.Mat.conversion.BGR2RGB)
tcvt = torch.ByteTensor()
opencv.C.THByteTensor_fromMat(cvtMat,torch.cdata(tcvt))

print(" +++ orig")
opencv.C.Mat_info(img)

print(" +++ fromT")
opencv.C.Mat_info(img2)

print(" +++ clone")
opencv.C.Mat_info(cvtMat)

timg = timg:transpose(1,3):transpose(2,3)
image.display(timg)

tcvt = tcvt:transpose(1,3):transpose(2,3)
image.display(tcvt)

opencv.C.Mat_showImage(img2,"mat2torch2mat")
opencv.C.Mat_showImage(cvtMat,"cvtMat")

-- opencv conversions
