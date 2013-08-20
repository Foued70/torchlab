opencv = require '../init'

-- adds some funcs to move between torch.Tensor userdata and cdata
ctorch = util.ctorch

-- low level from Tensor to Mat and back

t = torch.randn(3,5,5)
print(t)
m = opencv.Mat.new(t)
m:info()

t2 = m:toTensor()

t2[1][1][1] = 100
t2[2][2][2] = 100
t2[3][3][3] = 100

print(t2)
print(t)

-- from Mat to Tensor and back
img = opencv.Mat.new(CLOUDLAB_SRC.."/image/test/lena.jpg")
timg = img:toTensor()

img2 = opencv.Mat.new(timg)

-- convert (clone) BGR (opencv) to RGB (torch)
cvtMat = img2:convert(opencv.Mat.new(timg:clone()),"BGR2RGB")

-- copy to torch tensor Transposed to Depth x Height x Width
cvtTh  = cvtMat:toTensor("DHW")


print(" +++ orig")
img:info()

print(" +++ fromT")
img2:info()
img2:display("BGR")

img2:convert("BGR2RGB")
img2:display("RGB")

print(" +++ convert BGR")
cvtMat:info()

image.display(cvtTh)

