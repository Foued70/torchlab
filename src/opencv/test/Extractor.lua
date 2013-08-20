
opencv    = require '../init'

-- use graphics magick to load the image
img = image.load(CLOUDLAB_SRC.."/image/test/lena.jpg","byte",nil,"LAB","DHW")

-- convert 1st (L = luminosity) channel of torch tensor to opencv matrix
matMat = opencv.Mat.new(img[1])

detector    = opencv.Detector.new("ORB")

kpts, npts  = detector:detect(matMat,250)

extractor_type = "SIFT"
extractor   = opencv.Extractor.new(extractor_type)

descriptors = extractor:compute(matMat,kpts,npts)

_G.desc = descriptors:toTensor()

print(desc:size())


