
opencv    = require '../init'

-- use graphics magick to load the image
img = image.load(CLOUDLAB_SRC.."/image/test/lena.jpg","byte",nil,"LAB","DHW")

-- convert 1st (L = luminosity) channel of torch tensor to opencv matrix
matMat = opencv.Mat.new(img[1])

detector    = opencv.Detector.create("ORB")

kpts, npts  = opencv.Detector.detect(detector,matMat.mat,250)

extractor_type = opencv.Extractor.types[1] --sift
extractor   = opencv.Extractor.create(extractor_type)

descriptors = opencv.Extractor.compute(extractor,matMat.mat,kpts,npts)

_G.desc = opencv.Mat.new(descriptors):toTensor()

print(desc:size())


