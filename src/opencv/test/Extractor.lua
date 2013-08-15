
opencv    = require '../init'

-- use graphics magick to load the image
img = image.load(CLOUDLAB_SRC.."/image/test/lena.jpg","byte",nil,"LAB","DHW")

-- convert 1st (L = luminosity) channel of torch tensor to opencv matrix
mat = opencv.Mat.fromTensor(img[1])

detector    = opencv.Detector.create("ORB")

kpts, npts  = opencv.Detector.detect(detector,mat,250)

extractor   = opencv.Extractor.create("SIFT")

descriptors = opencv.Extractor.compute(extractor,mat,kpts,npts)

_G.desc = opencv.Mat.toTensor(descriptors)

print(desc:size())


