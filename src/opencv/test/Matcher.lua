
opencv    = require '../init'

-- use graphics magick to load the image
img_src = image.load(CLOUDLAB_SRC.."/image/test/baboon200.jpg","byte",nil,"LAB","DHW")
img_dest = image.load(CLOUDLAB_SRC.."/image/test/baboon200_rotated.jpg","byte",nil,"LAB","DHW")

-- convert 1st (L = luminosity) channel of torch tensor to opencv matrix
matMat_src = opencv.Mat.new(img_src[1])
matMat_dest = opencv.Mat.new(img_dest[1])

detectorType = "FAST"
detector    = opencv.Detector.create(detectorType)

kpts_src, npts_src  = opencv.Detector.detect(detector,matMat_src.mat,250)
kpts_dest, npts_dest  = opencv.Detector.detect(detector,matMat_dest.mat,250)
opencv.C.dump_keypoints(kpts_src,npts_src)
opencv.C.draw_keypoints(matMat_src.mat,kpts_src,npts_src)
opencv.C.draw_keypoints(matMat_dest.mat,kpts_dest,npts_dest)

extractor_type = "SIFT"
extractor   = opencv.Extractor.create(extractor_type)


descriptors_src = opencv.Extractor.compute(extractor,matMat_src.mat,kpts_src,npts_src)
descriptors_dest = opencv.Extractor.compute(extractor,matMat_dest.mat,kpts_dest,npts_dest)

descriptors_src_Mat =opencv.Mat.new(descriptors_src)
descriptors_dest_Mat =opencv.Mat.new(descriptors_dest)

matcher_type = "FlannBased"
matcher   = opencv.Matcher.create(matcher_type)
sizeSrc = descriptors_src_Mat:size()[1]
sizeDest = descriptors_dest_Mat:size()[1]

matches,nmatches = opencv.Matcher.match(matcher, descriptors_src_Mat.mat, descriptors_dest_Mat.mat, sizeSrc*sizeDest)

matches_good,nmatches_good = opencv.Matcher.reduce(matches, nmatches)


H = opencv.ImgProc.getHomography(kpts_src, npts_src, kpts_dest, npts_dest, matches_good, nmatches_good)

warped = opencv.ImgProc.warpImage(matMat_src.mat, H.mat)

opencv.ImgProc.displayGrayMatInLua(warped.mat)
opencv.ImgProc.displayGrayMatInLua(matMat_src.mat)
opencv.ImgProc.displayGrayMatInLua(matMat_dest.mat)