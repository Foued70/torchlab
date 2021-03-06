-- use graphics magick to load the image
img_src  = image.load(CLOUDLAB_SRC.."/image/test/baboon200.jpg","byte",nil,"LAB","DHW")
img_dest = image.load(CLOUDLAB_SRC.."/image/test/baboon200_rotated.jpg","byte",nil,"LAB","DHW")

-- convert 1st (L = luminosity) channel of torch tensor to opencv matrix
matMat_src  = opencv.Mat.new(img_src[1])
matMat_dest = opencv.Mat.new(img_dest[1])

detectorType = "SIFT"
detector     = opencv.Detector.new(detectorType)

kpts_src, npts_src    = detector:detect(matMat_src,250)
kpts_dest, npts_dest  = detector:detect(matMat_dest,250)

opencv.utils.dump_keypoints(kpts_src,npts_src)
--opencv.C.draw_keypoints(matMat_src.mat,kpts_src,npts_src)
--opencv.C.draw_keypoints(matMat_dest.mat,kpts_dest,npts_dest)

extractor_type = "SIFT"
extractor      = opencv.Extractor.new(extractor_type)

descriptors_src  = extractor:compute(matMat_src,kpts_src,npts_src)
descriptors_dest = extractor:compute(matMat_dest,kpts_dest,npts_dest)

matcher_type = "FlannBased"
matcher      = opencv.Matcher.new(matcher_type)
sizeSrc      = descriptors_src:size(1)
sizeDest     = descriptors_dest:size(1)

matches,nmatches = matcher:match(descriptors_src, descriptors_dest, sizeSrc*sizeDest)

matches_good,nmatches_good = matcher:reduce(matches, nmatches)

H = opencv.calib3d.getHomography(kpts_src, npts_src, kpts_dest, npts_dest, matches, nmatches)

warped = opencv.imgproc.warpImage(matMat_src, H, descriptors_dest:size(1), descriptors_dest:size(2))

image.display(matMat_src:toTensor())
image.display(matMat_dest:toTensor())
image.display(warped:toTensor())

