
opencv = require './libopencv.lua'
-- t = torch.CharTensor(256,256)
img = image.load("DSC_0130.png","byte",nil,"LAB","DHW")

cvmat = opencv.fromTensor(img[1])
libopencv = util.ffi.load('libluaopencv')
detector_type = {
   "FAST",      -- FastFeatureDetector
   "STAR",      -- StarFeatureDetector
   "SIFT",      -- SIFT (nonfree module)
   "SURF",      -- SURF (nonfree module)
   "ORB",       -- ORB
   "BRISK",     -- BRISK
   "MSER",      -- MSER
   "GFTT",      -- GoodFeaturesToTrackDetector
   "HARRIS",    -- GoodFeaturesToTrackDetector with Harris detector enabled
   "Dense",     -- DenseFeatureDetector
   "SimpleBlob" -- SimpleBlobDetector
}

cvpoints = libopencv.detectfeatures(cvmat,"FAST")
print(cvpoints)
opencv.toTensor(cvpoints)

cvpoints = libopencv.detectfeatures(cvmat,"HARRIS")
cvpoints = libopencv.detectfeatures(cvmat,"GFTT")
