opencv = require './libopencv.lua'
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

kpvect = libopencv.detect(cvmat,"FAST")
print(kpvect)
npts = kpvect.length

for i = 0,npts-1 do 
   cvp = kpvect.data[i]
   print(i,cvp.pt.x, cvp.pt.y, cvp.response)
end

-- opencv.toTensor(cvpoints)

-- cvpoints = libopencv.detectfeatures(cvmat,"HARRIS")
-- cvpoints = libopencv.detectfeatures(cvmat,"GFTT")
