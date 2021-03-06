ffi = require 'ffi'
opencv_ffi = require './opencv_ffi'

Detector = Class()

Detector.types = require './types/Detector'

-- <input> String detectorType
function Detector:__init(detectorType)
   detectorType = detectorType or "FAST"
   if not Detector.types[detectorType] then
      error("Don't understand detectorType: "..detectorType) 
   else
      self.detector = ffi.gc(opencv_ffi.FeatureDetector_create(ffi.string(detectorType)),
                             function (detector)
                                   opencv_ffi.FeatureDetector_destroy(detector)
                             end)
   end
end

function Detector:parameters()
   if type(self.detector) ~= "cdata" then
      error("poorly initialized detector object")
   end
   opencv_ffi.FeatureDetector_parameters(self.detector)
end

-- <input> detector, img, npts, Mat mask
-- <output> KeyPoint*, npts
function Detector:detect(img,npts,mask)
   if type(self.detector) ~= "cdata" then
      error("poorly initialized detector object")
   end
   mask = mask or opencv.Mat.new()
   npts = npts or 1000
   keypoints = ffi.new("KeyPoint[?]", npts)
   if ((not img.mat) or (type(img.mat) ~= "cdata")) then 
      error("problem with input image")
   end
   if ((not mask.mat) or (type(mask.mat) ~= "cdata")) then 
      error("problem with image mask")
   end
   npts = opencv_ffi.FeatureDetector_detect(self.detector,img.mat,mask.mat,keypoints[0],npts)
   return keypoints, npts
end
