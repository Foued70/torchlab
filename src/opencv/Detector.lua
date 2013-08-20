ffi = require 'ffi'
libopencv = util.ffi.load("libopencv")
ctorch = util.ctorch
Mat = require './Mat'

ffi.cdef [[

// ------------
//   opaque pointer (not visible from Lua interface)
// ------------
typedef struct FeatureDetector  FeatureDetector;

// ------------
//   functions implemented in opencv.cpp
// ------------
FeatureDetector* FeatureDetector_create(const char* detector_type);

int FeatureDetector_detect(FeatureDetector* detector, const Mat* img, const Mat* mask, KeyPoint* kptr, int npts);
void FeatureDetector_parameters(FeatureDetector* detector);

void FeatureDetector_destroy(FeatureDetector* detector);
]]

Detector = Class()

Detector.types = require './types/Detector'

-- <input> String detectorType
function Detector:__init(detectorType)
   detectorType = detectorType or "FAST"
   if not Detector.types[detectorType] then
      error("Don't understand detectorType: "..detectorType) 
   else
      self.detector = ffi.gc(libopencv.FeatureDetector_create(ffi.string(detectorType)),
                             function (detector)
                                   libopencv.FeatureDetector_destroy(detector)
                             end)
   end
end

function Detector:parameters()
   if type(self.detector) ~= "cdata" then
      error("poorly initialized detector object")
   end
   libopencv.FeatureDetector_parameters(self.detector)
end

-- <input> detector, img, npts, Mat mask
-- <output> KeyPoint*, npts
function Detector:detect(img,npts,mask)
   if type(self.detector) ~= "cdata" then
      error("poorly initialized detector object")
   end
   mask = mask or Mat.new()
   npts = npts or 1000
   keypoints = ffi.new("KeyPoint[?]", npts)
   if ((not img.mat) or (type(img.mat) ~= "cdata")) then 
      error("problem with input image")
   end
   if ((not mask.mat) or (type(mask.mat) ~= "cdata")) then 
      error("problem with image mask")
   end
   npts = libopencv.FeatureDetector_detect(self.detector,img.mat,mask.mat,keypoints[0],npts)
   return keypoints, npts
end

return Detector
