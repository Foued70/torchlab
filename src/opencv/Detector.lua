ffi = require 'ffi'
libopencv = util.ffi.load("libopencv")
ctorch = util.ctorch

ffi.cdef [[

// ------------
//   opaque pointer (not visible from Lua interface)
// ------------
typedef struct FeatureDetector  FeatureDetector;

// ------------
//   transparent pointers (internals accessible from Lua)
// ------------
typedef struct CvPoint2D32f
{
    float x;
    float y;
}
CvPoint2D32f;

// from opencv2/features2d/features2d.hpp
typedef struct KeyPoint
{
    CvPoint2D32f pt; //!< coordinates of the keypoints
    float size;      //!< diameter of the meaningful keypoint neighborhood

    float angle;     //!< computed orientation of the keypoint (-1 if not
                     //   applicable); it's in [0,360) degrees and
                     //   measured relative to image coordinate system,
                     //   ie in clockwise.

    float response;  //!< the response by which the most strong
                     //   keypoints have been selected. Can be used for the
                     //   further sorting or subsampling

    int octave;      //!< octave (pyramid layer) from which the keypoint
                     //   has been extracted

    int class_id;    //!< object class (if the keypoints need to be
                     //   clustered by an object they belong to)

} KeyPoint ;


// ------------
//   functions implemented in opencv.cpp
// ------------
FeatureDetector* FeatureDetector_create(const char* detector_type);
int FeatureDetector_detect(FeatureDetector* detector, const Mat* img, const Mat* mask, KeyPoint* kptr, int npts);
void FeatureDetector_destroy(FeatureDetector* detector);
void dump_keypoints(const KeyPoint* keyptr, int npts);
void draw_keypoints(Mat* img, const KeyPoint* keyptr, int npts);
Mat* detectCornerHarris(Mat* src, int blockSize, int ksize, int k);
Mat* getMatFromKeypoints(const KeyPoint* keyptr, int npts);
]]

Detector = {}

Detector.types = require './types/Detector'

-- <input> String detectorType
function Detector.create(detectorType)
   detectorType = detectorType or "ORB"
   return ffi.gc(libopencv.FeatureDetector_create(ffi.string(detectorType)),
                 function (detector)
                    libopencv.FeatureDetector_destroy(detector)
                 end)
end

-- <input> detector, img, npts, Mat mask
-- <output> KeyPoint*, npts
function Detector.detect(detector,mat,npts,mask)
   if type(detector) ~= "cdata" then
      error("need to pass opencv detector object")
   end
   mask = mask or libopencv.Mat_create(0,0,0)
   npts = npts or 1000
   keypoints = ffi.new("KeyPoint[?]", npts)
   npts = libopencv.FeatureDetector_detect(detector,mat,mask,keypoints[0],npts)
   return keypoints, npts
end

-- <input> img, npts, Mat mask
-- <output> KeyPoint*, npts
function Detector.detect(detector,mat,npts,mask)
   if type(detector) ~= "cdata" then
      error("need to pass opencv detector object")
   end
   mask = mask or libopencv.Mat_create(0,0,0)
   npts = npts or 1000
   keypoints = ffi.new("KeyPoint[?]", npts)
   npts = libopencv.FeatureDetector_detect(detector,mat,mask,keypoints[0],npts)
   return keypoints, npts
end

-- <input> img, blockSize, ksize, k
-- <output> Mat of responses at every location
function Detector.detectCornerHarris(mat,blockSize,ksize, k)
   if type(mat) ~= "cdata" then
      error("need to pass opencv mat as first arg")
   end
    if type(blockSize) ~= "number" then
      error("need to pass number as second arg")
   end
  if type(ksize) ~= "number" then
      error("need to pass number as third arg")
   end
    if type(k) ~= "number" then
      error("need to pass number as fourth arg")
   end
   return Mat.new(ffi.gc(libopencv.detectCornerHarris(mat, blockSize, ksize, k), Mat.destructor()))
end

-- <input> img, blockSize, ksize, k
-- <output> Mat of responses at every location
function Detector.getMatFromKeypoints(keyptr, npts)
   if type(keyptr) ~= "cdata" then
      error("need to pass opencv mat as first arg")
   end
    if type(npts) ~= "number" then
      error("need to pass number as second arg")
   end
   return Mat.new(ffi.gc(libopencv.getMatFromKeypoints(keyptr, npts), Mat.destructor()))
end

return Detector
