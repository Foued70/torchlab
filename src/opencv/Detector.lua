ffi = require 'ffi'
libopencv = require './libopencv'
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
  void debug_keypoints(Mat* img, const KeyPoint* keyptr, int npts);

]]

Detector = {}

Detector.types = {
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

-- <input> Mat img, String detectorType , Mat mask
-- <output> KeyPoint*, npts
function Detector.create(detectorType)
   detectorType = detectorType or "ORB"
   return ffi.gc(libopencv.FeatureDetector_create(ffi.string(detectorType)),
                 function (detector)
                    libopencv.FeatureDetector_destroy(detector)
                 end)
end

-- <input> Mat img, String detectorType , Mat mask
-- <output> KeyPoint*, npts
function Detector.detect(detector,mat,npts,mask)
   if type(detector) ~= "cdata" then
      error("need to pass opencv detector object")
   end
   print('ok detector')
   mask = mask or libopencv.Mat_create(0,0,0)
   print('ok mask')
   npts = npts or 1000
   keypoints = ffi.new("KeyPoint[?]", npts)
   print('ok keypoints')
   npts = libopencv.FeatureDetector_detect(detector,mat,mask,keypoints[0],npts)
   print('npts found')
   return keypoints, npts, detector
end


return Detector
