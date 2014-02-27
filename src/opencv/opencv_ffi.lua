ctorch = util.ctorch

ffi  = require 'ffi'
-- This is a list of wrapper functions encapsulating the opencv
-- functions we want.  The wrapper functions are defined in Mat.cpp
-- in an extern "C" block to make the C++ callable from lua C.) These
-- functions call the needed C++ operators. They accept and return the
-- opencv C structs defined above.

-- Mat.lua
ffi.cdef [[
// ------------
//   opaque pointer (not visible from Lua interface)
// ------------
typedef struct Mat Mat;

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
Mat* Mat_create (int width, int height, int type);
Mat* Mat_clone (Mat* mat);
Mat* getMatFromKeypoints(const KeyPoint* keyptr, int npts);

int  Mat_depth  (Mat* mat);
void Mat_size   (Mat* mat, THLongStorage* size);
void Mat_stride (Mat* mat, THLongStorage* stride);

void Mat_convert (Mat* input, Mat* output, int cvttype);

Mat* Mat_loadImage (const char* fname);
void Mat_showImage (Mat* mat, const char* wname);

void Mat_info      (Mat* mat);
int Mat_type(Mat* mat);
void Mat_destroy   (Mat* mat);
void KeyPoint_destroy(KeyPoint * kpts);


]]

-- ------------
--   generic functions implemented in generic/opencv.cpp
-- ------------

ctorch.generateTypes [[
Mat* THTensor_toMat(THTensor* tensor);
void THTensor_fromMat(Mat* mat,THTensor* tensor);
KeyPoint* THTensor_toKeypoints (THTensor* tensor);
]]

-- Detector.lua
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

-- Extractor.lua
ffi.cdef [[
// ------------
//   opaque pointer (not visible from Lua interface)
// ------------
typedef struct DescriptorExtractor DescriptorExtractor;

DescriptorExtractor* DescriptorExtractor_create(const char* feature_type);
void DescriptorExtractor_compute(DescriptorExtractor* extractor,
                                           const Mat* img,
                                                 Mat* descriptor,
                                            KeyPoint* keypointsC, 
                                                  int npts);
void DescriptorExtractor_destroy(DescriptorExtractor* extractor);

]]

-- Matcher.lua
ffi.cdef [[
// ------------
//   opaque pointer (not visible from Lua interface)
// ------------
typedef struct DescriptorMatcher DescriptorMatcher;

// from opencv2/features2d/features2d.hpp
typedef struct DMatch
{
  int queryIdx; // query descriptor index
  int trainIdx; // train descriptor index
  int imgIdx;   // train image index

  float distance;
} DMatch ;

DescriptorMatcher* DescriptorMatcher_create(const char* feature_type);
int DescriptorMatcher_match(DescriptorMatcher* matcher, const Mat*  descriptors_src, const Mat*  descriptors_dest, DMatch* matchesC, int npts);
void DescriptorMatcher_destroy(DescriptorMatcher* matcher);
int DescriptorMatcher_reduceMatches(DMatch* matchesptr, int nmatches, DMatch* matchesReducedC);
]]

-- calib3d.lua
ffi.cdef [[
Mat* getHomography(const KeyPoint* keyptr_src,  int npts_src, 
                   const KeyPoint* keyptr_dest, int npts_dest, 
                     const DMatch* matchptr,    int npts_match);
]] 

-- imgproc.lua
ffi.cdef [[
Mat* warpImage(const Mat* src, const Mat* transform, int size_x, int size_y);
Mat* computeConvexHull(const Mat* points, bool clockwise);
Mat* CannyDetectEdges(Mat* src, double threshold1, double threshold2);
Mat* HoughLinesRegular(Mat* image, double rho, double theta, int threshold, double srn, double stn);
Mat* HoughLinesProbabilistic(Mat* image, double rho, double theta, int threshold, double minLineLength, double maxLineGap);

Mat* getStructuringElement(int type, int size_x, int size_y, int center_x, int center_y);
void dilate(Mat*  src, Mat* structuringElement);
void erode(Mat*  src, Mat* structuringElement);

Mat* detectCornerHarris(Mat* src, int blockSize, int ksize, int k);
Mat* getPairwiseDistances(const Mat* A, const Mat* B);
Mat* findBestTransformation(const Mat* goodLocationsX_src, const Mat* goodLocationsY_src, const Mat* scores_src,  const Mat* pairwise_dis_src,
  const Mat* goodLocationsX_dest, const Mat* goodLocationsY_dest, const Mat* scores_dest, const Mat* pairwise_dis_dest, 
    const Mat* angle_diff,
  double corr_thresh, int minInliers, int numInliersMax, double cornerComparisonThreshold, double minx, double maxx, double miny, double maxy);
void flood_fill(Mat* img, Mat* result, int x, int y);

int find_contours(Mat* image, THDoubleTensor* th_contours, THDoubleTensor* th_segment_inds );

void distance_transform(Mat* img, Mat* result);
void distance_transform_labels(Mat* img, Mat* result, Mat* labels);
void fillQuad(Mat* img, double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4);
void fillQuadAll(Mat* img, Mat* quad);
void fillQuadAllWithInterpolation(Mat* img, Mat* resultD, Mat* quad);
void resize(Mat* src, Mat* dst, double factor);
void threshold(Mat* src,  Mat*dst);
Mat* rotateImage(const Mat* src, Mat* dst, double angle, double centerC, double centerR, int size_1, int size_2);
]]

-- utils.lua
ffi.cdef [[
void dump_keypoints(const KeyPoint* keyptr, int npts);
void draw_keypoints(Mat* img, const KeyPoint* keyptr, int npts);
]]

return util.ffi.load("libopencv_ffi")
