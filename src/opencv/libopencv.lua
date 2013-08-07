-- This file contains two parts.  First the ffi interface to opencv
-- and our C wrappers for opencv. Then the lua wrappers.

-- The lua wrappers to C functions which call the opencv C++
-- functions. The ffi interface is in libopencv.lua and the c wrapper
-- functions are written in luaopencv.cpp.

-- All functions in the opencv package take pointers to opencv objects
-- as input.  There are functions to convert to and from torch.Tensors
-- which don't copy data when then can.

-- TODO figure out how to put this in another file. To make code more readable.

-- FFI bindings to Opencv:
local ffi = require "ffi"
-- local bit = require 'bit'
local ctorch = util.ctorch

-- data types CV_<bit_depth>{U|S|F}C{num_channels}

-- copy needed structs from opencv2/core/types_c.h
-- these will create the interface between torch and opencv

ffi.cdef
[[

typedef unsigned char uchar;

typedef struct Mat Mat;

typedef struct CvPoint
{
    int x;
    int y;
}
CvPoint;

typedef struct CvPoint2D32f
{
    float x;
    float y;
}
CvPoint2D32f;

typedef struct CvPoint3D32f
{
    float x;
    float y;
    float z;
}
CvPoint3D32f;

typedef struct CvRect
{
    int x;
    int y;
    int width;
    int height;
} CvRect;

typedef struct CvSize
{
  int width;
  int height;
} CvSize;

// from opencv2/features2d/features2d.hpp
typedef struct KeyPoint
{
    CvPoint2D32f pt; //!< coordinates of the keypoints
    float size;      //!< diameter of the meaningful keypoint neighborhood
    float angle;     //!< computed orientation of the keypoint (-1 if not applicable);
                     //!< it's in [0,360) degrees and measured relative to
                     //!< image coordinate system, ie in clockwise.
    float response;  //!< the response by which the most strong keypoints have been selected. Can be used for the further sorting or subsampling
    int   octave;    //!< octave (pyramid layer) from which the keypoint has been extracted
    int   class_id;  //!< object class (if the keypoints need to be clustered by an object they belong to)
} KeyPoint ;


]]

-- This is a list of wrapper functions encapsulating the opencv
-- functions we want.  The wrapper functions are defined in opencv.cpp
-- in an extern "C" block to make the C++ callable from lua C.) These
-- functions call the needed C++ operators. They accept and return the
-- opencv C structs defined above.
ffi.cdef [[
Mat* Mat_create (int width, int height, int type);

int  Mat_depth(Mat* mat);

void Mat_size(Mat* mat, THLongStorage* size);
void Mat_stride(Mat* mat, THLongStorage* stride);

Mat* Mat_loadImage(const char* fname);
void Mat_showImage(Mat* mat, const char* wname);
void Mat_convert(Mat* input, Mat* output, int cvttype);
void Mat_info(Mat* mat);
void Mat_destroy(Mat* mat);
]]

ctorch.generateTypes [[
Mat* THTensor_toMat(THTensor* tensor);
void THTensor_fromMat(Mat* mat,THTensor* tensor);
]]

ffi.cdef [[
int detect(const Mat* img, const char* detector_type, const Mat* mask, KeyPoint* kpC, int npts);
void debug_keypoints(Mat* img, const KeyPoint* kptr, int npts);
]]

-- below starts our lua wrappers
libopencv = {}
-- load our C wrappers. TODO expose C or not...
libopencv.C = util.ffi.load('libcopencv')

-- copy large enums for opencv
libopencv.Mat_types      = require './mat_types'
libopencv.cvtColor_types = require './cvt_types'

libopencv.detector_type = {
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
function libopencv.detect(mat,detectorType,npts,mask)
   detectorType = detectorType or "FAST"
   mask = mask or libopencv.C.Mat_create(0,0,0)
   npts = npts or 1000
   keypoints = ffi.new("KeyPoint[?]", npts)
   npts = libopencv.C.detect(mat,detectorType,mask,keypoints[0],npts)
   return keypoints, npts
end

function libopencv.debug_keypoints(mat,kpts,npts)
   libopencv.C.debug_keypoints(mat,kpts,npts)
end

return libopencv
