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
local bit = require 'bit'
local ctorch = util.ctorch

-- data types CV_<bit_depth>{U|S|F}C{num_channels}

-- copy needed structs from opencv2/core/types_c.h
-- these will create the interface between torch and opencv

ffi.cdef
[[

typedef unsigned char uchar;

typedef struct Mat
{
    /*! includes several bit-fields:
         - the magic signature
         - continuity flag
         - depth
         - number of channels
     */
    int flags;
    //! the array dimensionality, >= 2
    int dims;

    //! the number of rows and columns or
    //! (-1, -1) when the array has more than 2 dimensions
    int rows, cols;
    //! pointer to the data
    uchar* data;

    //! pointer to the reference counter;
    // when array points to user-allocated data, the pointer is NULL
    int* refcount;

} Mat;

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


types = {}

-- expanded macros from opencv2/core/types_c.h
types.CV_8U  = {}
types.CV_8S  = {}
types.CV_16U = {}
types.CV_16S = {}
types.CV_32S = {}
types.CV_32F = {}
types.CV_64F = {}
types.CV_USRTYPE1 = 7
types.CV_8U[1] = 0
types.CV_8U[2] = 8
types.CV_8U[3] = 16
types.CV_8U[4] = 24
types.CV_8S[1] = 1
types.CV_8S[2] = 9
types.CV_8S[3] = 17
types.CV_8S[4] = 25
types.CV_16U[1] = 2
types.CV_16U[2] = 10
types.CV_16U[3] = 18
types.CV_16U[4] = 26
types.CV_16S[1] = 3
types.CV_16S[2] = 11
types.CV_16S[3] = 19
types.CV_16S[4] = 27
types.CV_32S[1] = 4
types.CV_32S[2] = 12
types.CV_32S[3] = 20
types.CV_32S[4] = 28
types.CV_32F[1] = 5
types.CV_32F[2] = 13
types.CV_32F[3] = 21
types.CV_32F[4] = 29
types.CV_64F[1] = 6
types.CV_64F[2] = 14
types.CV_64F[3] = 22
types.CV_64F[4] = 30

libopencv.Mat_types = types

-- expanded from opencv2/imgproc/types_c.h
cvtColor = {}
cvtColor.BGR2BGRA    = 0
cvtColor.RGB2RGBA    = cvtColor.BGR2BGRA

cvtColor.BGRA2BGR    = 1
cvtColor.RGBA2RGB    = cvtColor.BGRA2BGR

cvtColor.BGR2RGBA    = 2
cvtColor.RGB2BGRA    = cvtColor.BGR2RGBA

cvtColor.RGBA2BGR    = 3
cvtColor.BGRA2RGB    = cvtColor.RGBA2BGR

cvtColor.BGR2RGB     = 4
cvtColor.RGB2BGR     = cvtColor.BGR2RGB

cvtColor.BGRA2RGBA   = 5
cvtColor.RGBA2BGRA   = cvtColor.BGRA2RGBA

cvtColor.BGR2GRAY    = 6
cvtColor.RGB2GRAY    = 7
cvtColor.GRAY2BGR    = 8
cvtColor.GRAY2RGB    = cvtColor.GRAY2BGR
cvtColor.GRAY2BGRA   = 9
cvtColor.GRAY2RGBA   = cvtColor.GRAY2BGRA
cvtColor.BGRA2GRAY   = 10
cvtColor.RGBA2GRAY   = 11

cvtColor.BGR2BGR565  = 12
cvtColor.RGB2BGR565  = 13
cvtColor.BGR5652BGR  = 14
cvtColor.BGR5652RGB  = 15
cvtColor.BGRA2BGR565 = 16
cvtColor.RGBA2BGR565 = 17
cvtColor.BGR5652BGRA = 18
cvtColor.BGR5652RGBA = 19

cvtColor.GRAY2BGR565 = 20
cvtColor.BGR5652GRAY = 21

cvtColor.BGR2BGR555  = 22
cvtColor.RGB2BGR555  = 23
cvtColor.BGR5552BGR  = 24
cvtColor.BGR5552RGB  = 25
cvtColor.BGRA2BGR555 = 26
cvtColor.RGBA2BGR555 = 27
cvtColor.BGR5552BGRA = 28
cvtColor.BGR5552RGBA = 29

cvtColor.GRAY2BGR555 = 30
cvtColor.BGR5552GRAY = 31

cvtColor.BGR2XYZ     = 32
cvtColor.RGB2XYZ     = 33
cvtColor.XYZ2BGR     = 34
cvtColor.XYZ2RGB     = 35

cvtColor.BGR2YCrCb   = 36
cvtColor.RGB2YCrCb   = 37
cvtColor.YCrCb2BGR   = 38
cvtColor.YCrCb2RGB   = 39

cvtColor.BGR2HSV     = 40
cvtColor.RGB2HSV     = 41

cvtColor.BGR2Lab     = 44
cvtColor.RGB2Lab     = 45

cvtColor.BayerBG2BGR = 46
cvtColor.BayerGB2BGR = 47
cvtColor.BayerRG2BGR = 48
cvtColor.BayerGR2BGR = 49

cvtColor.BayerBG2RGB = cvtColor.BayerRG2BGR
cvtColor.BayerGB2RGB = cvtColor.BayerGR2BGR
cvtColor.BayerRG2RGB = cvtColor.BayerBG2BGR
cvtColor.BayerGR2RGB = cvtColor.BayerGB2BGR

cvtColor.BGR2Luv     = 50
cvtColor.RGB2Luv     = 51
cvtColor.BGR2HLS     = 52
cvtColor.RGB2HLS     = 53

cvtColor.HSV2BGR     = 54
cvtColor.HSV2RGB     = 55

cvtColor.Lab2BGR     = 56
cvtColor.Lab2RGB     = 57
cvtColor.Luv2BGR     = 58
cvtColor.Luv2RGB     = 59
cvtColor.HLS2BGR     = 60
cvtColor.HLS2RGB     = 61

cvtColor.BayerBG2BGR_VNG = 62
cvtColor.BayerGB2BGR_VNG = 63
cvtColor.BayerRG2BGR_VNG = 64
cvtColor.BayerGR2BGR_VNG = 65

cvtColor.BayerBG2RGB_VNG = cvtColor.BayerRG2BGR_VNG
cvtColor.BayerGB2RGB_VNG = cvtColor.BayerGR2BGR_VNG
cvtColor.BayerRG2RGB_VNG = cvtColor.BayerBG2BGR_VNG
cvtColor.BayerGR2RGB_VNG = cvtColor.BayerGB2BGR_VNG

cvtColor.BGR2HSV_FULL = 66
cvtColor.RGB2HSV_FULL = 67
cvtColor.BGR2HLS_FULL = 68
cvtColor.RGB2HLS_FULL = 69

cvtColor.HSV2BGR_FULL = 70
cvtColor.HSV2RGB_FULL = 71
cvtColor.HLS2BGR_FULL = 72
cvtColor.HLS2RGB_FULL = 73

cvtColor.LBGR2Lab     = 74
cvtColor.LRGB2Lab     = 75
cvtColor.LBGR2Luv     = 76
cvtColor.LRGB2Luv     = 77

cvtColor.Lab2LBGR     = 78
cvtColor.Lab2LRGB     = 79
cvtColor.Luv2LBGR     = 80
cvtColor.Luv2LRGB     = 81

cvtColor.BGR2YUV      = 82
cvtColor.RGB2YUV      = 83
cvtColor.YUV2BGR      = 84
cvtColor.YUV2RGB      = 85

cvtColor.BayerBG2GRAY = 86
cvtColor.BayerGB2GRAY = 87
cvtColor.BayerRG2GRAY = 88
cvtColor.BayerGR2GRAY = 89

-- YUV 4:2:0 formats family
cvtColor.YUV2RGB_NV12 = 90
cvtColor.YUV2BGR_NV12 = 91
cvtColor.YUV2RGB_NV21 = 92
cvtColor.YUV2BGR_NV21 = 93
cvtColor.YUV420sp2RGB = cvtColor.YUV2RGB_NV21
cvtColor.YUV420sp2BGR = cvtColor.YUV2BGR_NV21

cvtColor.YUV2RGBA_NV12 = 94
cvtColor.YUV2BGRA_NV12 = 95
cvtColor.YUV2RGBA_NV21 = 96
cvtColor.YUV2BGRA_NV21 = 97
cvtColor.YUV420sp2RGBA = cvtColor.YUV2RGBA_NV21
cvtColor.YUV420sp2BGRA = cvtColor.YUV2BGRA_NV21

cvtColor.YUV2RGB_YV12 = 98
cvtColor.YUV2BGR_YV12 = 99
cvtColor.YUV2RGB_IYUV = 100
cvtColor.YUV2BGR_IYUV = 101
cvtColor.YUV2RGB_I420 = cvtColor.YUV2RGB_IYUV
cvtColor.YUV2BGR_I420 = cvtColor.YUV2BGR_IYUV
cvtColor.YUV420p2RGB = cvtColor.YUV2RGB_YV12
cvtColor.YUV420p2BGR = cvtColor.YUV2BGR_YV12

cvtColor.YUV2RGBA_YV12 = 102
cvtColor.YUV2BGRA_YV12 = 103
cvtColor.YUV2RGBA_IYUV = 104
cvtColor.YUV2BGRA_IYUV = 105
cvtColor.YUV2RGBA_I420 = cvtColor.YUV2RGBA_IYUV
cvtColor.YUV2BGRA_I420 = cvtColor.YUV2BGRA_IYUV
cvtColor.YUV420p2RGBA = cvtColor.YUV2RGBA_YV12
cvtColor.YUV420p2BGRA = cvtColor.YUV2BGRA_YV12

cvtColor.YUV2GRAY_420 = 106
cvtColor.YUV2GRAY_NV21 = cvtColor.YUV2GRAY_420
cvtColor.YUV2GRAY_NV12 = cvtColor.YUV2GRAY_420
cvtColor.YUV2GRAY_YV12 = cvtColor.YUV2GRAY_420
cvtColor.YUV2GRAY_IYUV = cvtColor.YUV2GRAY_420
cvtColor.YUV2GRAY_I420 = cvtColor.YUV2GRAY_420
cvtColor.YUV420sp2GRAY = cvtColor.YUV2GRAY_420
cvtColor.YUV420p2GRAY = cvtColor.YUV2GRAY_420

    -- YUV 4:2:2 formats family
cvtColor.YUV2RGB_UYVY = 107
cvtColor.YUV2BGR_UYVY = 108
    -- cvtColor.YUV2RGB_VYUY = 109
    -- cvtColor.YUV2BGR_VYUY = 110
cvtColor.YUV2RGB_Y422 = cvtColor.YUV2RGB_UYVY
cvtColor.YUV2BGR_Y422 = cvtColor.YUV2BGR_UYVY
cvtColor.YUV2RGB_UYNV = cvtColor.YUV2RGB_UYVY
cvtColor.YUV2BGR_UYNV = cvtColor.YUV2BGR_UYVY

cvtColor.YUV2RGBA_UYVY = 111
cvtColor.YUV2BGRA_UYVY = 112
    -- cvtColor.YUV2RGBA_VYUY = 113
    -- cvtColor.YUV2BGRA_VYUY = 114
cvtColor.YUV2RGBA_Y422 = cvtColor.YUV2RGBA_UYVY
cvtColor.YUV2BGRA_Y422 = cvtColor.YUV2BGRA_UYVY
cvtColor.YUV2RGBA_UYNV = cvtColor.YUV2RGBA_UYVY
cvtColor.YUV2BGRA_UYNV = cvtColor.YUV2BGRA_UYVY

cvtColor.YUV2RGB_YUY2 = 115
cvtColor.YUV2BGR_YUY2 = 116
cvtColor.YUV2RGB_YVYU = 117
cvtColor.YUV2BGR_YVYU = 118
cvtColor.YUV2RGB_YUYV = cvtColor.YUV2RGB_YUY2
cvtColor.YUV2BGR_YUYV = cvtColor.YUV2BGR_YUY2
cvtColor.YUV2RGB_YUNV = cvtColor.YUV2RGB_YUY2
cvtColor.YUV2BGR_YUNV = cvtColor.YUV2BGR_YUY2

cvtColor.YUV2RGBA_YUY2 = 119
cvtColor.YUV2BGRA_YUY2 = 120
cvtColor.YUV2RGBA_YVYU = 121
cvtColor.YUV2BGRA_YVYU = 122
cvtColor.YUV2RGBA_YUYV = cvtColor.YUV2RGBA_YUY2
cvtColor.YUV2BGRA_YUYV = cvtColor.YUV2BGRA_YUY2
cvtColor.YUV2RGBA_YUNV = cvtColor.YUV2RGBA_YUY2
cvtColor.YUV2BGRA_YUNV = cvtColor.YUV2BGRA_YUY2

cvtColor.YUV2GRAY_UYVY = 123
cvtColor.YUV2GRAY_YUY2 = 124
    -- cvtColor.YUV2GRAY_VYUY = cvtColor.YUV2GRAY_UYVY
cvtColor.YUV2GRAY_Y422 = cvtColor.YUV2GRAY_UYVY
cvtColor.YUV2GRAY_UYNV = cvtColor.YUV2GRAY_UYVY
cvtColor.YUV2GRAY_YVYU = cvtColor.YUV2GRAY_YUY2
cvtColor.YUV2GRAY_YUYV = cvtColor.YUV2GRAY_YUY2
cvtColor.YUV2GRAY_YUNV = cvtColor.YUV2GRAY_YUY2

    -- alpha premultiplication
cvtColor.RGBA2mRGBA = 125
cvtColor.mRGBA2RGBA = 126

cvtColor.RGB2YUV_I420 = 127
cvtColor.BGR2YUV_I420 = 128
cvtColor.RGB2YUV_IYUV = cvtColor.RGB2YUV_I420
cvtColor.BGR2YUV_IYUV = cvtColor.BGR2YUV_I420

cvtColor.RGBA2YUV_I420 = 129
cvtColor.BGRA2YUV_I420 = 130
cvtColor.RGBA2YUV_IYUV = cvtColor.RGBA2YUV_I420
cvtColor.BGRA2YUV_IYUV = cvtColor.BGRA2YUV_I420
cvtColor.RGB2YUV_YV12  = 131
cvtColor.BGR2YUV_YV12  = 132
cvtColor.RGBA2YUV_YV12 = 133
cvtColor.BGRA2YUV_YV12 = 134

cvtColor.COLORCVT_MAX  = 135

libopencv.cvtColor_types = cvtColor

function libopencv.fromTensor(tensor,dimensions)
   dimensions = dimensions or 'DHW'

   local ndim = tensor:nDimension()

   local height, width, depth, cvtype

   if ndim == 3 then
      if dims == 'DHW' then
         depth,height,width= tensor:size(1),tensor:size(2),tensor:size(3)
         tensor = tensor:transpose(1,3):transpose(1,2)
      else -- dims == 'HWD'
         height,width,depth = tensor:size(1),tensor:size(2),tensor:size(3)
      end
   elseif ndim == 2 then
      depth  = 1
      height = tensor:size(1)
      width  = tensor:size(2)
   end

   -- Force contiguous:
   tensor = tensor:contiguous()

   -- create cv Matrix
   mat = ffi.new("Mat")
   mat.rows = height
   mat.cols = width
   tensor_type = tensor:type()
   if tensor_type == "torch.DoubleTensor" then
      cvtype = "CV_64F"
      cvmat.data.db  = torch.data(tensor);
      cvmat.step = 8 * width
   elseif tensor_type == "torch.FloatTensor" then
      cvtype = "CV_32F"
      cvmat.data.fl  = torch.data(tensor);
      cvmat.step = 4 * width
   elseif tensor_type == "torch.ByteTensor" then
      cvtype = "CV_8U"
      cvmat.data.ptr  = torch.data(tensor);
      cvmat.step = 1 * width
   elseif tensor_type == "torch.IntTensor" then
      cvtype = "CV_32S"
      cvmat.data.i  = torch.data(tensor);
   elseif tensor_type == "torch.LongTensor" then
      cvtype = "CV_32S"
      cvmat.data.i  = torch.data(tensor:int());
   elseif tensor_type == "torch.CharTensor" then
      cvtype = "CV_8S"
      cvmat.data.ptr  = torch.data(tensor);
   elseif tensor_type == "torch.ShortTensor" then
      cvtype = "CV_16S"
      cvmat.data.s  = torch.data(tensor);
   end
   cvmat.type  = types[cvtype][depth]

   return cvmat
end

-- datatype,colorspace, and dims determine the type of tensor we want.
function libopencv.toTensor(cvmat, datatype, colorspace, dims, nocopy)

   datatype   = datatype or torch.getdefaulttensortype()
   colorspace = colorspace or "RGB"
   dims       = dims or "DHW"

   -- Dims:
   height = cvmat.rows
   width  = cvmat.cols
   -- bytes per row
   step   = cvmat.step
   mtype  = cvmat.type

   print(height,width,step,mtype)

end


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
function libopencv.detect(img,detectorType,npts,mask)
   if type(img) == "userdata" and img:type():gmatch("Tensor") == "Tensor" then
      img = libopencv.C.Mat_fromTensor(img)
   elseif not (type(img) == "cdata") then
      error("Wrong type for img")
   end
   detectorType = detectorType or "FAST"
   -- to do
   mask = mask or ffi.new("Mat")
   npts = npts or 1000
   keypoints = ffi.new("KeyPoint[?]", npts)
   npts = libopencv.C.detect(img_Mat,detectorType,mask,keypoints[0],npts)
   return keypoints, npts
end

function libopencv.debug_keypoints(img_Mat,kpts,npts)
   libopencv.C.debug_keypoints(img_Mat,kpts,npts)
end

return libopencv
