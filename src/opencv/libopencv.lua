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

-- data types CV_<bit_depth>{U|S|F}C{num_channels} 

-- copy needed structs from opencv2/core/types_c.h
-- these will create the interface between torch and opencv

ffi.cdef
[[

typedef unsigned char uchar;

void CvArr;

typedef struct CvMat
{
  int type;
  int step;

  int* refcount;
  int hdr_refcount;

  union
  {
    uchar* ptr;
    short* s;
    int* i;
    float* fl;
    double* db;
  } data;

  int rows;
  int cols;
}
  CvMat;

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

typedef struct CvMatND
{
  int type;
  int dims;

  int* refcount;
  int hdr_refcount;

  union
  {
    uchar* ptr;
    float* fl;
    double* db;
    int* i;
    short* s;
  } data;

  struct
  {
    int size;
    int step;
  }
    dim[32];
} CvMatND;

struct CvSet;

typedef struct CvSparseMat
{
  int type;
  int dims;
  int* refcount;
  int hdr_refcount;

  struct CvSet* heap;
  void** hashtable;
  int hashsize;
  int valoffset;
  int idxoffset;
  int size[32];
} CvSparseMat;

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

typedef struct KeyPointVector
{
  int length;
  KeyPoint *data;
} KeyPointVector;

]]

-- This is a list of wrapper functions encapsulating the opencv
-- functions we want.  The wrapper functions are defined in opencv.cpp
-- in an extern "C" block to make the C++ callable from lua C.) These
-- functions call the needed C++ operators. They accept and return the
-- opencv C structs defined above.
ffi.cdef [[ 
KeyPointVector detect(const CvMat* data, const char* detector_type, const CvMat* mask);
void draw_keypoints(const CvMat* data, const KeyPointVector kpv);

]]

-- below starts our lua wrappers
libopencv = {}
-- load our C wrappers.
C = util.ffi.load('libcopencv')


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
   cvmat = ffi.new("CvMat")
   cvmat.rows = height
   cvmat.cols = width
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
function libopencv.toTensor(cvmat, dataType, colorspace, dims, nocopy)
   -- Dims:
   local width,height = self:size()

   height = cvmat.rows
   width  = cvmat.cols
   -- bytes per row
   step   = cvmat.step
   mtype  = cvmat.type
   print(height,width,step,mtype)
   t = torch.Tensor()
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

-- <input> CvMat, String detectorType , CvMat mask
-- <output> KeyPointsVector
function libopencv.detect(img_cvmat,detectorType,mask)
   detectorType = detectorType or "FAST"
   mask = mask or ffi.new("CvMat")
   kpvect = C.detect(img_cvmat,detectorType,mask)
   return kpvect
end

function libopencv.draw_keypoints(img_cvmat,kpvect)
   C.draw_keypoints(img_cvmat,kpvect)
end
-- function opencv.draw_keypoints()
return libopencv
