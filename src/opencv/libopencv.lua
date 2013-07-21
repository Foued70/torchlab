-- FFI bindings to Opencv:
local ffi = require "ffi"

-- /* data types CV_<bit_depth>{U|S|F}C{num_channels} */

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
}
  CvMatND;

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
}
  CvSparseMat;

typedef struct CvSize
{
  int width;
  int height;
}
  CvSize;

void detectfeatures(const CvMat* data,char* detector_type);

]]

-- opencv_core has the C style CvMat stuff.
libopencv_core = util.ffi.load('libopencv_core')

-- load our wrappers.
libopencv = util.ffi.load('libluaopencv')

io = require 'io'
ctorch = util.ctorch

opencv = {}

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

opencv.types = types

function opencv.fromTensor(tensor,dimensions)
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
   elseif tensor_type == "torch.FloatTensor" then 
      cvtype = "CV_32F"
      cvmat.data.fl  = torch.data(tensor);
   elseif tensor_type == "torch.ByteTensor" then 
      cvtype = "CV_8U"
      cvmat.data.ptr  = torch.data(tensor);
   elseif tensor_type == "torch.IntTensor" then 
      cvtype = "CV_32S"
      cvmat.data.i  = torch.data(tensor);
   elseif tensor_type == "torch.LongTensor" then 
      cvtype = "CV_64S"
      cvmat.data.i  = torch.data(tensor);
   elseif tensor_type == "torch.CharTensor" then 
      cvtype = "CV_8S"
      cvmat.data.s  = torch.data(tensor);
   elseif tensor_type == "torch.ShortTensor" then 
      cvtype = "CV_16S"
      cvmat.data.s  = torch.data(tensor);
   end
   cvmat.type  = types[cvtype][depth]

   return cvmat
end

return opencv

-- idea for automatically parsing the c headers

-- shell : (careful additionally need to change uchar to unsigned char)

-- for header in `find ${CLOUDLAB_INSTALL_ROOT}/include/opencv2/ -name '*_c.h'`
-- do gcc -E -I${CLOUDLAB_INSTALL_ROOT}/include/ $header >>opencv.h 
-- done

-- luvit : (but I still can't figure out the childprocess stuff)
-- local include_dir = CLOUDLAB_ROOT .. "/include/"
-- local header_files = { "opencv2/core/types_c.h", "opencv2/core/core_c.h" }
-- local ffi_string = ""
-- for i,v in pairs(header_files) do 
--    file_path = include_dir .. v
--    if not util.fs.is_file(file_path) then 
--       error("can't find header" .. file_path)
--    end
--    --execute 
-- end

