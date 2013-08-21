ffi = require 'ffi'
libopencv = util.ffi.load("libopencv")
ctorch = util.ctorch

-- This is a list of wrapper functions encapsulating the opencv
-- functions we want.  The wrapper functions are defined in Mat.cpp
-- in an extern "C" block to make the C++ callable from lua C.) These
-- functions call the needed C++ operators. They accept and return the
-- opencv C structs defined above.
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
]]

-- ------------
--   generic functions implemented in generic/opencv.cpp
-- ------------

ctorch.generateTypes [[
Mat* THTensor_toMat(THTensor* tensor);
void THTensor_fromMat(Mat* mat,THTensor* tensor);
]]

types      = require './types/Mat'
conversion = require './types/Mat_conversion'

local function destructor ()
   return function (mat)
      libopencv.Mat_destroy(mat)
   end
end

Mat = Class()

function Mat:__init(pathOrTensorOrMat, ...)
   self.name = "opencv.Mat"
   -- Arg?
   args  = {...} 
   nargs = #args
   if not pathOrTensorOrMat then
      -- initialize an empty Mat*
      self.mat = ffi.gc(libopencv.Mat_create(0,0,0), destructor())
   else
      if type(pathOrTensorOrMat) == 'string' then
         -- Is a path: use opencv.load_image
         self:load(pathOrTensorOrMat, ...)   
      elseif type(pathOrTensorOrMat) == 'userdata' then
         -- Is a tensor: 
         self:fromTensor(pathOrTensorOrMat, ...) 
      elseif type(pathOrTensorOrMat) == 'cdata' then
         if nargs == 1 and type(args[1]) == "number" then
            -- this is a Keypoint vector
            npts = args[1]
            self.mat = ffi.gc(libopencv.getMatFromKeypoints(pathOrTensorOrMat, npts), destructor())
         else
            -- Is an opencv Mat* : wrap with the destructor
            self.mat = ffi.gc(pathOrTensorOrMat, destructor())
         end
      end
   end
end
function Mat:info()
   if self.mat then 
      libopencv.Mat_info(self.mat)
   else
      error("No mat to display info")
   end
end
function Mat:matType()
   if self.mat then 
      return libopencv.Mat_type(self.mat)
   else
      error("No mat to get type")
   end
end
function Mat:size()
   if self._size then 
      return self._size
   else
      if self.mat then 
         self._size = torch.LongStorage()
         libopencv.Mat_size(self.mat,torch.cdata(self._size))
         return self._size
      end
   end
   return
end

function Mat:stride()
   if self._stride then 
      return self._stride
   else
      if self.mat then 
         self._stride = torch.LongStorage()
         libopencv.Mat_stride(self.mat,torch.cdata(self._stride))
         return self._stride
      end
   end
   return
end

function Mat:display(title)
   if self.mat then 
      title = title or self.name
      libopencv.Mat_showImage(self.mat,title)
   else
      error("No mat to display")
   end
end


function Mat:load(path)
   self.mat = ffi.gc(libopencv.Mat_loadImage(path), destructor())
end

function Mat:clone()
   return Mat.new(ffi.gc(libopencv.Mat_clone(self.mat), destructor()))
end

function Mat:fromTensor(tensor,dimensions)

   tensor_type = tensor:type()

   if tensor_type == "torch.DoubleTensor" then
      mat = ffi.gc( libopencv.THDoubleTensor_toMat( torch.cdata(tensor)), destructor())
   elseif tensor_type == "torch.FloatTensor" then
      mat = ffi.gc(  libopencv.THFloatTensor_toMat( torch.cdata(tensor)), destructor())
   elseif tensor_type == "torch.ByteTensor" then
      mat = ffi.gc(   libopencv.THByteTensor_toMat( torch.cdata(tensor)), destructor())
   elseif tensor_type == "torch.IntTensor" then
      mat = ffi.gc(    libopencv.THIntTensor_toMat( torch.cdata(tensor)), destructor())
   elseif tensor_type == "torch.LongTensor" then
      print("Warning no analog for LongTensor in opencv. casting to int")
      tensor = tensor:int()
      mat = ffi.gc(    libopencv.THIntTensor_toMat( torch.cdata(tensor)),destructor())
   elseif tensor_type == "torch.CharTensor" then
      mat = ffi.gc(   libopencv.THCharTensor_toMat( torch.cdata(tensor)), destructor())
   elseif tensor_type == "torch.ShortTensor" then
      mat = ffi.gc(  libopencv.THShortTensor_toMat( torch.cdata(tensor)), destructor())
   end

   self.mat = mat

end

-- datatype,colorspace, and dims determine the type of tensor we want.
function Mat:toTensor(dimension)
   mat = self.mat
   depth = libopencv.Mat_depth(mat)
   if depth == 0 then  -- CV_8U
      tensor = torch.ByteTensor()
      libopencv.THByteTensor_fromMat(mat,torch.cdata(tensor)) 
   elseif depth == 1 then -- CV_8S
      tensor = torch.CharTensor()
      libopencv.THCharTensor_fromMat(mat,torch.cdata(tensor))
   elseif depth == 2 then -- CV_16U
      error("no analog in torch for CV_16U")
   elseif depth == 3 then -- CV_16S
      tensor = torch.ShortTensor()
      libopencv.THShortTensor_fromMat(mat,torch.cdata(tensor))
   elseif depth == 4 then -- CV_32S
      tensor = torch.IntTensor()
      libopencv.THIntTensor_fromMat(mat,torch.cdata(tensor))
   elseif depth == 5 then -- CV_32F
      tensor = torch.FloatTensor()
      libopencv.THFloatTensor_fromMat(mat,torch.cdata(tensor))
   elseif depth == 6 then -- CV_64F
      tensor = torch.DoubleTensor()
      libopencv.THDoubleTensor_fromMat(mat,torch.cdata(tensor))
   else
      error("something is wrong")
   end
   if dimension == "DHW" then 
         tensor = tensor:transpose(3,1):transpose(2,3)
   end
   return tensor
end

--type_str should be a string, not a number!
function Mat:convert(...) 
   local output, type_str
   args = {...}
   nargs = #args
   if nargs == 1 then 
      type_str  = args[1]
   elseif nargs == 2 then 
      output    = args[1]
      type_str  = args[2]
   else
      error("wrong number of args" ..#args)
   end
   if not self.mat then 
      error("Mat not initialized no opencv data")
   end
   if type(type_str) ~= "string" then
      error("need to pass string as type of conversion")
   end
   type_enum  = conversion[type_str]
   if type_enum then 
      if output then 
         output_mat = output.mat
      else 
         output_mat = self.mat
      end
      libopencv.Mat_convert(self.mat,output_mat,type_enum)
   else
      error("Don't understand conversion type "..type_str)
   end 
   return output
end

return Mat
