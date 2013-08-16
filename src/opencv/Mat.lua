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
//   functions implemented in opencv.cpp
// ------------
Mat* Mat_create (int width, int height, int type);
Mat* Mat_clone (Mat* mat);

int  Mat_depth  (Mat* mat);
void Mat_size   (Mat* mat, THLongStorage* size);
void Mat_stride (Mat* mat, THLongStorage* stride);

Mat* Mat_loadImage (const char* fname);
void Mat_showImage (Mat* mat, const char* wname);
void Mat_convert   (Mat* input, Mat* output, int cvttype);
void Mat_info      (Mat* mat);
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

Mat = Class()

function Mat:__init(pathOrTensorOrMat, ...)
   self.name = "opencv.Mat"
      -- Arg?
   if type(pathOrTensorOrMat) == 'string' then
      -- Is a path:
      self:load(pathOrTensorOrMat, ...)
   
   elseif type(pathOrTensorOrMat) == 'userdata' then
      -- Is a tensor:
      self:fromTensor(pathOrTensorOrMat, ...)
   elseif type(pathOrTensorOrMat) == 'cdata' then
      -- Is an opencv Mat*
      self.mat = pathOrTensorOrMat
   end
end

function Mat:info()
   if self.mat then 
      libopencv.Mat_info(self.mat)
   end
end

local function destructor ()
   return function (mat)
      libopencv.Mat_destroy(mat)
   end
end

function Mat:load(path)
   self.mat = ffi.gc(libopencv.Mat_loadImage(path), destructor())
end

function Mat:clone(path)
   return Mat.new(ffi.gc(libopencv.Mat_clone(self.mat), destructor()))
end

function Mat:fromTensor(tensor,dimensions)

   tensor_type = tensor:type()

   if tensor_type == "torch.DoubleTensor" then
      mat = ffi.gc(libopencv.THDoubleTensor_toMat(torch.cdata(tensor)),
                   destructor())
   elseif tensor_type == "torch.FloatTensor" then
      mat = ffi.gc(libopencv.THFloatTensor_toMat(torch.cdata(tensor)),
                   function (mat)
                      libopencv.Mat_destroy(mat)
                   end)
   elseif tensor_type == "torch.ByteTensor" then
      mat = ffi.gc(libopencv.THByteTensor_toMat(torch.cdata(tensor)),
                   function (mat)
                      libopencv.Mat_destroy(mat)
                   end)
   elseif tensor_type == "torch.IntTensor" then
      mat = ffi.gc(libopencv.THIntTensor_toMat(torch.cdata(tensor)),
                   function (mat)
                      libopencv.Mat_destroy(mat)
                   end)
   elseif tensor_type == "torch.LongTensor" then
      print("Warning no analog for LongTensor in opencv. casting to int")
      mat = ffi.gc(libopencv.THIntTensor_toMat(tensor:int()),
                   function (mat)
                      libopencv.Mat_destroy(mat)
                   end) 
   elseif tensor_type == "torch.CharTensor" then
      mat = ffi.gc(libopencv.THCharTensor_toMat(torch.cdata(tensor)),
                   function (mat)
                      libopencv.Mat_destroy(mat)
                   end)
   elseif tensor_type == "torch.ShortTensor" then
      mat = ffi.gc(libopencv.THShortTensor_toMat(torch.cdata(tensor)),
                   function (mat)
                      libopencv.Mat_destroy(mat)
                   end)
   end

   self.mat = mat

end

-- datatype,colorspace, and dims determine the type of tensor we want.
function Mat:toTensor() -- , datatype, colorspace, dims, nocopy)
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
   return tensor
end


return Mat
