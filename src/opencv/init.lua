ffi = require 'ffi'
libopencv = require './libopencv'
Class()

function MatfromTensor(tensor,dimensions)

   -- TODO: to convert automatically or not
   if dimensions and dimensions == "DHW" then
         tensor = tensor:transpose(1,3):transpose(1,2)
   end

   tensor_type = tensor:type()
   if tensor_type == "torch.DoubleTensor" then
      mat = ffi.gc(libopencv.C.THDoubleTensor_toMat(tensor),
                   function (mat)
                      libopencv.C.Mat_destroy(mat)
                   end)
   elseif tensor_type == "torch.FloatTensor" then
      mat = ffi.gc(libopencv.C.THFloatTensor_toMat(tensor),
                   function (mat)
                      libopencv.C.Mat_destroy(mat)
                   end)
   elseif tensor_type == "torch.ByteTensor" then
      mat = ffi.gc(libopencv.C.THByteTensor_toMat(tensor),
                   function (mat)
                      libopencv.C.Mat_destroy(mat)
                   end)
   elseif tensor_type == "torch.IntTensor" then
      mat = ffi.gc(libopencv.C.THIntTensor_toMat(tensor),
                   function (mat)
                      libopencv.C.Mat_destroy(mat)
                   end)
   elseif tensor_type == "torch.LongTensor" then
      print("Warning no analog for LongTensor in opencv. casting to int")
      mat = ffi.gc(libopencv.C.THIntTensor_toMat(tensor:int()),
                   function (mat)
                      libopencv.C.Mat_destroy(mat)
                   end) 
   elseif tensor_type == "torch.CharTensor" then
      mat = ffi.gc(libopencv.C.THCharTensor_toMat(tensor),
                   function (mat)
                      libopencv.C.Mat_destroy(mat)
                   end)
   elseif tensor_type == "torch.ShortTensor" then
      mat = ffi.gc(libopencv.C.THShortTensor_toMat(tensor),
                   function (mat)
                      libopencv.C.Mat_destroy(mat)
                   end)
   end

   return mat
end

-- datatype,colorspace, and dims determine the type of tensor we want.
function MattoTensor(mat) -- , datatype, colorspace, dims, nocopy)
   depth = libopencv.C.Mat_depth(mat)
   if depth == 0 then  -- CV_8U
      tensor = torch.ByteTensor()
      libopencv.C.THByteTensor_fromMat(mat,tensor)
   elseif depth == 1 then -- CV_8S
      tensor = torch.CharTensor()
      libopencv.C.THCharTensor_fromMat(mat,tensor)
   elseif depth == 2 then -- CV_16U
      error("no analog in torch for CV_16U")
   elseif depth == 3 then -- CV_16S
      tensor = torch.ShortTensor()
      libopencv.C.THShortTensor_fromMat(mat,tensor)
   elseif depth == 4 then -- CV_32S
      tensor = torch.IntTensor()
      libopencv.C.THIntTensor_fromMat(mat,tensor)
   elseif depth == 5 then -- CV_32F
      tensor = torch.FloatTensor()
      libopencv.C.THFloatTensor_fromMat(mat,tensor)
   elseif depth == 6 then -- CV_64F
      tensor = torch.DoubleTensor()
      libopencv.C.THDoubleTensor_fromMat(mat,tensor)
   else
      error("something is wrong")
   end
   return tensor
end

