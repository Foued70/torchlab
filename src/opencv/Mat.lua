ffi        = require 'ffi'
libopencv  = require './libopencv'

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

function Mat:size(dim)
   if not self._size then 
      if self.mat then 
         self._size = torch.LongStorage()
         libopencv.Mat_size(self.mat,torch.cdata(self._size))
      else
         error("No mat to get size") 
      end
   end
   if dim and type(dim) == "number" then 
      return self._size[dim]
   else
      return self._size
   end
end

function Mat:stride(dim)
   if not self._stride then 
      if self.mat then 
         self._stride = torch.LongStorage()
         libopencv.Mat_stride(self.mat,torch.cdata(self._stride))
      else
         error("No mat to get stride") 
      end
   end
   if dim and type(dim) == "number" then 
      return self._stride[dim]
   else
      return self._stride
   end
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

function Mat:fromTensor(tensor)

   tensor_type = tensor:type()

   self.tensor = tensor:contiguous()

   if tensor_type == "torch.DoubleTensor" then
      mat = ffi.gc( libopencv.THDoubleTensor_toMat( torch.cdata(self.tensor)), destructor())
   elseif tensor_type == "torch.FloatTensor" then
      mat = ffi.gc(  libopencv.THFloatTensor_toMat( torch.cdata(self.tensor)), destructor())
   elseif tensor_type == "torch.ByteTensor" then
      mat = ffi.gc(   libopencv.THByteTensor_toMat( torch.cdata(self.tensor)), destructor())
   elseif tensor_type == "torch.IntTensor" then
      mat = ffi.gc(    libopencv.THIntTensor_toMat( torch.cdata(self.tensor)), destructor())
   elseif tensor_type == "torch.LongTensor" then
      print("Warning no analog for LongTensor in opencv. casting to int")
      tensor = tensor:int()
      mat = ffi.gc(    libopencv.THIntTensor_toMat( torch.cdata(self.tensor)),destructor())
   elseif tensor_type == "torch.CharTensor" then
      mat = ffi.gc(   libopencv.THCharTensor_toMat( torch.cdata(self.tensor)), destructor())
   elseif tensor_type == "torch.ShortTensor" then
      mat = ffi.gc(  libopencv.THShortTensor_toMat( torch.cdata(self.tensor)), destructor())
   end

   -- make sure we keep link to original torch Tensor so data is not
   -- garbage collected should original tensor go out of scope.
   self.tensor = tensor 
   self.mat    = mat

end

-- dimensions can be HWD (opencv default) or DHW torch default
function Mat:toTensor(dimensions_or_nocopy)
   tensor = nil
   if not self.tensor then 
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
      
      self.tensor = tensor
   end

   if dimensions_or_nocopy == "DHW" then 
      -- changing dimensions always returns a copy
      return self.tensor:transpose(3,1):transpose(2,3):contiguous() 
   elseif dimensions_or_nocopy then 
      -- dangerous assume user wants pointer to the raw data
      return self.tensor
   else 
      return self.tensor:clone()
   end
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
