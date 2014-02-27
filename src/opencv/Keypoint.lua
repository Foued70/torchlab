ffi        = require 'ffi'
opencv_ffi  = require './opencv_ffi'

Keypoint = Class()

local function destructor ()
   return function (kpts)
      opencv_ffi.KeyPoint_destroy(kpts)
   end
end

function Keypoint:__init(t)
   self.name = "opencv.Keypoint"
   -- Arg?

   if not t then
      error('not tensor given')
   else
      if type(t) == 'userdata' then
         -- Is a tensor: 
         self:fromTensor(t) 
      end
   end
end

function Keypoint:fromTensor(tensor)

   tensor_type = tensor:type()

   tensor = tensor:contiguous()

   keypoints = ffi.gc(opencv_ffi.THDoubleTensor_toKeypoints(torch.cdata(tensor:double())), destructor())

   -- make sure we keep link to original torch Tensor so data is not
   -- garbage collected should original tensor go out of scope.
   self.tensor = tensor 
   self.keypoints    = keypoints
   self.npts   = self.tensor:size(1)

end
