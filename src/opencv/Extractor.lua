ffi = require 'ffi'
libopencv = util.ffi.load("libopencv")
ctorch = util.ctorch
Mat    = require './Mat'

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

Extractor = Class()

Extractor.types = require './types/Extractor'

-- <input> String extractorType
function Extractor:__init(extractorType)
   extractorType = extractorType or "ORB"
  if not Extractor.types[extractorType] then
      error("Don't understand extractorType: "..extractorType) 
  end
   self.extractor = ffi.gc(libopencv.DescriptorExtractor_create(ffi.string(extractorType)),
                 function (extractor)
                    libopencv.DescriptorExtractor_destroy(extractor)
                 end)
end

-- <input> extractor, image, keypoints, nkeypoints, (optional) descriptor
-- <output> descriptor
function Extractor:compute(img,keypoints,npts,descriptor)
   if ((not img.mat) or (type(img.mat) ~= "cdata")) then 
      error("problem with input image")
   end
   if type(keypoints) ~= "cdata" then
      error("need to pass opencv keypoints object as second arg")
   end
   descriptor = descriptor or Mat.new()
   if ((not descriptor.mat) or (type(descriptor.mat) ~= "cdata")) then 
      error("problem with descriptor mat")
   end
   libopencv.DescriptorExtractor_compute(self.extractor,img.mat,descriptor.mat,keypoints[0],npts)
   return descriptor
end

return Extractor
