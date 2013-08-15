ffi = require 'ffi'
libopencv = util.ffi.load("libopencv")
ctorch = util.ctorch

Extractor = {}

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

Extractor.types = require './types/Extractor'

-- <input> String extractorType
function Extractor.create(extractorType)
   extractorType = extractorType or "ORB"
   return ffi.gc(libopencv.DescriptorExtractor_create(ffi.string(extractorType)),
                 function (extractor)
                    libopencv.DescriptorExtractor_destroy(extractor)
                 end)
end

-- <input> extractor, image, keypoints, nkeypoints, (optional) descriptor
-- <output> descriptor
function Extractor.compute(extractor,image,keypoints,npts,descriptor)
   if type(extractor) ~= "cdata" then
      error("need to pass opencv extractor object")
   end
   if type(image) ~= "cdata" then
      error("need to pass opencv Mat object")
   end
   if type(keypoints) ~= "cdata" then
      error("need to pass opencv keypoints object")
   end
   descriptor = descriptor or libopencv.Mat_create(0,0,0)
   libopencv.DescriptorExtractor_compute(extractor,image,descriptor,keypoints[0],npts)
   return descriptor
end


return Extractor
