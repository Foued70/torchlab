ffi = require 'ffi'
libopencv = util.ffi.load("libopencv")
ctorch = util.ctorch

Matcher = {}

ffi.cdef [[
// ------------
//   opaque pointer (not visible from Lua interface)
// ------------
typedef struct DescriptorMatcher DescriptorMatcher;

// from opencv2/features2d/features2d.hpp
typedef struct DMatch
{
    int queryIdx; // query descriptor index
    int trainIdx; // train descriptor index
    int imgIdx;   // train image index

    float distance;
} DMatch ;

DescriptorMatcher* DescriptorMatcher_create(const char* feature_type);
int DescriptorMatcher_match(DescriptorMatcher* matcher, const Mat*  descriptors_src, const Mat*  descriptors_dest, DMatch* matchesC, int npts);
void DescriptorMatcher_destroy(DescriptorMatcher* matcher);
int DescriptorMatcher_reduceMatches(DMatch* matchesptr, int nmatches, DMatch* matchesReducedC);
]]

Matcher.types = require './types/Matcher'

-- <input> String matcherType
function Matcher.create(matcherType)
  matcherType = matcherType or Matcher.types[1]
  mtype = Matcher.types[matcherType] 
  if not mtype then
    error("need to pass appropriate matcher type")
  end
  return ffi.gc(libopencv.DescriptorMatcher_create(ffi.string(matcherType)),
                 function (matcher)
                    libopencv.DescriptorMatcher_destroy(matcher)
                 end)
end

-- <input> matcher, source descriptor matrix, dest descriptor matrix, nmatches max
-- <output> matches and number of matches found
function Matcher.match(matcher,descriptors_src,descriptors_dest, nmatches_max)
   if type(matcher) ~= "cdata" then
      error("need to pass opencv matcher object for first argument")
   end
   if type(descriptors_src) ~= "cdata" then
      error("need to pass opencv Mat object for second argument")
   end
    if type(descriptors_dest) ~= "cdata" then
      error("need to pass opencv Mat object for third argument")
   end
    if type(nmatches_max) ~= "number" then
      error("need to pass integer for fourth argument")
   end
   matches = ffi.new("DMatch[?]", nmatches_max)

   nmatches_new = libopencv.DescriptorMatcher_match(matcher,descriptors_src, descriptors_dest, matches, nmatches_max)
   return matches, nmatches_new
end

-- <input> matches, number of matches
-- <output> reduced matches, number of reduced matches
function Matcher.reduce(matches,nmatches)
   if type(matches) ~= "cdata" then
      error("need to pass opencv DMatch list object for first argument")
   end
    if type(nmatches) ~= "number" then
      error("need to pass integer for fifth argument")
   end
   matches_good = ffi.new("DMatch[?]", nmatches)
   nmatches_new = libopencv.DescriptorMatcher_reduceMatches(matches, nmatches, matches_good)
   return matches_good, nmatches_new
end

return Matcher
