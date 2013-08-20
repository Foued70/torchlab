ffi = require 'ffi'
libopencv = util.ffi.load("libopencv")
ctorch = util.ctorch

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

Matcher = Class()

Matcher.types = require './types/Matcher'

-- <input> String matcherType
function Matcher:__init(matcherType)
  matcherType = matcherType or "BruteForce"
  if not Matcher.types[matcherType]  then
    error("need to pass appropriate matcher type")
  end
  self.matcher = ffi.gc(libopencv.DescriptorMatcher_create(ffi.string(matcherType)),
   function (matcher)
    libopencv.DescriptorMatcher_destroy(matcher)
    end)
end

-- <input> matcher, source descriptor matrix, dest descriptor matrix, nmatches max
-- <output> matches and number of matches found
function Matcher:match(descriptors_src, descriptors_dest, nmatches_max)
  if ((not descriptors_src.mat) or (type(descriptors_src.mat) ~= "cdata")) then 
    error("problem with source descriptors")
  end
  if ((not descriptors_dest.mat) or (type(descriptors_dest.mat) ~= "cdata")) then 
    error("problem with source descriptors")
  end
  if type(nmatches_max) ~= "number" then
    error("need to pass integer for max number of matches")
  end
  matches = ffi.new("DMatch[?]", nmatches_max)
  nmatches_new = libopencv.DescriptorMatcher_match(self.matcher,descriptors_src.mat, descriptors_dest.mat, matches, nmatches_max)
  return matches, nmatches_new
end

-- <input> matches, number of matches
-- <output> reduced matches, number of reduced matches
function Matcher:reduce(matches,nmatches)
  if (type(matches) ~= "cdata") then 
    error("problem with input matches")
  end
  if type(nmatches) ~= "number" then
    error("need to pass integer for number of matches")
  end
  matches_good = ffi.new("DMatch[?]", nmatches)
  nmatches_new = libopencv.DescriptorMatcher_reduceMatches(matches, nmatches, matches_good)
  return matches_good, nmatches_new
end

return Matcher
