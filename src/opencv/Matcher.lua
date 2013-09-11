ffi = require 'ffi'
opencv_ffi = require './opencv_ffi'

Matcher = Class()

types = require './types/Matcher'

-- <input> String matcherType
function Matcher:__init(matcherType)
  matcherType = matcherType or "BruteForce"
  if not Matcher.types[matcherType]  then
    error("need to pass appropriate matcher type")
  end
  self.matcher = ffi.gc(opencv_ffi.DescriptorMatcher_create(ffi.string(matcherType)),
   function (matcher)
    opencv_ffi.DescriptorMatcher_destroy(matcher)
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
  nmatches_new = opencv_ffi.DescriptorMatcher_match(self.matcher,descriptors_src.mat, descriptors_dest.mat, matches, nmatches_max)
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
  nmatches_new = opencv_ffi.DescriptorMatcher_reduceMatches(matches, nmatches, matches_good)
  return matches_good, nmatches_new
end
