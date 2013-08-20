ffi       = require 'ffi'
libopencv = util.ffi.load("libopencv")
Mat       = require './Mat'

ffi.cdef [[
Mat* getHomography(const KeyPoint* keyptr_src,  int npts_src, 
                   const KeyPoint* keyptr_dest, int npts_dest, 
                     const DMatch* matchptr,    int npts_match);
]] 

calib3d = {}
-- <input> matcher, source descriptor matrix, dest descriptor matrix, nmatches max
-- <output> matches and number of matches found
function calib3d.getHomography(keypoints_src, npts_src, keypoints_dest, npts_dest, matches, nmatches)
   if type(keypoints_src) ~= "cdata" then
      error("need to pass opencv keypoints object for first argument")
   end
   if type(npts_src) ~= "number" then
      error("need to pass number object for second argument")
   end
   if type(keypoints_dest) ~= "cdata" then
      error("need to pass opencv keypoints object for third argument")
   end
   if type(npts_dest) ~= "number" then
      error("need to pass number object for fourth argument")
   end
   if type(matches) ~= "cdata" then
      error("need to pass opencv DMatch object for fifth argument")
   end
   if type(nmatches) ~= "number" then
      error("need to pass number object for sixth argument")
   end
   return Mat.new(libopencv.getHomography(keypoints_src, npts_src, keypoints_dest, npts_dest, matches, nmatches))
end

return calib3d
