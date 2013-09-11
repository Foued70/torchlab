opencv_ffi = require './opencv_ffi'

Class()

function draw_keypoints(img,kpts,npts)
   if ((not img.mat) or (type(img.mat) ~= "cdata")) then 
      error("expecting Mat object as first arg.")
   end
   if type(kpts) ~= "cdata" then 
      error("expecting opencv KeyPoints* array")
   end
   opencv_ffi.draw_keypoints(img.mat,kpts,npts)
end

function dump_keypoints(kpts,npts)
   if type(kpts) ~= "cdata" then 
      error("expecting opencv KeyPoints* array")
   end
   opencv_ffi.dump_keypoints(kpts,npts)
end
   
