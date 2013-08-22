libopencv = require './libopencv'

Class()

function draw_keypoints(img,kpts,npts)
   if ((not img.mat) or (type(img.mat) ~= "cdata")) then 
      error("expecting Mat object as first arg.")
   end
   if type(kpts) ~= "cdata" then 
      error("expecting opencv KeyPoints* array")
   end
   libopencv.draw_keypoints(img.mat,kpts,npts)
end

function dump_keypoints(kpts,npts)
   if type(kpts) ~= "cdata" then 
      error("expecting opencv KeyPoints* array")
   end
   libopencv.dump_keypoints(kpts,npts)
end
   
