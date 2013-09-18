opencv_ffi = require './opencv_ffi'

Class()

--source should be nxp, where n is number of points and p is dimension
function flann_knn(source, dest, knn)
   if ((not source.mat) or (type(source.mat) ~= "cdata")) then 
      error("problem with source points")
   end
   if ((not dest.mat) or (type(dest.mat) ~= "cdata")) then 
      error("problem with dest points")
   end
   if not(knn) then
   		knn = 1
   end
   dis = opencv.Mat.new()
   indices = opencv.Mat.new()
   opencv_ffi.flann_knn(source.mat, dest.mat, knn, indices.mat, dis.mat);
   return opencv.Mat.new(indices:toTensor()+1), dis
end


--source should be nxp, where n is number of points and p is dimension
function flann_search(source, dest, radius, maxresults)
   if ((not source.mat) or (type(source.mat) ~= "cdata")) then 
      error("problem with source points")
   end
   if ((not dest.mat) or (type(dest.mat) ~= "cdata")) then 
      error("problem with dest points")
   end
   if not(radius) then
      error("radius must be defined")
   end
   dis = opencv.Mat.new()
   indices = opencv.Mat.new()
   opencv_ffi.flann_radius(source.mat, dest.mat, radius, maxresults, indices.mat, dis.mat);
   return opencv.Mat.new(indices:toTensor()+1), dis
end