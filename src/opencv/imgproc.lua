opencv_ffi = require './opencv_ffi'

Class()

morph_types = require './types/Morph'
colors      = require './types/Colors.lua'

-- <input> image, homography
-- <output> image warped by homography
function warpImage(img, homography, size_x, size_y)
   if ((not img.mat) or (type(img.mat) ~= "cdata")) then 
      error("problem with input images")
   end
   if ((not homography.mat) or (type(homography.mat) ~= "cdata")) then 
      error("problem with homography matrix")
   end

   return opencv.Mat.new(opencv_ffi.warpImage(img.mat, homography.mat, size_x, size_y))
end

-- <input> image1, image2, homography
-- <output> combined images -1st channel is warped and translated image
function combineImages(img1, img2, homography)
   if ((not img1.mat) or (type(img1.mat) ~= "cdata")) then 
      error("problem with src image")
   end
   if ((not img2.mat) or (type(img2.mat) ~= "cdata")) then 
      error("problem with dest image")
   end
   if ((not homography.mat) or (type(homography.mat) ~= "cdata")) then 
      error("problem with homography matrix")
   end
   error("not implemented")
   --return opencv.Mat.new(opencv_ffi.combineImages(img1.mat, img2.mat, homography.mat, result_size_x, result_size_y, result_center_x, result_center_y))
end

-- <input> img, blockSize, ksize, k
-- <output> Mat of responses at every location
function detectCornerHarris(img,blockSize,ksize, k)
   if ((not img.mat) or (type(img.mat) ~= "cdata")) then 
      error("problem with input image")
   end
   if type(blockSize) ~= "number" then
      error("need to pass number as second arg")
   end
   if type(ksize) ~= "number" then
      error("need to pass number as third arg")
   end
   if type(k) ~= "number" then
      error("need to pass number as fourth arg")
   end
   return opencv.Mat.new(opencv_ffi.detectCornerHarris(img.mat, blockSize, ksize, k))
end

-- <input> image, threshold1, threshold2 -- see opencv function
-- <output> line mat
function CannyDetectEdges(img, threshold1, threshold2)
   if ((not img.mat) or (type(img.mat) ~= "cdata")) then 
      error("problem with input image")
   end
   if type(threshold1) ~= "number" then
      error("need to pass number for second argument")
   end
   if type(threshold2) ~= "number" then
      error("need to pass number for third argument")
   end

   return opencv.Mat.new(opencv_ffi.CannyDetectEdges(img.mat, threshold1, threshold2))
end

-- <input> -- see opencv function
-- <output> line mat
function HoughLinesRegular(img, rho, theta, threshold, srn, stn)
   if ((not img.mat) or (type(img.mat) ~= "cdata")) then 
      error("problem with input image")
   end
   if type(rho) ~= "number" then
      error("need to pass number for second argument")
   end
   if type(theta) ~= "number" then
      error("need to pass number for third argument")
   end
   if type(srn) ~= "number" then
      error("need to pass number for fourth argument")
   end   
   if type(stn) ~= "number" then
      error("need to pass number for fifth argument")
   end
   return opencv.Mat.new(opencv_ffi.HoughLinesRegular(img.mat, rho, theta, threshold, srn, stn))
end

-- <input> -- see opencv function
-- <output> line mat
function HoughLinesProbabilistic(img, rho, theta, threshold, mineLineLength, maxLineGap)
   if ((not img.mat) or (type(img.mat) ~= "cdata")) then 
      error("problem with input image")
   end
   if type(rho) ~= "number" then
      error("need to pass number for second argument")
   end
   if type(theta) ~= "number" then
      error("need to pass number for third argument")
   end
   if type(mineLineLength) ~= "number" then
      error("need to pass number for fourth argument")
   end   
   if type(maxLineGap) ~= "number" then
      error("need to pass number for fifth argument")
   end
   return opencv.Mat.new(opencv_ffi.HoughLinesProbabilistic(img.mat, rho, theta, threshold, mineLineLength, maxLineGap))
end

function getPairwiseDistances(A, B)
   if ((not A.mat) or (type(A.mat) ~= "cdata")) then 
      error("problem with A Mat - first argument")
   end
   if ((not B.mat) or (type(B.mat) ~= "cdata")) then 
      error("problem with B Mat -- second argument")
   end
   return opencv.Mat.new(opencv_ffi.getPairwiseDistances(A.mat,B.mat))
end

function findBestTransformation(goodLocationsX_src, goodLocationsY_src, scores_src, pairwise_dis_src, goodLocationsX_dest, goodLocationsY_dest, scores_dest, pairwise_dis_dest, corr_thresh,
      minInliers, numInliersMax, cornerComparisonThreshold, size_x, size_y)

   if ((not goodLocationsX_src.mat) or (type(goodLocationsX_src.mat) ~= "cdata")) then 
      error("problem with source locations Mat - first argument")
   end
   if ((not goodLocationsY_src.mat) or (type(goodLocationsY_src.mat) ~= "cdata")) then 
      error("problem with source locations Mat - second argument")
   end
   if ((not scores_src.mat) or (type(scores_src.mat) ~= "cdata")) then 
      error("problem with source scores Mat -- third argument")
   end
   if ((not pairwise_dis_src.mat) or (type(pairwise_dis_src.mat) ~= "cdata")) then 
      error("problem with source distance Mat - fourth argument")
   end
   if ((not goodLocationsX_dest.mat) or (type(goodLocationsX_dest.mat) ~= "cdata")) then 
      error("problem with dest locations Mat - fifth argument")
   end
   if ((not goodLocationsY_dest.mat) or (type(goodLocationsY_dest.mat) ~= "cdata")) then 
      error("problem with dest locations Mat - sixth argument")
   end

   if ((not scores_dest.mat) or (type(scores_dest.mat) ~= "cdata")) then 
      error("problem with dest scores Mat -- seventh argument")
   end
   if ((not pairwise_dis_dest.mat) or (type(pairwise_dis_dest.mat) ~= "cdata")) then 
      error("problem with dest distance Mat - eighth argument")
   end
   if (type(corr_thresh) ~= "number") then 
      error("problem with threshold - ninth argument")
   end
   if (type(minInliers) ~= "number") then 
      error("problem with min inliers - tenth argument")
   end
   if (type(numInliersMax) ~= "number") then 
      error("problem with max num of inliers - eleventh argument")
   end
   if (type(cornerComparisonThreshold) ~= "number") then 
      error("problem with corner threshold - twelfth argument")
   end
   if (type(size_x) ~= "number") then 
      error("problem with x size - thirteenth argument")
   end
   if (type(size_y) ~= "number") then 
      error("problem with y size - fourteenth argument")
   end
   matOfStuff= opencv.Mat.new(opencv_ffi.findBestTransformation(goodLocationsX_src.mat, goodLocationsY_src.mat, scores_src.mat, 
      pairwise_dis_src.mat, goodLocationsX_dest.mat, goodLocationsY_dest.mat, scores_dest.mat, pairwise_dis_dest.mat, corr_thresh,
      minInliers, numInliersMax, cornerComparisonThreshold, size_x, size_y))
   
   best_transformations = {};
   if(matOfStuff:toTensor():nDimension() ~=0) then
      best_pts = torch.zeros(matOfStuff:toTensor():size()[1])

      for i =1,matOfStuff:toTensor():size()[1] do
         best_transformations[i] = geom.Homography.new(matOfStuff:toTensor()[{i,{2,10}}]:reshape(3,3))
         best_pts[{i}] = matOfStuff:toTensor()[{i,1}]
      end
   else
      best_pts = torch.zeros(0);
   end
   return best_pts, best_transformations;
end

--DILATION AND EROSION ---
-- <input> -- see opencv function
-- <output> line mat
function getStructuringElement(structType, size_x, size_y, center_x, center_y)
   if type(structType) ~= "number" then
      error("need to pass opencv mat object for first argument")
   end
   if type(size_x) ~= "number" then
      error("need to pass number for second argument")
   end
   if type(size_y) ~= "number" then
      error("need to pass number for third argument")
   end
   if type(center_x) ~= "number" then
      error("need to pass number for fourth argument")
   end   
   if type(center_y) ~= "number" then
      error("need to pass number for fifth argument")
   end
   return opencv.Mat.new(opencv_ffi.getStructuringElement(structType, size_x, size_y, center_x, center_y))
end

function getDefaultStructuringMat(erosion_size)
   if type(erosion_size) ~= "number" then
      error("need to pass number for first argument")
   end
   return getStructuringElement(morph_types.MORPH_RECT, 2*erosion_size+1, 2*erosion_size+1, erosion_size, erosion_size);
end

--dilate the img 
function dilate(img, structuringElement)
   if ((not img.mat) or (type(img.mat) ~= "cdata")) then 
      error("problem with input images")
   end
   if ((not structuringElement.mat) or (type(structuringElement.mat) ~= "cdata")) then 
      error("problem with structuring element images")
   end
   opencv_ffi.dilate(img.mat, structuringElement.mat);
end

--erode the img 
function erode(img, structuringElement)
   if ((not img.mat) or (type(img.mat) ~= "cdata")) then 
      error("problem with input images")
   end
   if ((not structuringElement.mat) or (type(structuringElement.mat) ~= "cdata")) then 
      error("problem with structuring element images")
   end
   opencv_ffi.erode(img.mat, structuringElement.mat);
end
