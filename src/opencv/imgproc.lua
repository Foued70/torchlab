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
      minInliers, numInliersMax, cornerComparisonThreshold, minx, maxx, miny, maxy)
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
   if (type(minx) ~= "number") then 
      error("problem with x size - thirteenth argument")
   end
   if (type(maxx) ~= "number") then 
      error("problem with x size - fourteenth argument")
   end
   if (type(miny) ~= "number") then 
      error("problem with y size - 15th argument")
   end
   if (type(maxy) ~= "number") then 
      error("problem with y size - 16th argument")
   end
   matOfStuff= opencv.Mat.new(opencv_ffi.findBestTransformation(goodLocationsX_src.mat, goodLocationsY_src.mat, scores_src.mat, 
      pairwise_dis_src.mat, goodLocationsX_dest.mat, goodLocationsY_dest.mat, scores_dest.mat, pairwise_dis_dest.mat, corr_thresh,
      minInliers, numInliersMax, cornerComparisonThreshold, minx, maxx, miny, maxy))
   
   best_transformations = {};
   if(matOfStuff:toTensor():nDimension() ~=0) then
      best_pts = torch.zeros(matOfStuff:toTensor():size()[1])

      for i =1,matOfStuff:toTensor():size()[1] do
         best_transformations[i] = matOfStuff:toTensor()[{i,{2,10}}]:reshape(3,3)
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

--erode the img 
function getOrientation(img, ksize)
   if ((not img.mat) or (type(img.mat) ~= "cdata")) then 
      error("problem with input images")
   end
   if (not ksize) then 
      ksize = 3
   end
   ori = opencv.Mat.new()
   mag = opencv.Mat.new()
   opencv_ffi.get_orientation(img.mat, ksize, ori.mat, mag.mat);
   return ori, mag
end


--erode the img 
function phaseCorrelate(img, dest)
   if ((not img.mat) or (type(img.mat) ~= "cdata")) then 
      error("problem with input images")
   end
   if ((not dest.mat) or (type(dest.mat) ~= "cdata")) then 
      error("problem with dest image")
   end

   return opencv.Mat.new(opencv_ffi.phaseCorrelate(img.mat, dest.mat));
end

function floodFill(img, x,y)
   if ((not img.mat) or (type(img.mat) ~= "cdata")) then 
      error("problem with input images")
   end
   dest = opencv.Mat.new(torch.zeros(img:size(1)+2, img:size(2)+2):byte())  
   opencv_ffi.flood_fill(img.mat, dest.mat, x, y)
   return dest
end

function  find_contours(img)
   if ((not img.mat) or (type(img.mat) ~= "cdata")) then 
      error("problem with input images")
   end
   dest = opencv_ffi.find_contours(img.mat)
   return opencv.Mat.new(dest)
end

function distanceTransform(img)
   if ((not img.mat) or (type(img.mat) ~= "cdata")) then 
      error("problem with input images")
   end
   dest = opencv.Mat.new(torch.zeros(img:size(1), img:size(2)):byte())  
   opencv_ffi.distance_transform(img.mat, dest.mat)
   return dest
end

function fillQuad(img, xyTensor)
   if ((not img.mat) or (type(img.mat) ~= "cdata")) then 
      error("problem with input images")
   end
   if(xyTensor:size(1) ~=4 or xyTensor:size(2)~=2) then
      error("wrong tensor size, should be 4x2")
   end
   xyTensor = xyTensor-1
   xyTensor = torch.cat(xyTensor:select(2,2),xyTensor:select(2,1),2) --flip x and y for opencv
   opencv_ffi.fillQuad(img.mat, xyTensor[1][1], xyTensor[1][2], xyTensor[2][1], xyTensor[2][2],
                                    xyTensor[3][1], xyTensor[3][2], xyTensor[4][1], xyTensor[4][2])
   return img
end
function fillQuadAll(img, xyTensor)
   if ((not img.mat) or (type(img.mat) ~= "cdata")) then 
      error("problem with input images")
   end
   if(xyTensor:size(2)~=8) then
      error("wrong tensor size, should be nx2")
   end
   xyTensor = xyTensor-1
   xyTensor = xyTensor:index(2,torch.LongTensor({2,1,4,3,6,5,8,7})) --flip for opencv
   opencv_ffi.fillQuadAll(img.mat,opencv.Mat.new(xyTensor).mat)
   return img
end

function fillQuadAllWithInterpolation(img, xyTensor)
   if ((not img.mat) or (type(img.mat) ~= "cdata")) then 
      error("problem with input images")
   end
   if(xyTensor:size(2)~=20) then
      error("wrong tensor size, should be nx2")
   end
   local xyTensor = xyTensor-1
   xyTensor = xyTensor:index(2,torch.LongTensor({  2,1,3,4, 
                                                   6,5,7,8,
                                                   10,9, 11, 12, 
                                                   14, 13, 15,16,
                                                   18,17, 19, 20})) --flip for opencv
   --local lastDigit = (xyTensor*10):floor()-(xyTensor:clone():floor()*10)
   --xyTensor[torch.ge(lastDigit,5)]:ceil()
   --xyTensor[torch.lt(lastDigit,5)]:floor()
   --xyYensor = xyTensor:int()
   local resultD = opencv.Mat.new(torch.ones(img:size(1), img:size(2)):fill(-1))  
   opencv_ffi.fillQuadAllWithInterpolation(img.mat,resultD.mat, opencv.Mat.new(xyTensor).mat)
   return img, resultD
end

function resize(img, factor)
   if ((not img.mat) or (type(img.mat) ~= "cdata")) then 
      error("problem with input images")
   end
   dest = opencv.Mat.new(torch.zeros(img:size(1)*factor, img:size(2)*factor))  
   opencv_ffi.resize(img.mat, dest.mat, factor)
   return dest
end

function DFT(img)
   if ((not img.mat) or (type(img.mat) ~= "cdata")) then 
      error("problem with input images")
   end
   dest = opencv.Mat.new(opencv_ffi.DFT(img.mat))
   return dest
end

function threshold(img)
   if ((not img.mat) or (type(img.mat) ~= "cdata")) then 
      error("problem with input images")
   end
   dst = opencv.Mat.new(torch.zeros(img:size(1), img:size(2)))  
   opencv_ffi.threshold(img.mat, dest.mat)
   return dest
end

function rotateImage(img, deg, centerX, centerY, size_x, size_y)
   if ((not img.mat) or (type(img.mat) ~= "cdata")) then 
      error("problem with input images")
   end
   dst = opencv.Mat.new(torch.zeros(size_x, size_y))  
   print(centerX-1, centerY-1)
   opencv_ffi.rotateImage(img.mat, dest.mat, deg, centerY-1, centerX-1, size_x, size_y)
   return dest
end

function inpaint(img, mask, radius, method)
   if ((not img.mat) or (type(img.mat) ~= "cdata")) then 
      error("problem with input images")
   end
   if ((not mask.mat) or (type(mask.mat) ~= "cdata")) then 
      error("problem with mask images")
   end
   dst = opencv.Mat.new(torch.zeros(img:size(1), img:size(2)))  
   opencv_ffi.inpaint(img.mat, mask.mat, dest.mat, radius or 3, method or true)
   return dest
end
