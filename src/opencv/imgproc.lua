libopencv = require './libopencv'

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

   return opencv.Mat.new(libopencv.warpImage(img.mat, homography.mat, size_x, size_y))
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
   --return opencv.Mat.new(libopencv.combineImages(img1.mat, img2.mat, homography.mat, result_size_x, result_size_y, result_center_x, result_center_y))
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
    return opencv.Mat.new(libopencv.detectCornerHarris(img.mat, blockSize, ksize, k))
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

   return opencv.Mat.new(libopencv.CannyDetectEdges(img.mat, threshold1, threshold2))
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
   return opencv.Mat.new(libopencv.HoughLinesRegular(img.mat, rho, theta, threshold, srn, stn))
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
   return opencv.Mat.new(libopencv.HoughLinesProbabilistic(img.mat, rho, theta, threshold, mineLineLength, maxLineGap))
end

function getPairwiseDistances(A, B)
   if ((not A.mat) or (type(A.mat) ~= "cdata")) then 
      error("problem with A Mat - first argument")
   end
   if ((not B.mat) or (type(B.mat) ~= "cdata")) then 
      error("problem with B Mat -- second argument")
   end
   return opencv.Mat.new(libopencv.getPairwiseDistances(A.mat,B.mat))
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
   return opencv.Mat.new(libopencv.getStructuringElement(structType, size_x, size_y, center_x, center_y))
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
   libopencv.dilate(img.mat, structuringElement.mat);
end

--erode the img 
function erode(img, structuringElement)
   if ((not img.mat) or (type(img.mat) ~= "cdata")) then 
      error("problem with input images")
   end
   if ((not structuringElement.mat) or (type(structuringElement.mat) ~= "cdata")) then 
      error("problem with structuring element images")
   end
   libopencv.erode(img.mat, structuringElement.mat);
end
