ffi = require 'ffi'
libopencv = util.ffi.load("libopencv")
Mat    = require './Mat'

ctorch = util.ctorch

ffi.cdef [[
Mat* warpImage(const Mat* src, const Mat* transform, int size_x, int size_y);
Mat* CannyDetectEdges(Mat* src, double threshold1, double threshold2);
Mat* HoughLinesRegular(Mat* image, double rho, double theta, int threshold, double srn, double stn);
Mat* HoughLinesProbabilistic(Mat* image, double rho, double theta, int threshold, double minLineLength, double maxLineGap);

Mat* getStructuringElement(int type, int size_x, int size_y, int center_x, int center_y);
void dilate(Mat*  src, Mat* structuringElement);
void erode(Mat*  src, Mat* structuringElement);

Mat* detectCornerHarris(Mat* src, int blockSize, int ksize, int k);

]]

local function destructor ()
   return function (mat)
      libopencv.Mat_destroy(mat)
   end
end

morph_types = require './types/Morph'

-- not a "Class()" no self, just a bunch of functions in a namespace.
imgproc = {}
imgproc.colors = require './types/Colors.lua'

-- <input> image, homography
-- <output> image warped by homography
function imgproc.warpImage(img, homography, size_x, size_y)
   if ((not img.mat) or (type(img.mat) ~= "cdata")) then 
      error("problem with input images")
   end
   if ((not homography.mat) or (type(homography.mat) ~= "cdata")) then 
      error("problem with homography matrix")
   end

   return Mat.new(libopencv.warpImage(img.mat, homography.mat, size_x, size_y))
end

-- <input> image1, image2, homography
-- <output> combined images -1st channel is warped and translated image
function imgproc.combineImages(img1, img2, homography)
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
   --return Mat.new(libopencv.combineImages(img1.mat, img2.mat, homography.mat, result_size_x, result_size_y, result_center_x, result_center_y))
end

-- <input> img, blockSize, ksize, k
-- <output> Mat of responses at every location
function imgproc.detectCornerHarris(img,blockSize,ksize, k)
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
    return Mat.new(libopencv.detectCornerHarris(img.mat, blockSize, ksize, k))
end

-- <input> image, threshold1, threshold2 -- see opencv function
-- <output> line mat
function imgproc.CannyDetectEdges(img, threshold1, threshold2)
   if ((not img.mat) or (type(img.mat) ~= "cdata")) then 
      error("problem with input image")
   end
   if type(threshold1) ~= "number" then
      error("need to pass number for second argument")
   end
   if type(threshold2) ~= "number" then
      error("need to pass number for third argument")
   end

   return Mat.new(libopencv.CannyDetectEdges(img.mat, threshold1, threshold2))
end

-- <input> -- see opencv function
-- <output> line mat
function imgproc.HoughLinesRegular(img, rho, theta, threshold, srn, stn)
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
   return Mat.new(libopencv.HoughLinesRegular(img.mat, rho, theta, threshold, srn, stn))
end

-- <input> -- see opencv function
-- <output> line mat
function imgproc.HoughLinesProbabilistic(img, rho, theta, threshold, mineLineLength, maxLineGap)
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
   return Mat.new(libopencv.HoughLinesProbabilistic(img.mat, rho, theta, threshold, mineLineLength, maxLineGap))
end


--DILATION AND EROSION ---
-- <input> -- see opencv function
-- <output> line mat
function imgproc.getStructuringElement(structType, size_x, size_y, center_x, center_y)
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
   return Mat.new(libopencv.getStructuringElement(structType, size_x, size_y, center_x, center_y))
end

function imgproc.getDefaultStructuringMat(erosion_size)
   if type(erosion_size) ~= "number" then
      error("need to pass number for first argument")
   end
   return imgproc.getStructuringElement(morph_types.MORPH_RECT, 2*erosion_size+1, 2*erosion_size+1, erosion_size, erosion_size);
end

--dilate the img 
function imgproc.dilate(img, structuringElement)
   if ((not img.mat) or (type(img.mat) ~= "cdata")) then 
      error("problem with input images")
   end
   if ((not structuringElement.mat) or (type(structuringElement.mat) ~= "cdata")) then 
      error("problem with structuring element images")
   end
   libopencv.dilate(img.mat, structuringElement.mat);
end

--erode the img 
function imgproc.erode(img, structuringElement)
   if ((not img.mat) or (type(img.mat) ~= "cdata")) then 
      error("problem with input images")
   end
   if ((not structuringElement.mat) or (type(structuringElement.mat) ~= "cdata")) then 
      error("problem with structuring element images")
   end
   libopencv.erode(img.mat, structuringElement.mat);
end

----- DISPLAY FUNCTIONS -------
function imgproc.displayGrayMatInLua(cv_mat)
   image.display(imgproc.get3DTensorfrom1DMat(cv_mat))
end

function imgproc.displayHarrisPoint(cv_mat, disp_color, point_x, point_y, radius)
   color_tensor = imgproc.get3DTensorfrom1DMat(cv_mat) 
   imgproc.addPointToTensor(color_tensor, disp_color, point_x, point_y, radius)
   image.display(color_tensor)
end

--cv_mat is grayscale image, points_mat is x,y location of points
--disp_color is defined as in types/Colors.lua
--displays square points for laziness
function imgproc.displayPoints(img, points_mat, disp_color, radius)
   color_tensor = imgproc.get3DTensorfrom1DMatCopyChannels(img)
   for i=1, points_mat:size(1) do
      imgproc.addPointToTensor(color_tensor, disp_color, points_mat[i][1], points_mat[i][2], radius)
   end
   image.display(color_tensor)
end

function imgproc.get3DTensorfrom1DMat(cvMat)
   mat_1D = cvMat
   if type(mat_1D) == "cdata" then
      tensor_th = Mat.new(mat_1D):toTensor()
   else
      mat_1D = cvMat.mat
      if type(mat_1D) ~= "cdata" then 
         error("need to pass Matat object for argument")
      end
      tensor_th = cvMat:toTensor()
   end
   h = tensor_th:size(1)
   w = tensor_th:size(2)
   -- no copying, or duplicating data just tricks with strides
   tensor_th = tensor_th:resize(1,h,w):expand(3,h,w)
   return tensor_th
end

function imgproc.get3DTensorfrom1DMatCopyChannels(cvMat)
   mat_1D = cvMat
   if type(mat_1D) == "cdata" then
      tensor_th = Mat.new(mat_1D):toTensor()
   else
      mat_1D = cvMat.mat
      if type(mat_1D) ~= "cdata" then 
         error("need to pass Matat object for argument")
      end
      tensor_th = cvMat:toTensor()
   end
   h = tensor_th:size(1)
   w = tensor_th:size(2)
   -- no copying, or duplicating data just tricks with strides
   tensor_th = tensor_th:repeatTensor(3,1,1)
   return tensor_th
end

--helper function, changes the color of the squares of raidus centered at point_x,point_y to color_tensor
function imgproc.addPointToTensor(color_tensor, disp_color, point_x, point_y, radius)
   for j=1,3 do
      color_tensor[{j,
      {math.max(point_x-radius,1), math.min(point_x+radius, color_tensor:size(2))},
      {math.max(point_y-radius,1), math.min(point_y+radius, color_tensor:size(3))}}]
      =disp_color[j]
   end
end

return imgproc
