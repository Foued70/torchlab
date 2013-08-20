ffi = require 'ffi'
libopencv = util.ffi.load("libopencv")
Mat    = require './Mat'

ctorch = util.ctorch

ffi.cdef [[
Mat* getHomography(const KeyPoint* keyptr_src, int npts_src, const KeyPoint* keyptr_dest, int npts_dest, const DMatch* matchptr, int npts_match);
Mat* warpImage(const Mat* src, const Mat* transform);
Mat* combineImages(const Mat* src, 
                                     const Mat* dest, 
                                     const Mat* transform, 
                                     int result_size_x, 
                                     int result_size_y, 
                                     int result_center_x, 
                                     int result_center_y);

Mat* CannyDetectEdges(Mat* src, double threshold1, double threshold2);
Mat* HoughLinesRegular(Mat* image, double rho, double theta, int threshold, double srn, double stn);
Mat* HoughLinesProbabilistic(Mat* image, double rho, double theta, int threshold, double minLineLength, double maxLineGap);

Mat* getStructuringElement(int type, int size_x, int size_y, int center_x, int center_y);
void dilate(Mat*  src, Mat* structuringElement);
void erode(Mat*  src, Mat* structuringElement);

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
function imgproc.warpImage(img, homography)
   if type(img) ~= "cdata" then
      error("need to pass opencv mat object for first argument")
   end
   if type(homography) ~= "cdata" then
      error("need to pass opencv mat object for second argument")
   end

   return Mat.new(libopencv.warpImage(img, homography))
end

-- <input> image1, image2, homography, 
-- <output> image warped by homography
function imgproc.combineImages(img1, img2, homography, result_size_x, result_size_y, result_center_x, result_center_y)
   if type(img1) ~= "cdata" then
      error("need to pass opencv mat object for first argument")
   end
   if type(img2) ~= "cdata" then
      error("need to pass opencv mat object for sec argument")
   end
   if type(homography) ~= "cdata" then
      error("need to pass opencv mat object for third argument")
   end
   if type(result_size_x) ~= "number" then
      error("need to pass opencv mat object for fourth argument")
   end
   if type(result_size_y) ~= "number" then
      error("need to pass opencv mat object for fifth argument")
   end
   if type(result_center_x) ~= "number" then
      error("need to pass opencv mat object for sixth argument")
   end
   if type(result_center_y) ~= "number" then
      error("need to pass opencv mat object for seventh argument")
   end

   return Mat.new(libopencv.combineImages(img1, img2, homography, result_size_x, result_size_y, result_center_x, result_center_y))
end
-- <input> image, threshold1, threshold2 -- see opencv function
-- <output> line mat
function imgproc.CannyDetectEdges(img, threshold1, threshold2)
   if type(img) ~= "cdata" then
      error("need to pass opencv mat object for first argument")
   end
   if type(threshold1) ~= "number" then
      error("need to pass number for second argument")
   end
   if type(threshold2) ~= "number" then
      error("need to pass number for third argument")
   end

   return Mat.new(libopencv.CannyDetectEdges(img, threshold1, threshold2))
end

-- <input> -- see opencv function
-- <output> line mat
function imgproc.HoughLinesRegular(img, rho, theta, threshold, srn, stn)
   if type(img) ~= "cdata" then
      error("need to pass opencv mat object for first argument")
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
   return Mat.new(libopencv.HoughLinesRegular(img, rho, theta, threshold, srn, stn))
end

-- <input> -- see opencv function
-- <output> line mat
function imgproc.HoughLinesProbabilistic(img, rho, theta, threshold, mineLineLength, maxLineGap)
   if type(img) ~= "cdata" then
      error("need to pass opencv mat object for first argument")
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
   return Mat.new(libopencv.HoughLinesProbabilistic(img, rho, theta, threshold, mineLineLength, maxLineGap))
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
   if type(img) ~= "cdata" then
      error("need to pass opencv mat object for first argument")
   end
   if type(structuringElement) ~= "cdata" then
      error("need to pass opencv mat object for first argument")
   end
   libopencv.dilate(img, structuringElement);
end

--erode the img 
function imgproc.erode(img, structuringElement)
   if type(img) ~= "cdata" then
      error("need to pass opencv mat object for first argument")
   end
   if type(structuringElement) ~= "cdata" then
      error("need to pass opencv mat object for first argument")
   end
   libopencv.erode(img, structuringElement);
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

--cv_mat is grayscale image, cv_score_mat is tuples of
--disp_color is defined as in types/Colors.lua
--displays square points for laziness
function imgproc.displayHarris(cv_mat, score_mat_torch, disp_color, radius)
   color_tensor = imgproc.get3DTensorfrom1DMat(cv_mat) 
   drawn_t = imgproc.opencv.MattoTensor(cv_mat)
   for i=1, score_mat_torch:size(1) do
      imgproc.addPointToTensor(color_tensor, disp_color, score_mat_torch[i][1], score_mat_torch[i][2], radius)
   end
   image.display(color_tensor)
end

function imgproc.get3DTensorfrom1DMat(cv_mat)
   mat_1D = cv_mat
   if type(mat_1D) == "cdata" then
      tensor_th = Mat.new(mat_1D):toTensor()
   else
      mat_1D = cv_mat.mat
      if type(mat_1D) ~= "cdata" then 
         error("need to pass Matat object for argument")
      end
      tensor_th = cv_mat:toTensor()
   end
   h = tensor_th:size(1)
   w = tensor_th:size(2)
   -- no copying, or duplicating data just tricks with strides
   tensor_th = tensor_th:resize(1,h,w):expand(3,h,w)
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
