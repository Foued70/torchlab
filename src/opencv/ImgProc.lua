ffi = require 'ffi'
libopencv = util.ffi.load("libopencv")
Mat    = require './Mat'

ctorch = util.ctorch

ffi.cdef [[
void Mat_convert (Mat* input, Mat* output, int cvttype);

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

conversion = require './types/Mat_conversion'
morph_types = require './types/Morph'

-- not a "Class()" no self, just a bunch of functions in a namespace.
ImgProc = {}
ImgProc.colors = require './types/Colors.lua'

--type_str should be a string, not a number!
function ImgProc.convert(...) 
   local input, output, type_str
   args = {...}
   nargs = #args
   if nargs == 2 then 
      input     = args[1]
      output    = input:clone() 
      type_str  = args[2]
   elseif nargs == 3 then 
      output    = args[1]
      input     = args[2]
      type_str  = args[3]
   else
      error("wrong number of args" ..#args)
   end
   if type(type_str) ~= "string" then
      error("need to pass string as type of conversion")
   end
   type_enum  = conversion[type_str]
   input_mat  = input.mat
   output_mat = output.mat
   if type_enum and input_mat and output_mat then 
      -- print("converting ".. type_str .. " " .. type_enum)
      libopencv.Mat_convert(input_mat,output_mat,type_enum)
      return output
   else
      error("Don't understand conversion type "..type_str)
   end 
   return output
end

-- <input> matcher, source descriptor matrix, dest descriptor matrix, nmatches max
-- <output> matches and number of matches found
function ImgProc.getHomography(keypoints_src, npts_src, keypoints_dest, npts_dest, matches, nmatches)
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
   return Mat.new(ffi.gc(libopencv.getHomography(keypoints_src, npts_src, keypoints_dest, npts_dest, matches, nmatches), destructor()))
end

-- <input> image, homography
-- <output> image warped by homography
function ImgProc.warpImage(img, homography)
   if type(img) ~= "cdata" then
      error("need to pass opencv mat object for first argument")
   end
   if type(homography) ~= "cdata" then
      error("need to pass opencv mat object for second argument")
   end

   return Mat.new(ffi.gc(libopencv.warpImage(img, homography), destructor()))
end

-- <input> image1, image2, homography, 
-- <output> image warped by homography
function ImgProc.combineImages(img1, img2, homography, result_size_x, result_size_y, result_center_x, result_center_y)
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

   return Mat.new(ffi.gc(libopencv.combineImages(img1, img2, homography, result_size_x, result_size_y, result_center_x, result_center_y), destructor()))
end
-- <input> image, threshold1, threshold2 -- see opencv function
-- <output> line mat
function ImgProc.CannyDetectEdges(img, threshold1, threshold2)
   if type(img) ~= "cdata" then
      error("need to pass opencv mat object for first argument")
   end
   if type(threshold1) ~= "number" then
      error("need to pass number for second argument")
   end
   if type(threshold2) ~= "number" then
      error("need to pass number for third argument")
   end

   return Mat.new(ffi.gc(libopencv.CannyDetectEdges(img, threshold1, threshold2), destructor()))
end

-- <input> -- see opencv function
-- <output> line mat
function ImgProc.HoughLinesRegular(img, rho, theta, threshold, srn, stn)
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
   return Mat.new(ffi.gc(libopencv.HoughLinesRegular(img, rho, theta, threshold, srn, stn), destructor()))
end

-- <input> -- see opencv function
-- <output> line mat
function ImgProc.HoughLinesRegular(img, rho, theta, threshold, mineLineLength, maxLineGap)
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
   return Mat.new(ffi.gc(libopencv.HoughLinesProbabilistic(img, rho, theta, threshold, mineLineLength, maxLineGap), destructor()))
end


--DILATION AND EROSION ---
-- <input> -- see opencv function
-- <output> line mat
function ImgProc.getStructuringElement(structType, size_x, size_y, center_x, center_y)
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
   return Mat.new(ffi.gc(libopencv.getStructuringElement(structType, size_x, size_y, center_x, center_y), destructor()))
end

function ImgProc.getDefaultStructuringMat(erosion_size)
   if type(erosion_size) ~= "number" then
      error("need to pass number for first argument")
   end
   return ImgProc.getStructuringElement(morph_types.MORPH_RECT, 2*erosion_size+1, 2*erosion_size+1, erosion_size, erosion_size);
end

--dilate the img 
function ImgProc.dilate(img, structuringElement)
   if type(img) ~= "cdata" then
      error("need to pass opencv mat object for first argument")
   end
   if type(structuringElement) ~= "cdata" then
      error("need to pass opencv mat object for first argument")
   end
   libopencv.dilate(img, structuringElement);
end

--erode the img 
function ImgProc.erode(img, structuringElement)
   if type(img) ~= "cdata" then
      error("need to pass opencv mat object for first argument")
   end
   if type(structuringElement) ~= "cdata" then
      error("need to pass opencv mat object for first argument")
   end
   libopencv.erode(img, structuringElement);
end

----- DISPLAY FUNCTIONS -------
function ImgProc.displayGrayMatInLua(cv_mat)
   image.display(ImgProc.get3DTensorfrom1DMat(cv_mat))
end

function ImgProc.displayHarrisPoint(cv_mat, disp_color, point_x, point_y, radius)
   color_tensor = ImgProc.get3DTensorfrom1DMat(cv_mat) 
   ImgProc.addPointToTensor(color_tensor, disp_color, point_x, point_y, radius)
   image.display(color_tensor)
end

--cv_mat is grayscale image, cv_score_mat is tuples of
--disp_color is defined as in types/Colors.lua
--displays square points for laziness
function ImgProc.displayHarris(cv_mat, score_mat_torch, disp_color, radius)
   color_tensor = ImgProc.get3DTensorfrom1DMat(cv_mat) 
   drawn_t = ImgProc.opencv.MattoTensor(cv_mat)
   for i=1, score_mat_torch:size(1) do
      ImgProc.addPointToTensor(color_tensor, disp_color, score_mat_torch[i][1], score_mat_torch[i][2], radius)
   end
   image.display(color_tensor)
end

function ImgProc.get3DTensorfrom1DMat(cv_mat)
   if type(cv_mat) ~= "cdata" then
      error("need to pass opencv mat object for argument")
   end
   cv_mat_3d = ImgProc.convert(Mat.new(cv_mat), "GRAY2RGB")
   drawn_t = cv_mat_3d:toTensor()
   drawn_t = drawn_t:transpose(3,1):transpose(2,3)
   return drawn_t
end

--helper function, changes the color of the squares of raidus centered at point_x,point_y to color_tensor
function ImgProc.addPointToTensor(color_tensor, disp_color, point_x, point_y, radius)
   for j=1,3 do
      color_tensor[{j,
      {math.max(point_x-radius,1), math.min(point_x+radius, color_tensor:size(2))},
      {math.max(point_y-radius,1), math.min(point_y+radius, color_tensor:size(3))}}]
      =disp_color[j]
   end
end

return ImgProc
