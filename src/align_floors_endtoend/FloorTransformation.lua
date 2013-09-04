FloorTransformation = Class()

colors = require '../opencv/types/Colors.lua'
Homography = geom.Homography
homography_funs = geom.rotation_translation_homography

FloorTransformation.parameters = {}
FloorTransformation.parameters.erosion_size =1
FloorTransformation.parameters.maxNumReturn = 50
FloorTransformation.parameters.warpWithBorders = false
FloorTransformation.parameters.cornerDistanceLimit = 25
FloorTransformation.parameters.corr_thresh = 2
FloorTransformation.parameters.minInliersForMatch = 2


harrisParameters = {}
harrisParameters.threshold = 50 --anything w score > .1*max score will be kept
harrisParameters.blockSize =5
harrisParameters.kSize =3
harrisParameters.k = .04
harrisParameters.npts_interest = 25
harrisParameters.radius_local_max = 5;
FloorTransformation.parameters.Harris = harrisParameters


--HOUGH PARAMETERS
HoughParameters = {}
HoughParameters.houghRo = 1
HoughParameters.houghTheta = math.pi/360
HoughParameters.houghMinLineLength = 25
HoughParameters.houghMaxLineGap = 80
HoughParameters.minThreshold = 10
HoughParameters.maxThreshold = 150
HoughParameters.defaultThreshold = 25
HoughParameters.numLinesDesired = 10

FloorTransformation.parameters.HoughParameters = HoughParameters

FloorTransformation.parameters.useHough = false

local GeneralInterestPointsParameters={}
GeneralInterestPointsParameters.dtype = "SIFT"
GeneralInterestPointsParameters.npts_interest = 150

FloorTransformation.parameters.GeneralInterestPointsParameters = GeneralInterestPointsParameters

--initialize based on parameters
local parameterizedHough = align_floors_endtoend.Hough.new(FloorTransformation.parameters.HoughParameters)
local parameterizedHarris = align_floors_endtoend.cornerHarris.new(FloorTransformation.parameters.Harris)
local parameterizedGeneral = align_floors_endtoend.GeneralInterestPoints.new(FloorTransformation.parameters.GeneralInterestPointsParameters.Harris)

function FloorTransformation.findTransformationOurs(image1Path, image2Path, display)
   log.tic()

   local img_src = FloorTransformation.imagePreProcessing(image1Path)
   local img_dest = FloorTransformation.imagePreProcessing(image2Path)
   local scores_src_torch = parameterizedHarris:findCorners(img_src)
   local scores_dest_torch = parameterizedHarris:findCorners(img_dest)

   scores_src_torch = scores_src_torch[{{}, {1,2}}]
   scores_dest_torch = scores_dest_torch[{{}, {1,2}}]
   
   print(scores_src_torch:size(1))
   print(scores_dest_torch:size(1))

   --example of how to do sift points:
   --src_corners, dest_corners = parameterizedGeneral:getPoints(img_src, img_dest)   
   
   if(FloorTransformation.parameters.useHough) then
      local locations_hough_source = parameterizedHough:getHoughLinesAndPoints(img_src)--FloorTransformation.getHoughLineIntersects(img_src)
      local locations_hough_dest = parameterizedHough:getHoughLinesAndPoints(img_dest)--FloorTransformation.getHoughLineIntersects(img_dest)

	  if locations_hough_source then
	      scores_src_torch = torch.cat(scores_src_torch, locations_hough_source,1)
	  end  
	  if locations_hough_dest then
	      scores_dest_torch = torch.cat(scores_dest_torch, locations_hough_dest,1)
	  end
      
      print(scores_src_torch:size(1))
   print(scores_dest_torch:size(1))
   end
   scores_src = opencv.Mat.new(scores_src_torch:clone())
   scores_dest = opencv.Mat.new(scores_dest_torch:clone())

   if(display) then
   	
   	--[[]]
      imgsizex = img_src:size(1)
      imgsizey = img_src:size(2)
   	  local indd = torch.range(1,scores_src_torch:size(1))
   	  indd:apply(function(i)
   	    local scoresi = scores_src_torch[i]
   	    local sx = scoresi[1]
   	    local sy = scoresi[2]
   	    if sx < 1 or sy < 1 or sx > imgsizex or sy > imgsizey then
   	    	scores_src_torch[i][1] = 1
   	    	scores_src_torch[i][2] = 1
   	    end
   	  end)
      image.displayPoints(img_src:toTensor(), scores_src_torch, colors.MAGENTA, 2)
      --[[]]
      
      --[[]]
      imgsizex = img_dest:size(1)
      imgsizey = img_dest:size(2)
   	  local indd = torch.range(1,scores_dest_torch:size(1))
   	  indd:apply(function(i)
   	    local scoresi = scores_dest_torch[i]
   	    local sx = scoresi[1]
   	    local sy = scoresi[2]
   	    if sx < 1 or sy < 1 or sx > imgsizex or sy > imgsizey then
   	    	scores_dest_torch[i][1] = 1
   	    	scores_dest_torch[i][2] = 1
   	    end
   	  end)
      image.displayPoints(img_dest:toTensor(), scores_dest_torch, colors.CYAN, 2)
      --[[]]
      
      return 0
   end
   --pairwise distance between corner points
   --we can then threshold this, to make sure we only look at pairs which are not too close together
   local pairwise_dis_src = geom.util.pairwise_distance(scores_src_torch, scores_src_torch)
   local pairwise_dis_dest = geom.util.pairwise_distance(scores_dest_torch, scores_dest_torch)

   goodLocationsX_src, goodLocationsY_src = image.thresholdReturnCoordinates(pairwise_dis_src,2 *FloorTransformation.parameters.corr_thresh)
   goodLocationsX_dest, goodLocationsY_dest = image.thresholdReturnCoordinates(pairwise_dis_dest,2 *FloorTransformation.parameters.corr_thresh)

   goodLocationsX_src = goodLocationsX_src:reshape(goodLocationsX_src:size()[1],1)
   goodLocationsY_src = goodLocationsY_src:reshape(goodLocationsY_src:size()[1],1)
   goodLocationsX_dest = goodLocationsX_dest:reshape(goodLocationsX_dest:size()[1],1)
   goodLocationsY_dest = goodLocationsY_dest:reshape(goodLocationsY_dest:size()[1],1)

      print(pairwise_dis_src:size())

   local best_pts, best_transformations = opencv.imgproc.findBestTransformation(
      opencv.Mat.new(goodLocationsX_src:clone()),  opencv.Mat.new(goodLocationsY_src:clone()), opencv.Mat.new(scores_src_torch), opencv.Mat.new(pairwise_dis_src:clone()),
      opencv.Mat.new(goodLocationsX_dest:clone()), opencv.Mat.new(goodLocationsY_dest:clone()), opencv.Mat.new(scores_dest_torch), opencv.Mat.new(pairwise_dis_dest:clone()),
      FloorTransformation.parameters.corr_thresh, FloorTransformation.parameters.minInliersForMatch, FloorTransformation.parameters.maxNumReturn, 
      FloorTransformation.parameters.cornerDistanceLimit, img_src:size()[1], img_src:size()[2])

   local mainDirections = FloorTransformation.findMainDirections(img_src, img_dest)

   local trans1 = {}
   local trans2 = {}
   local combined = {}
   local inliers = {}
   local transformations = {}
   local src_centers_h = {}
   local src_centers_w = {}
   local tgt_centers_h = {}
   local tgt_centers_w = {}
   local size_x_all = {}
   local size_y_all = {}
   
   local anglediff = torch.zeros(table.getn(best_transformations))
  
   sorted,ordering = torch.sort(best_pts)
   for k=1,table.getn(best_transformations) do
      i=table.getn(best_transformations)-k+1
      inliers[k] = sorted[i]
      local trans1_i, trans2_i, combined_i
      if FloorTransformation.parameters.warpWithBorders then
         trans1_i, trans2_i, combined_i = image.warpAndCombineWithBorders(best_transformations[ordering[i]], img_src, img_dest)
      else
         trans1_i, trans2_i, combined_i = image.warpAndCombine(best_transformations[ordering[i]], img_src, img_dest)
      end
      
      local center1y = img_src:size(1)/2
      local center1x = img_src:size(2)/2
      
      local center2y = img_dest:size(1)/2
      local center2x = img_dest:size(2)/2
      
      local t1 = opencv.Mat.toTensor(trans1_i)
      local t2 = opencv.Mat.toTensor(trans2_i)
      
      local center1 = t1*torch.Tensor({{center1x},{center1y},{1}})+1
      local center2 = t2*torch.Tensor({{center2x},{center2y},{1}})+1
      
      center1x = math.floor(center1[1][1])
      center1y = math.floor(center1[2][1])
      center2x = math.floor(center2[1][1])
      center2y = math.floor(center2[2][1])
     
      src_centers_h[k] = center1y
      src_centers_w[k] = center1x
      tgt_centers_h[k] = center2y
      tgt_centers_w[k] = center2x

      size_x_all[k] = img_src:size(1)
      size_y_all[k] = img_src:size(2)

      transformations[k] = best_transformations[ordering[i]]
      trans1[k] = trans1_i
      trans2[k] = trans2_i
      combined[k] = combined_i
      --the closer to zero the better
      anglediff[k] = FloorTransformation.isAlignedWithMainDirection(mainDirections, best_transformations[ordering[i]].H)
   end
   print(log.toc())
   return transformations, trans1, trans2, combined, inliers, anglediff, src_centers_h, src_centers_w, tgt_centers_h, tgt_centers_w, size_x_all, size_y_all
end

--intersect/union
function FloorTransformation.scoreTransformationPair(H1, H2, size_x, size_y)
   H1 = geom.Homography.new(H1)
   H2 = geom.Homography.new(H2)
   
   a = opencv.Mat.new(torch.ones(size_x, size_y))

   t1, t2, t3, first, t5 = image.warpAndCombine(H1, a, a)
   t1, t2, t3, warped1, warped2 = image.warpAndCombine(H2, a, first)
   warped1 = warped1:toTensor()
   warped2 = warped2:toTensor()
   return torch.sum(torch.eq(warped1+warped2, 2))/torch.sum(torch.ge(warped1+warped2, 1))
end

--returns an opencv mat with the image with the following done to it:
--loaded in opencv 
--converted to grayscale
--dilated and eroded to get rid of random noise/small things
function FloorTransformation.imagePreProcessing(imagePath)
   local img = opencv.Mat.new(imagePath)
   img:convert("RGB2GRAY");
   structElement = opencv.imgproc.getDefaultStructuringMat(FloorTransformation.parameters.erosion_size) 
   opencv.imgproc.dilate(img, structElement);
   opencv.imgproc.erode(img, structElement);
   return img
end

function FloorTransformation.findMainDirections(img_src, img_dest)
   Hough = align_floors_endtoend.Hough
   local domAngle1_src,domAngle2_src = parameterizedHough:getMainDirections(img_src)
   local domAngle1_dest,domAngle2_dest = parameterizedHough:getMainDirections(img_dest)

   local angle_options = torch.Tensor({domAngle1_dest-domAngle1_src,domAngle2_dest-domAngle2_src, domAngle1_dest-domAngle2_src, domAngle2_dest-domAngle1_src})
   angle_options = torch.cat(angle_options, angle_options+math.pi, 1)
   angle_options:apply(function(v) if(v<0) then return 2*math.pi+v elseif (v>2*math.pi) then return v-2*math.pi else return v end end);
   angle_options = torch.sort(angle_options)

   if torch.abs(2*math.pi+angle_options[1] - angle_options[angle_options:size()[1]])<2*math.pi/360*20 then
   
      angle_options = torch.cat( angle_options[{{2,angle_options:size()[1]},1}],-angle_options[{{1}}]+2*math.pi, 1)
   end

   local angle_options_real = torch.Tensor({angle_options[1]+angle_options[2], angle_options[3]+angle_options[4],
      angle_options[5]+angle_options[6],angle_options[7]+angle_options[8]})/2
   return angle_options_real
end

function FloorTransformation.isAlignedWithMainDirection(angle_options_real, H)
   if (H[1][1]) > 1 then
      theta1 = 0
   elseif H[1][1] < -1 then
      theta1 = math.pi
   else
      theta1 = torch.acos(H[1][1])
   end
   if (H[1][2]) > 1 then
      theta2= -math.pi/2
   elseif H[1][2] < -1 then
      theta2 = math.pi/2
   else
      theta2 = -torch.asin(H[1][2])
   end
   theta = (theta1+theta2)/2
    --if both are positive, we are in upper right and correct
    --if theta1 positive, and theta2 negative, should be negative
    if(theta1<=math.pi/2 and theta2<=0) then
      theta = 2*math.pi+theta2
    elseif (theta1<=math.pi/2 and theta2 > 0) then
      theta = theta1
    elseif (theta1 > math.pi/2 and theta2 <= 0) then
      theta = 2*math.pi+theta2
    elseif (theta1 > math.pi/2 and theta2 >0) then
      theta = theta1
    end
    return math.min(torch.min(torch.abs(angle_options_real-theta)), torch.min(torch.abs(-angle_options_real+theta+2*math.pi)))
end

function FloorTransformation.getParameterString(prefix, parameters)
   toReturn = ""
   if type(parameters)=="table" then
      for i in pairs(parameters) do
            if type(parameters[i])=="table" then
               toReturn = toReturn ..  getParameterString(prefix .. i .. "." , parameters[i])
            else
               toReturn = toReturn .. prefix .. i .. '=' .. string.format("%s",parameters[i]) .. "\n"
            end
      end
   end
   return toReturn 
end


