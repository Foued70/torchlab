FloorTransformation = Class()

colors = require '../opencv/types/Colors.lua'
Homography = geom.Homography
homography_funs = geom.rotation_translation_homography

FloorTransformation.parameters = {}
FloorTransformation.parameters.erosion_size =1
FloorTransformation.parameters.maxNumCompute = 50
FloorTransformation.parameters.maxNumReturn = 25
FloorTransformation.parameters.warpWithBorders = false
FloorTransformation.parameters.cornerDistanceLimit = 1
FloorTransformation.parameters.corr_thresh = 2
FloorTransformation.parameters.minInliersForMatch = 2
FloorTransformation.parameters.rotation_thresh = 2*math.pi * (4.5/360)  --should be 0 to 45 (0 means less results, 45 means all)

harrisParameters = {}
harrisParameters.threshold = 75 --anything w score > .1*max score will be kept
harrisParameters.blockSize =5
harrisParameters.kSize =3
harrisParameters.k = .04
harrisParameters.npts_interest = 50
harrisParameters.radius_local_max = 10;
FloorTransformation.parameters.Harris = harrisParameters


--HOUGH PARAMETERS
HoughParameters = {}
HoughParameters.houghRo = 1
HoughParameters.houghTheta = math.pi/360
HoughParameters.houghMinLineLength = 25
HoughParameters.houghMaxLineGap = 80
HoughParameters.minThreshold = 10
HoughParameters.maxThreshold = 150
HoughParameters.defaultThreshold = 75
HoughParameters.numLinesDesired = 15

FloorTransformation.parameters.HoughParameters = HoughParameters

FloorTransformation.parameters.useHough = true

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
   
   local scores_src_torch
   local scores_dest_torc

   local img_src = FloorTransformation.imagePreProcessing(image1Path)
   local img_dest = FloorTransformation.imagePreProcessing(image2Path)
   
   local scores_src_torch_ha = parameterizedHarris:findCorners(img_src)
   local scores_dest_torch_ha = parameterizedHarris:findCorners(img_dest)
   
   	if scores_src_torch_ha then
	   scores_src_torch = scores_src_torch_ha[{{}, {1,2}}]
	end
	
	if scores_dest_torch_ha then
	   scores_dest_torch = scores_dest_torch_ha[{{}, {1,2}}]
	end
   
   local scores_src_torch_ho = parameterizedHough:getHoughCorners(img_src)
   local scores_dest_torch_ho = parameterizedHough:getHoughCorners(img_dest)
   
   	if scores_src_torch then
   		if scores_src_torch_ho then
	   		scores_src_torch = torch.cat(scores_src_torch,scores_src_torch_ho[{{}, {1,2}}],1)
	   	end
	else
		if scores_src_torch_ho then
		   scores_src_torch = scores_src_torch_ho[{{}, {1,2}}]
		end
	end
	
	if scores_dest_torch then
   		if scores_dest_torch_ho then
	   		scores_dest_torch = torch.cat(scores_dest_torch,scores_dest_torch_ho[{{}, {1,2}}],1)
	   	end
	else
	   if scores_dest_torch_ho then
	   		scores_dest_torch = scores_dest_torch_ho[{{}, {1,2}}]
	   	end
	end

   --example of how to do sift points:
   src_corners, dest_corners = parameterizedGeneral:getPoints(img_src, img_dest)   
   
   if(FloorTransformation.parameters.useHough) then
      local locations_hough_source = parameterizedHough:getHoughLinesAndPoints(img_src)
      local locations_hough_dest = parameterizedHough:getHoughLinesAndPoints(img_dest)

		if scores_src_torch then
	   		if locations_hough_source then
	      		scores_src_torch = torch.cat(scores_src_torch, locations_hough_source,1)
			end  
		else
	   		scores_src_torch = locations_hough_source
		end
	
		if scores_dest_torch then
	   		if locations_hough_dest then
	      		scores_dest_torch = torch.cat(scores_dest_torch, locations_hough_dest,1)
	  		end
		else
	   		scores_dest_torch = locations_hough_dest
		end
      
   end
   
   if scores_src_torch == nil or scores_dest_torch == nil then
   		log.error('no corner points found!')
   		return nil
   	end
   	
   	print(scores_src_torch:size(1))
   print(scores_dest_torch:size(1))
   	
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
      FloorTransformation.parameters.corr_thresh, FloorTransformation.parameters.minInliersForMatch, FloorTransformation.parameters.maxNumCompute, 
      FloorTransformation.parameters.cornerDistanceLimit, img_src:size()[1], img_src:size()[2])
      
      local mainDirections = FloorTransformation.findMainDirections(img_src, img_dest)
   
   --[[]]
   local trans1_tmp = {}
   local trans2_tmp = {}
   local combined_tmp = {}
   local src_centers_h_tmp = {}
   local src_centers_w_tmp = {}
   local tgt_centers_h_tmp = {}
   local tgt_centers_w_tmp = {}
   
   local size_x_all_tmp = {}
   local size_y_all_tmp = {}
   
   local anglediff_tmp = torch.zeros(table.getn(best_transformations))
   
   local best_overlap = torch.Tensor(best_pts:size())
   for k=1,table.getn(best_transformations) do
      local i=k
      local trans1_i, trans2_i, combined_i
      if FloorTransformation.parameters.warpWithBorders then
         trans1_i, trans2_i, combined_i = image.warpAndCombineWithBorders(best_transformations[i], img_src, img_dest)
      else
         trans1_i, trans2_i, combined_i = image.warpAndCombine(best_transformations[i], img_src, img_dest)
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
     
      src_centers_h_tmp[i] = center1y
      src_centers_w_tmp[i] = center1x
      tgt_centers_h_tmp[i] = center2y
      tgt_centers_w_tmp[i] = center2x
      
      trans1_tmp[i] = trans1_i
      trans2_tmp[i] = trans2_i
      combined_tmp[i] = combined_i
      
      size_x_all_tmp[i] = img_src:size(1)
      size_y_all_tmp[i] = img_src:size(2)
      
      --the closer to zero the better
	   anglediff_tmp[i] = FloorTransformation.findMainDirections(opencv.Mat.new(combined_tmp[i][1]:byte()), opencv.Mat.new(combined_tmp[i][2]:byte()))
   
      local srci = combined_i:clone():select(1,1)
      srci:cdiv(srci:clone():add(0.0000000001)):ceil()
      local desi = combined_i:clone():select(1,2)
      desi:cdiv(desi:clone():add(0.0000000001)):ceil()
	  best_overlap[i]=srci:clone():cmul(desi):sum()
	  
	  collectgarbage()
   end
    collectgarbage()
   
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
   
   sorted,ordering = torch.sort(best_overlap)
   
   local numRet = math.min(table.getn(best_transformations),FloorTransformation.parameters.maxNumReturn)
   local scores = torch.Tensor(numRet)
   
   local i = table.getn(best_transformations)
   local k = 0
   while k<numRet and i > 0 do
      
      local o = ordering[i]
      
      if anglediff_tmp[o] <  FloorTransformation.parameters.rotation_thresh then
      	k = k+1
      	inliers[k] = best_pts[o]
	    scores[k] = 2*sorted[i]/sorted:max() + inliers[k]/math.max(scores_src_torch:size(1),scores_dest_torch:size(1))
      
      	src_centers_h[k] = src_centers_h_tmp[o]
      	src_centers_w[k] = src_centers_w_tmp[o]
      	tgt_centers_h[k] = tgt_centers_h_tmp[o]
      	tgt_centers_w[k] = tgt_centers_w_tmp[o]
      
      	transformations[k] = best_transformations[o]
      	trans1[k] = trans1_tmp[o]
      	trans2[k] = trans2_tmp[o]
      	combined[k] = combined_tmp[o]
      
      	size_x_all[k] = size_x_all_tmp[o]
      	size_y_all[k] = size_x_all_tmp[o]
      
      	--the closer to zero the better
	  	anglediff[k] = anglediff_tmp[o]
	  end
	  i = i-1
	  collectgarbage()
	  
   end
   
   if k < numRet then
   	scores = scores:sub(1,k)
   end
   
   trans1_tmp = nil
   trans2_tmp = nil
   combined_tmp = nil
   src_centers_h_tmp = nil
   src_centers_w_tmp = nil
   tgt_centers_h_tmp = nil
   tgt_centers_w_tmp = nil
   size_x_all_tmp = nil
   size_y_all_tmp = nil
   
   anglediff_tmp = nil
   
   collectgarbage()
   
   print(log.toc())

   return transformations, trans1, trans2, combined, inliers, anglediff, src_centers_h, src_centers_w, tgt_centers_h, tgt_centers_w, size_x_all, size_y_all, scores
end

function FloorTransformation.findTransformationSavedCorners(image1Path, image2Path, corners1Path, corners2Path, display)
   
   local scores_src_torch
   local scores_dest_torc

   local img_src = FloorTransformation.imagePreProcessing(image1Path)
   local img_dest = FloorTransformation.imagePreProcessing(image2Path)
   
   local scores_src_torch_ha = parameterizedHarris:findCorners(img_src)
   local scores_dest_torch_ha = parameterizedHarris:findCorners(img_dest)
   
   	scores_src_torch = torch.load(corners1Path)
   	scores_dest_torch = torch.load(corners2Path)

	if scores_src_torch == nil or scores_dest_torch == nil then
   		log.error('no corner points found!')
   		return nil
   	end
   	
   --example of how to do sift points:
   src_corners, dest_corners = parameterizedGeneral:getPoints(img_src, img_dest)   
   	
   print(scores_src_torch:size(1))
   print(scores_dest_torch:size(1))
   	
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

   local best_pts, best_transformations = opencv.imgproc.findBestTransformation(
      opencv.Mat.new(goodLocationsX_src:clone()),  opencv.Mat.new(goodLocationsY_src:clone()), opencv.Mat.new(scores_src_torch), opencv.Mat.new(pairwise_dis_src:clone()),
      opencv.Mat.new(goodLocationsX_dest:clone()), opencv.Mat.new(goodLocationsY_dest:clone()), opencv.Mat.new(scores_dest_torch), opencv.Mat.new(pairwise_dis_dest:clone()),
      FloorTransformation.parameters.corr_thresh, FloorTransformation.parameters.minInliersForMatch, FloorTransformation.parameters.maxNumCompute, 
      FloorTransformation.parameters.cornerDistanceLimit, img_src:size()[1], img_src:size()[2])
      
   local mainDirections = FloorTransformation.findMainDirections(img_src, img_dest)
   
   --[[]]
   local trans1_tmp = {}
   local trans2_tmp = {}
   local combined_tmp = {}
   local src_centers_h_tmp = {}
   local src_centers_w_tmp = {}
   local tgt_centers_h_tmp = {}
   local tgt_centers_w_tmp = {}
   
   local size_x_all_tmp = {}
   local size_y_all_tmp = {}
   
   local anglediff_tmp = torch.zeros(table.getn(best_transformations))
   
   local best_overlap = torch.Tensor(best_pts:size())
   for k=1,table.getn(best_transformations) do
      local i=k
      local trans1_i, trans2_i, combined_i
      if FloorTransformation.parameters.warpWithBorders then
         trans1_i, trans2_i, combined_i = image.warpAndCombineWithBorders(best_transformations[i], img_src, img_dest)
      else
         trans1_i, trans2_i, combined_i = image.warpAndCombine(best_transformations[i], img_src, img_dest)
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
     
      src_centers_h_tmp[i] = center1y
      src_centers_w_tmp[i] = center1x
      tgt_centers_h_tmp[i] = center2y
      tgt_centers_w_tmp[i] = center2x
      
      trans1_tmp[i] = t1--trans1_i
      trans2_tmp[i] = t2--trans2_i
      combined_tmp[i] = combined_i
      
      size_x_all_tmp[i] = img_src:size(1)
      size_y_all_tmp[i] = img_src:size(2)
      
      --the closer to zero the better
	   anglediff_tmp[i] = FloorTransformation.findMainDirections(opencv.Mat.new(combined_tmp[i][1]:byte()), opencv.Mat.new(combined_tmp[i][2]:byte()))
   
      local srci = combined_i:clone():select(1,1)
      srci:cdiv(srci:clone():add(0.0000000001)):ceil()
      local desi = combined_i:clone():select(1,2)
      desi:cdiv(desi:clone():add(0.0000000001)):ceil()
	  best_overlap[i]=srci:clone():cmul(desi):sum()
	  
	  collectgarbage()
   end
    collectgarbage()
   
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
   
   sorted,ordering = torch.sort(best_overlap)
   
   local numRet = math.min(table.getn(best_transformations),FloorTransformation.parameters.maxNumReturn)
   local scores = torch.Tensor(numRet)
   
   local i = table.getn(best_transformations)
   local k = 0
   while k<numRet and i > 0 do
      
      local o = ordering[i]
      
      if anglediff_tmp[o] <  FloorTransformation.parameters.rotation_thresh then
      	k = k+1
      	inliers[k] = best_pts[o]
	    scores[k] = sorted[i]/sorted:max()-- + inliers[k]/math.max(scores_src_torch:size(1),scores_dest_torch:size(1))
      
      	src_centers_h[k] = src_centers_h_tmp[o]
      	src_centers_w[k] = src_centers_w_tmp[o]
      	tgt_centers_h[k] = tgt_centers_h_tmp[o]
      	tgt_centers_w[k] = tgt_centers_w_tmp[o]
      
      	transformations[k] = best_transformations[o]
      	trans1[k] = trans1_tmp[o]
      	trans2[k] = trans2_tmp[o]
      	combined[k] = combined_tmp[o]
      
      	size_x_all[k] = size_x_all_tmp[o]
      	size_y_all[k] = size_x_all_tmp[o]
      
      	--the closer to zero the better
	  	anglediff[k] = anglediff_tmp[o]
	  end
	  i = i-1
	  collectgarbage()
	  
   end
   
   if k < numRet then
   	scores = scores:sub(1,k)
   end
   
   trans1_tmp = nil
   trans2_tmp = nil
   combined_tmp = nil
   src_centers_h_tmp = nil
   src_centers_w_tmp = nil
   tgt_centers_h_tmp = nil
   tgt_centers_w_tmp = nil
   size_x_all_tmp = nil
   size_y_all_tmp = nil
   
   anglediff_tmp = nil
   
   collectgarbage()

   return transformations, trans1, trans2, combined, inliers, anglediff, src_centers_h, src_centers_w, tgt_centers_h, tgt_centers_w, size_x_all, size_y_all, scores
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
   return (math.min(angle_options_real[1], math.pi/2-angle_options_real[1]))
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


