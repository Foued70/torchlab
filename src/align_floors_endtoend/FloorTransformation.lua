FloorTransformation = Class()

colors = require '../opencv/types/Colors.lua'
Homography = geom.Homography
homography_funs = geom.rotation_translation_homography

FloorTransformation.erosion_size =1
FloorTransformation.npts_interest = 25--1000
FloorTransformation.corr_thresh = 2
FloorTransformation.structElement = opencv.imgproc.getDefaultStructuringMat(FloorTransformation.erosion_size) 
FloorTransformation.threshold = 50--70 --anything w score > .1*max score will be kept
FloorTransformation.radius_local_max = 5--12;
FloorTransformation.blockSize =5--2
FloorTransformation.kSize =3
FloorTransformation.k = .04
FloorTransformation.maxNumReturn = 50
FloorTransformation.warpWithBorders = false
FloorTransformation.cornerDistanceLimit = 25
FloorTransformation.minInliersForMatch = 2

--HOUGH PARAMETERS
FloorTransformation.houghRo = 1
FloorTransformation.houghTheta = math.pi/360
FloorTransformation.houghNumLines = 10 -- you will get roughly half this number squared of corner points
FloorTransformation.houghMinLineLength = 25
FloorTransformation.houghMaxLineGap = 80
FloorTransformation.useHough = true

function FloorTransformation.findTransformationStandard(image1Path, image2Path)
   img_src = FloorTransformation.imagePreProcessing(image1Path)
   img_dest = FloorTransformation.imagePreProcessing(image2Path)

   dtype = "SIFT"
   detector    = opencv.Detector.new(dtype)
   kpts_src, npts_src  = detector:detect(img_src,FloorTransformation.npts_interest)
   kpts_dest, npts_dest  = detector:detect(img_dest,FloorTransformation.npts_interest)
   
   src_corners = torch.Tensor(npts_src,2)
   dest_corners = torch.Tensor(npts_dest,2)

   for i=1, npts_src do
      src_x = torch.ceil(kpts_src[i].pt.x)
      src_y = torch.ceil(kpts_src[i].pt.y)

      src_corners[i][1] = src_y
      src_corners[i][2] = src_x
   end
   for i=1, npts_dest do
      dest_x = torch.ceil(kpts_dest[i].pt.x)
      dest_y = torch.ceil(kpts_dest[i].pt.y)
      dest_corners[i][1] = dest_y
      dest_corners[i][2] = dest_x
   end
   --image.displayPoints(img_src:toTensor(), src_corners, colors.CYAN, 2)
   --image.displayPoints(img_dest:toTensor(), dest_corners, colors.MAGENTA, 2)

   extractor_type = "SIFT"
   extractor   = opencv.Extractor.new(extractor_type)

   descriptors_src  = extractor:compute(img_src,kpts_src,npts_src)
   descriptors_dest = extractor:compute(img_dest,kpts_dest,npts_dest)

   matcher_type = "FlannBased"
   matcher      = opencv.Matcher.new(matcher_type)
   sizeSrc      = descriptors_src:size()[1]
   sizeDest     = descriptors_dest:size()[1]

   matches,nmatches = matcher:match(descriptors_src, descriptors_dest, sizeSrc*sizeDest)

   matches_good,nmatches_good = matcher:reduce(matches, nmatches)

   if(nmatches_good < 10) then
      matches_good = matches
      nmatches_good = nmatches
   end
   H = opencv.calib3d.getHomography(kpts_src, npts_src, kpts_dest, npts_dest, matches_good, nmatches_good)

   warped = opencv.imgproc.warpImage(img_src, H)
   image.displayGrayInLua(warped)
end
function FloorTransformation.findTransformationOurs(image1Path, image2Path, display)
   log.tic()
   local img_src = FloorTransformation.imagePreProcessing(image1Path)
   local img_dest = FloorTransformation.imagePreProcessing(image2Path)
   local scores_src_torch = FloorTransformation.cornerHarris(img_src)
   local scores_dest_torch = FloorTransformation.cornerHarris(img_dest)

   scores_src_torch = scores_src_torch[{{}, {1,2}}]
   scores_dest_torch = scores_dest_torch[{{}, {1,2}}]
   
   print(scores_src_torch:size(1))
   print(scores_dest_torch:size(1))
   
   if(FloorTransformation.useHough) then
      local locations_hough_source = align_floors_endtoend.Hough.getHoughLinesAndPoints(img_src)--FloorTransformation.getHoughLineIntersects(img_src)
      local locations_hough_dest = align_floors_endtoend.Hough.getHoughLinesAndPoints(img_dest)--FloorTransformation.getHoughLineIntersects(img_dest)

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

   goodLocationsX_src, goodLocationsY_src = image.thresholdReturnCoordinates(pairwise_dis_src,2 *FloorTransformation.corr_thresh)
   goodLocationsX_dest, goodLocationsY_dest = image.thresholdReturnCoordinates(pairwise_dis_dest,2 *FloorTransformation.corr_thresh)

   goodLocationsX_src = goodLocationsX_src:reshape(goodLocationsX_src:size()[1],1)
   goodLocationsY_src = goodLocationsY_src:reshape(goodLocationsY_src:size()[1],1)
   goodLocationsX_dest = goodLocationsX_dest:reshape(goodLocationsX_dest:size()[1],1)
   goodLocationsY_dest = goodLocationsY_dest:reshape(goodLocationsY_dest:size()[1],1)

      print(pairwise_dis_src:size())

   local best_pts, best_transformations = opencv.imgproc.findBestTransformation(
      opencv.Mat.new(goodLocationsX_src:clone()),  opencv.Mat.new(goodLocationsY_src:clone()), opencv.Mat.new(scores_src_torch), opencv.Mat.new(pairwise_dis_src:clone()),
      opencv.Mat.new(goodLocationsX_dest:clone()), opencv.Mat.new(goodLocationsY_dest:clone()), opencv.Mat.new(scores_dest_torch), opencv.Mat.new(pairwise_dis_dest:clone()),
      FloorTransformation.corr_thresh, FloorTransformation.minInliersForMatch, FloorTransformation.maxNumReturn, 
      FloorTransformation.cornerDistanceLimit, img_src:size()[1], img_src:size()[2])

   local trans1 = {}
   local trans2 = {}
   local combined = {}
   local outliers = {}
   local transformations = {}
   local src_centers_h = {}
   local src_centers_w = {}
   local tgt_centers_h = {}
   local tgt_centers_w = {}
  
   sorted,ordering = torch.sort(best_pts)
   for k=1,table.getn(best_transformations) do
      i=table.getn(best_transformations)-k+1
      outliers[k] = sorted[i]
      local trans1_i, trans2_i, combined_i
      if FloorTransformation.warpWithBorders then
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
      
      transformations[k] = best_transformations[ordering[i]]
      trans1[k] = trans1_i
      trans2[k] = trans2_i
      combined[k] = combined_i
      --if(display) then
      --   image.display(combined[i])
      --end
   end
   print(log.toc())
   return best_transformations, trans1, trans2, combined, outliers, src_centers_h, src_centers_w, tgt_centers_h, tgt_centers_w
end


--returns an opencv mat with the image with the following done to it:
--loaded in opencv 
--converted to grayscale
--dilated and eroded to get rid of random noise/small things
function FloorTransformation.imagePreProcessing(imagePath)
   local img = opencv.Mat.new(imagePath)
   img:convert("RGB2GRAY");
   opencv.imgproc.dilate(img, FloorTransformation.structElement);
   opencv.imgproc.erode(img, FloorTransformation.structElement);
   return img
end

--return 3xn matrix where each coordinate represents x,y,score of a key point using opencv's cornerHarris method
function FloorTransformation.cornerHarris(img)
   local scoresMat  = opencv.imgproc.detectCornerHarris(img, FloorTransformation.blockSize, FloorTransformation.kSize, FloorTransformation.k)
   local scoresTorchF = scoresMat:toTensor()
   local scoresTorchSquare = torch.DoubleTensor(scoresTorchF:size()):copy(scoresTorchF)
   
   --FloorTransformation.pickLocalMax(scoresTorchSquare)
   local thresholdScores= scoresTorchSquare[torch.ge(scoresTorchSquare, FloorTransformation.threshold)]
   local goodLocationsX, goodLocationsY = image.thresholdReturnCoordinates(scoresTorchSquare,FloorTransformation.threshold)
   
   local scoresTorchSquare = FloorTransformation.pickLocalMaxFast(scoresTorchSquare, goodLocationsX, goodLocationsY)
   local thresholdScores= scoresTorchSquare[torch.ge(scoresTorchSquare,FloorTransformation.threshold)]
   local goodLocationsX, goodLocationsY = image.thresholdReturnCoordinates(scoresTorchSquare, FloorTransformation.threshold)
   local t1,sortOrder = torch.sort(thresholdScores,1,true)
   local topScores = torch.Tensor(math.min(FloorTransformation.npts_interest,thresholdScores:size(1)),3)
   for i=1,math.min(FloorTransformation.npts_interest,thresholdScores:size(1))  do
      topScores[{i,{}}]=torch.Tensor({goodLocationsX[sortOrder[i]],goodLocationsY[sortOrder[i]],t1[i]})
   end
   return topScores;
end

--removes points which are not local max's in a radius_local_max bounding box around them
--only considers points which have been deemed as goods (i.e. larger than the threshold and in goodlocationsX/Y)
function FloorTransformation.pickLocalMaxFast(scores_torch, goodLocationsX, goodLocationsY)
   local output = torch.zeros(scores_torch:size())
   i =0
   goodLocationsX:apply(function(val) 
      i=i+1
      x = goodLocationsX[i]
      y = goodLocationsY[i]
      localMax = scores_torch[{
         {math.max(x-FloorTransformation.radius_local_max,1), math.min(x+FloorTransformation.radius_local_max, scores_torch:size(1))},
         {math.max(y-FloorTransformation.radius_local_max,1), math.min(y+FloorTransformation.radius_local_max, scores_torch:size(2))}
         }]:max();
      if(scores_torch[x][y]>=localMax) then
         output[x][y] =scores_torch[x][y]
      end
   end)
   return output
end
--removes points which are not local max's in a radius_local_max bounding box around them
function FloorTransformation.pickLocalMaxSlow(scores_torch)
   local output = scores_torch:clone()
   i =0
   output:apply(function(val) 
      i=i+1
      x = torch.ceil(i/scores_torch:size(2))
      y = (i-1)%scores_torch:size(2)+1
      localMax = scores_torch[{
         {math.max(x-FloorTransformation.radius_local_max,1), math.min(x+FloorTransformation.radius_local_max, scores_torch:size(1))},
         {math.max(y-FloorTransformation.radius_local_max,1), math.min(y+FloorTransformation.radius_local_max, scores_torch:size(2))}
         }]:max();
      if(scores_torch[x][y]>=localMax) then
         return val
      else
         return 0
      end
   end)
   return output
end


