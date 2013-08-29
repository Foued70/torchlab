FloorTransformation = Class()

colors = require '../opencv/types/Colors.lua'
Homography = geom.Homography
homography_funs = geom.rotation_translation_homography

FloorTransformation.erosion_size =1
FloorTransformation.npts_interest = 1000
FloorTransformation.corr_thresh = 2
FloorTransformation.structElement = opencv.imgproc.getDefaultStructuringMat(FloorTransformation.erosion_size) 
FloorTransformation.threshold = 70 --anything w score > .1*max score will be kept
FloorTransformation.radius_local_max = 12;
FloorTransformation.blockSize =2
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
FloorTransformation.useHough = false

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
   
   if(FloorTransformation.useHough) then
      local locations_hough_source = FloorTransformation.getHoughLineIntersects(img_src)
      local locations_hough_dest = FloorTransformation.getHoughLineIntersects(img_dest)

      scores_src_torch = torch.cat(scores_src_torch, locations_hough_source,1)
      scores_dest_torch = torch.cat(scores_dest_torch, locations_hough_dest,1)
      end
   scores_src = opencv.Mat.new(scores_src_torch:clone())
   scores_dest = opencv.Mat.new(scores_dest_torch:clone())

   if(display) then
      image.displayPoints(img_src:toTensor(), scores_src_torch, colors.MAGENTA, 2)
      image.displayPoints(img_dest:toTensor(), scores_dest_torch, colors.CYAN, 2)
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

   print(scores_src_torch:size()[1])
   best_pts, best_transformations = opencv.imgproc.findBestTransformation(
      opencv.Mat.new(goodLocationsX_src:clone()),  opencv.Mat.new(goodLocationsY_src:clone()), opencv.Mat.new(scores_src_torch:clone()), opencv.Mat.new(pairwise_dis_src:clone()),
      opencv.Mat.new(goodLocationsX_dest:clone()), opencv.Mat.new(goodLocationsY_dest:clone()), opencv.Mat.new(scores_dest_torch:clone()), opencv.Mat.new(pairwise_dis_dest:clone()),
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


function FloorTransformation.isSameTransformation(H1, H2, size_x, size_y)
   corners = torch.Tensor({{size_x,1},{1,1},{1,size_y},{size_x,size_y}}):t()
   transformedCorners1 = H1:applyToPointsReturn2d(corners)
   transformedCorners2 = H2:applyToPointsReturn2d(corners)
   totalCloseCorners = torch.sum(torch.le(geom.util.distance(transformedCorners1,transformedCorners2), FloorTransformation.cornerDistanceLimit))
   return totalCloseCorners==4
   --[[
   angle1 = torch.acos(H1[1][1])
   if (torch.sin(angle1) >0) and H1[1][1] >0 then
      angle1= 2*math.pi - angle1;
   end

   angle2 = torch.acos(H2[1][1])
   if (torch.sin(angle2) >0) and H2[1][1] >0 then
      angle2= 2*math.pi - angle2;
   end
   if torch.abs(angle1-angle2) < 5*2*math.pi/360 and 
                     torch.abs(H1[1][3]-H2[1][3]) < 30 and
                     torch.abs(H1[2][3]-H2[2][3]) < 30
      then
      return true
   end
   return false
   ]]--
end
--returns an opencv mat with the image with the following done to it:
--loaded in opencv 
--converted to grayscale
--dilated and eroded to get rid of random noise/small things
function FloorTransformation.imagePreProcessing(imagePath)
   img = opencv.Mat.new(imagePath)
   img:convert("RGB2GRAY");
   opencv.imgproc.dilate(img, FloorTransformation.structElement);
   opencv.imgproc.erode(img, FloorTransformation.structElement);
   return img
end

--return 3xn matrix where each coordinate represents x,y,score of a key point using opencv's cornerHarris method
function FloorTransformation.cornerHarris(img)
   scoresMat  = opencv.imgproc.detectCornerHarris(img, FloorTransformation.blockSize, FloorTransformation.kSize, FloorTransformation.k)
   scoresTorchF = scoresMat:toTensor()
   scoresTorchSquare = torch.DoubleTensor(scoresTorchF:size()):copy(scoresTorchF)
   
   --FloorTransformation.pickLocalMax(scoresTorchSquare)
   thresholdScores= scoresTorchSquare[torch.ge(scoresTorchSquare, FloorTransformation.threshold)]
   goodLocationsX, goodLocationsY = image.thresholdReturnCoordinates(scoresTorchSquare,FloorTransformation.threshold)
   
   scoresTorchSquare = FloorTransformation.pickLocalMaxFast(scoresTorchSquare, goodLocationsX, goodLocationsY)
   thresholdScores= scoresTorchSquare[torch.ge(scoresTorchSquare,FloorTransformation.threshold)]
   goodLocationsX, goodLocationsY = image.thresholdReturnCoordinates(scoresTorchSquare, FloorTransformation.threshold)
   t1,sortOrder = torch.sort(thresholdScores,1,true)
   topScores = torch.Tensor(math.min(FloorTransformation.npts_interest,thresholdScores:size(1)),3)
   for i=1,math.min(FloorTransformation.npts_interest,thresholdScores:size(1))  do
      topScores[{i,{}}]=torch.Tensor({goodLocationsX[sortOrder[i]],goodLocationsY[sortOrder[i]],t1[i]})
   end
   return topScores;
end

--removes points which are not local max's in a radius_local_max bounding box around them
--only considers points which have been deemed as goods (i.e. larger than the threshold and in goodlocationsX/Y)
function FloorTransformation.pickLocalMaxFast(scores_torch, goodLocationsX, goodLocationsY)
   output = torch.zeros(scores_torch:size())
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
      output = scores_torch:clone()

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


function FloorTransformation.drawLine(img, x0, y0, x1, y1, value)
   startY = 1
   endY =img:size()[2]

   startX = 1
   endX = img:size()[1]
   
   if x1~=x0 then
      for i=startX, endX do
         y = y0 + (y1-y0)/(x1-x0)*(i-x0)
         if not(y>endY) and not(y<startY) then
            img[i][y] = value
         end
      end
   end
   if y1~=y0 then
      for i=startY, endY do
         x = x0 + (x1-x0)/(y1-y0)*(i-y0)
         if not(x>endX) and not(x<startX) then
            img[x][i] = value
         end
      end
   end
end

function FloorTransformation.findIntersect(firstCoordinates, secondCoordinates)

   --first line
   sx0 = firstCoordinates[2]
   sy0 = firstCoordinates[1]
   sx1 = firstCoordinates[4]
   sy1 = firstCoordinates[3]

   sm=(sy1-sy0)/(sx1-sx0)

   sb = sy0-sm*sx0
   dx0 = secondCoordinates[2]
   dy0 = secondCoordinates[1]
   dx1 = secondCoordinates[4]
   dy1 = secondCoordinates[3]

   dm=(dy1-dy0)/(dx1-dx0)
   db = dy0-dm*dx0

   ix = (db-sb)/(sm-dm)
   iy = sm*(ix)+sb

   return ix, iy

end

function FloorTransformation.getHoughLineIntersects(img)
   -- use graphics magick to load the image

   dst = opencv.imgproc.CannyDetectEdges(img, 100, 200)
   linesP = FloorTransformation.binarySearchForClosestNumberLines(dst, 20, 120)
   linesT = linesP:toTensor()+1;
   linesT = linesT:reshape(linesT:size()[2], linesT:size()[3])

   slopes = torch.atan(torch.cdiv((linesT[{{},1}]-linesT[{{},3}]):double(),(linesT[{{},2}]-linesT[{{},4}]):double()))
   slopes:apply(function(val) if val<0 then return math.pi + val end end)
   vals, order = torch.sort(slopes)
   absoluted = torch.abs(vals[{{1,vals:size()[1]-1},1}]-vals[{{2,vals:size()[1]},1}])
   maxV, locV = torch.max(absoluted,1)

   firstGroupIndices = order[{{1,locV[1]},1}]
   secondGroupIndices = order[{{locV[1]+1, vals:size()[1]},1}]
   points = torch.zeros((firstGroupIndices:size()[1])*secondGroupIndices:size()[1], 2)
   counter = 1
   for i=1, firstGroupIndices:size()[1] do
      for j=1, secondGroupIndices:size()[1] do
         x, y = FloorTransformation.findIntersect(linesT[{firstGroupIndices[i],{}}],linesT[{secondGroupIndices[j],{}}])
         if (x>0) and (x<=img:size()[1]) and (y>0 ) and (y<=img:size()[2]) then
            torch.Tensor({x,y})
            points[{counter,{}}]:size()
            points[{counter,{}}] = torch.Tensor({x,y})
            counter = counter+1
         end
         x = nil
         y = nil
      end
   end
   return points[{{1,counter-1},{}}]
end


function FloorTransformation.binarySearchForClosestNumberLines(dst, minH, maxH)
   houghThreshold = torch.floor((maxH+minH)/2)
   linesP = opencv.imgproc.HoughLinesProbabilistic(dst, FloorTransformation.houghRo, FloorTransformation.houghTheta, 
               houghThreshold, FloorTransformation.houghMinLineLength, FloorTransformation.houghMaxLineGap  );
   numLines = linesP:size()[2]
   if(houghThreshold == minH) or (houghThreshold == maxH) or (numLines == FloorTransformation.houghNumLines) then
      return linesP
   end
   if(numLines > FloorTransformation.houghNumLines) then
         return FloorTransformation.binarySearchForClosestNumberLines(dst, houghThreshold, maxH)
   else
         return FloorTransformation.binarySearchForClosestNumberLines(dst, minH, houghThreshold)
   end
end
