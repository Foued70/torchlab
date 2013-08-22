FloorTransformation = {}

colors = require '../types/Colors.lua'
require '../../geom/Homography'
require '../../geom/util'
require '../../geom/rotation_translation_homography'
Homography = geom.Homography
geom_util = geom.util
homography_funs = geom.rotation_translation_homography

FloorTransformation.erosion_size =4
FloorTransformation.npts_interest = 1000
FloorTransformation.corr_thresh = 2
FloorTransformation.structElement = opencv.imgproc.getDefaultStructuringMat(FloorTransformation.erosion_size) 
FloorTransformation.threshold_percentage = .4 --anything w score > .1*max score will be kept
FloorTransformation.radius_local_max = 3;

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
   image.displayPoints(img_src:toTensor(), src_corners, colors.CYAN, 2)
   image.displayPoints(img_dest:toTensor(), dest_corners, colors.MAGENTA, 2)

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
function FloorTransformation.findTransformationOurs(image1Path, image2Path)
   img_src = FloorTransformation.imagePreProcessing(image1Path)
   img_dest = FloorTransformation.imagePreProcessing(image2Path)
   scores_src_torch = FloorTransformation.cornerHarris(img_src)
   scores_dest_torch = FloorTransformation.cornerHarris(img_dest)
   print(scores_src_torch)
   image.displayPoints(img_src:toTensor(), scores_src_torch, colors.MAGENTA, 2)
   image.displayPoints(img_dest:toTensor(), scores_dest_torch, colors.CYAN, 2)
   
   --pairwise distance between corner points
   --we can then threshold this, to make sure we only look at pairs which are not too close together
   pairwise_dis_src = geom_util.pairwise_distance(scores_src_torch[{{},{1,2}}], scores_src_torch[{{},{1,2}}])
   pairwise_dis_dest = geom_util.pairwise_distance(scores_dest_torch[{{},{1,2}}], scores_dest_torch[{{},{1,2}}])

   goodLocationsX_src, goodLocationsY_src = image.thresholdReturnCoordinates(pairwise_dis_src,2 *FloorTransformation.corr_thresh)
   goodLocationsX_dest, goodLocationsY_dest = image.thresholdReturnCoordinates(pairwise_dis_dest,2 *FloorTransformation.corr_thresh)

   max_inliers = 0
   for i_src = 1, goodLocationsX_src:size(1) do
      pt1_src = goodLocationsX_src[i_src]
      pt2_src = goodLocationsY_src[i_src]
      src_pt1= scores_src_torch[{pt1_src, {1,2}}]
      src_pt2= scores_src_torch[{pt2_src, {1,2}}]
      d_src = pairwise_dis_src[pt1_src][pt2_src]

      --calculate transformation matrix and it's inverse
      A_inv = homography_funs.getInverseMatrixFromSource(src_pt1, src_pt2)
      for i_dest =1,goodLocationsX_dest:size(1) do
         pt1_dest = goodLocationsX_dest[i_dest]
         pt2_dest = goodLocationsY_dest[i_dest]
         dest_pt1= scores_dest_torch[{pt1_dest, {1,2}}]
         dest_pt2= scores_dest_torch[{pt2_dest, {1,2}}]
         d_dest = pairwise_dis_dest[pt1_dest][pt2_dest]  
         if (math.abs(d_dest-d_src)<FloorTransformation.corr_thresh) then
            b = homography_funs.getMatrixFromDestination(dest_pt1, dest_pt2)
            --check source+target condition abs(dist(p_src, p1_src) -  dist(p_target, p1_target)) < corr_thresh)
            --find transformation
            --transform all the source points, find distance from each transformed source point to it's equivalent destination point
            --take this matrix do a <corr_thresh and take it's sum, this is number of inliers
            H = homography_funs.getHomography(A_inv, b)
            transformed_src = H:applyToPointsReturn2d(scores_src_torch[{{}, {1,2}}]:t())

            num_inliers = torch.sum(
               torch.le(
                  geom_util.pairwise_distance(transformed_src:t(), scores_dest_torch[{{}, {1,2}}]),
                  FloorTransformation.corr_thresh))

            if(num_inliers > max_inliers) then
               --do validation here
               max_inliers = num_inliers;  
               bestT = H;
               print(num_inliers)
            end
         end
      end
   end
  
   trans1, trans2, combined = image.warpAndCombineWithBorders(bestT, img_src, img_dest)
   image.display(combined)
  -- combinedImage = opencv.imgproc.combineImages(img_src, img_dest, transform, 2700, 2700,500, 1700 )
   --opencv.imgproc.displayGrayMatInLua(combinedImage)

   return bestT
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
   scoresMat  = opencv.imgproc.detectCornerHarris(img, 2, 3, .04)
   scoresTorchF = scoresMat:toTensor()
   scoresTorchSquare = torch.DoubleTensor(scoresTorchF:size()):copy(scoresTorchF)
   
   --FloorTransformation.pickLocalMax(scoresTorchSquare)
   thresholdScores= scoresTorchSquare[torch.ge(scoresTorchSquare,scoresTorchSquare:max()*FloorTransformation.threshold_percentage)]
   goodLocationsX, goodLocationsY = image.thresholdReturnCoordinates(scoresTorchSquare,scoresTorchSquare:max() * FloorTransformation.threshold_percentage)
   
   scoresTorchSquare = FloorTransformation.pickLocalMaxFast(scoresTorchSquare, goodLocationsX, goodLocationsY)
   thresholdScores= scoresTorchSquare[torch.ge(scoresTorchSquare,scoresTorchSquare:max()*FloorTransformation.threshold_percentage)]
   goodLocationsX, goodLocationsY = image.thresholdReturnCoordinates(scoresTorchSquare,scoresTorchSquare:max() * FloorTransformation.threshold_percentage)
   
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
      output[x][y] =val
   end
   end)
   return output
end
--removes points which are not local max's in a radius_local_max bounding box around them
function FloorTransformation.pickLocalMaxSlow(scores_torch)
   i =0
   scores_torch:apply(function(val) 
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
end

return FloorTransformation
