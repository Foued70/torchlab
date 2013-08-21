FloorTransformation = {}

opencv    = require '../init'
colors = require '../types/Colors.lua'
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
   _G.img_src = img_src
   _G.src_corners = src_corners
   _G.opencv = opencv
   opencv.imgproc.displayPoints(img_src, src_corners, colors.CYAN, 2)
   opencv.imgproc.displayPoints(img_dest, dest_corners, colors.MAGENTA, 2)

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
   opencv.imgproc.displayGrayMatInLua(warped)
end
function FloorTransformation.findTransformationOurs(image1Path, image2Path)
   img_src = FloorTransformation.imagePreProcessing(image1Path)
   img_dest = FloorTransformation.imagePreProcessing(image2Path)
   scores_src_torch = FloorTransformation.cornerHarris(img_src)
   scores_dest_torch = FloorTransformation.cornerHarris(img_dest)
   
   opencv.imgproc.displayPoints(img_src, scores_src_torch, colors.MAGENTA, 2)
   opencv.imgproc.displayPoints(img_dest, scores_dest_torch, colors.CYAN, 2)
   
   pairwise_dis_src = FloorTransformation.getDistance(scores_src_torch[{{},{1,2}}], scores_src_torch[{{},{1,2}}])
   pairwise_dis_dest = FloorTransformation.getDistance(scores_dest_torch[{{},{1,2}}], scores_dest_torch[{{},{1,2}}])

   goodLocationsX_src, goodLocationsY_src = FloorTransformation.selectInRectangularMatrix(pairwise_dis_src,2 *FloorTransformation.corr_thresh)
   goodLocationsX_dest, goodLocationsY_dest = FloorTransformation.selectInRectangularMatrix(pairwise_dis_dest,2 *FloorTransformation.corr_thresh)

   max_inliers = 0
   for i_src = 1, goodLocationsX_src:size(1) do
      pt1_src = goodLocationsX_src[i_src]
      pt2_src = goodLocationsY_src[i_src]
      src_pt1= scores_src_torch[{pt1_src, {1,2}}]
      src_pt2= scores_src_torch[{pt2_src, {1,2}}]
      d_src = pairwise_dis_src[pt1_src][pt2_src]

      --calculate transformation matrix and it's inverse
      A_inv = FloorTransformation.getSourceInverseMatrix(src_pt1, src_pt2)
      for i_dest =1,goodLocationsX_dest:size(1) do
         pt1_dest = goodLocationsX_dest[i_dest]
         pt2_dest = goodLocationsY_dest[i_dest]
         dest_pt1= scores_dest_torch[{pt1_dest, {1,2}}]
         dest_pt2= scores_dest_torch[{pt2_dest, {1,2}}]
         d_dest = pairwise_dis_dest[pt1_dest][pt2_dest]  
         if (math.abs(d_dest-d_src)<FloorTransformation.corr_thresh) then
            b = FloorTransformation.getDestinationMatrix(dest_pt1, dest_pt2)
            --check source+target condition abs(dist(p_src, p1_src) -  dist(p_target, p1_target)) < corr_thresh)
            --find transformation
            --transform all the source points, find distance from each transformed source point to it's equivalent destination point
            --take this matrix do a <corr_thresh and take it's sum, this is number of inliers
            T = FloorTransformation.getTransformationMatrix(A_inv, b)
            transformed_src = FloorTransformation.transformSrcMatrix(T,scores_src_torch[{{}, {1,2}}]:t())

            num_inliers = torch.sum(
               torch.le(
                  FloorTransformation.getDistance(transformed_src[{{1,2}, {}}]:t(), scores_dest_torch[{{}, {1,2}}]),
                  FloorTransformation.corr_thresh))

            if(num_inliers > max_inliers) then
               --do validation here
               max_inliers = num_inliers;  
               bestT = T;
               print(num_inliers)
            end
         end
      end
   end
  
FloorTransformation.displayWithBorder(bestT, img_src, img_dest)

  -- combinedImage = opencv.imgproc.combineImages(img_src, img_dest, transform, 2700, 2700,500, 1700 )
   --opencv.imgproc.displayGrayMatInLua(combinedImage)

   return bestT
end

function FloorTransformation.displayWithBorder(bestT, img_src, img_dest)
   img_dest_copy = img_dest:clone()
   img_dest_copy:toTensor()[{{1,img_dest:size()[1]},{1}}]:fill(255)
   img_dest_copy:toTensor()[{{1,img_dest:size()[1]},{img_dest:size()[2]}}]:fill(255)
   img_dest_copy:toTensor()[{{1},{1,img_dest:size()[2]}}]:fill(255)
   img_dest_copy:toTensor()[{{img_dest:size()[1]}, {1,img_dest:size()[2]}}]:fill(255)

   img_src_copy = img_src:clone()
   img_src_copy:toTensor()[{{1,img_src:size()[1]},{1}}]:fill(255)
   img_src_copy:toTensor()[{{1,img_src:size()[1]},{img_src:size()[2]}}]:fill(255)
   img_src_copy:toTensor()[{{1},{1,img_src:size()[2]}}]:fill(255)
   img_src_copy:toTensor()[{{img_src:size()[1]}, {1,img_src:size()[2]}}]:fill(255)

   return FloorTransformation.display(bestT, img_src_copy, img_dest_copy)
end

function FloorTransformation.display(bestT, img_src, img_dest)
   corners_src = torch.Tensor({{0,0}, {0, img_src:size()[2]}, {img_src:size()[1],0}, {img_src:size()[1],img_src:size()[2]}}):t()
   corners_dest =torch.Tensor({{0,0}, {0, img_dest:size()[2]}, {img_dest:size()[1],0}, {img_dest:size()[1],img_dest:size()[2]}}):t()
   corners_src_warped = FloorTransformation.transformSrcMatrix(bestT, corners_src)[{{1,2},{}}]

   min_corner = torch.cat(corners_src_warped, corners_dest):min(2)
   max_corner = torch.cat(corners_src_warped, corners_dest):max(2)

   --this is in lua coordinate system, swap x and y for opencv
   size_x = (max_corner-min_corner)[1][1]
   size_y = (max_corner-min_corner)[2][1]

   translate = torch.Tensor({{1, 0, -min_corner[1][1]},{0, 1, -min_corner[2][1]},{0, 0, 1}})
   src_transformation_lua = translate*bestT
   --switch to opencv coordinates!
   src_transform = opencv.Mat.new(FloorTransformation.switch3x3MatrixToCV(src_transformation_lua))
   dest_transform = opencv.Mat.new(FloorTransformation.switch3x3MatrixToCV(translate))   

   --opencv.imgproc.displayGrayMatInLua(img_src_copy)
   warpedSrc = opencv.imgproc.warpImage(img_src, src_transform, size_y, size_x)

   warpedDest = opencv.imgproc.warpImage(img_dest_copy, dest_transform, size_y, size_x)
   --opencv.imgproc.displayGrayMatInLua(warpedDest)

   tensor3d = torch.zeros(3,warpedSrc:size()[1], warpedSrc:size()[2])
   tensor3d[1] = warpedSrc:toTensor()
   tensor3d[2] = warpedDest:toTensor()

   image.display(tensor3d)
   return src_transform, dest_transform, tensor3d
end

function FloorTransformation.switch3x3MatrixToCV(transform) 
   bestT = transform:clone()
   bestT[1][2] = -bestT[1][2]
   bestT[2][1]=-bestT[2][1]
   temp = bestT[1][3]
   bestT[1][3]=bestT[2][3]
   bestT[2][3]=transform[1][3]
   return bestT
end
--in the transformation we are trying to solve for t b=At, where A is dependent on source points, t is the transformation we are solving for 
-- we will returned the reshaped matrix such that T*x represents where the point x gets mapped
--goal is to solve:
   --[d0_x d1_x; d0y d1y; 1 1] = [t0 -t1 t2; t1 t0 t3; 0 0 1]*[s0x s1x; s0y s1y; 1 1]
   --or equivalently
   --[d0_x; d0_y; d1_x; d1_y] = [s0_x s0_y 1 0; -s0_y s0_x 0 1; s0x -s1y 1 0; s1y s1x 0 1] * [t_0; t_1; t_2; t_3]
   --left is destination matrix ([d0_x; d0_y; d1_x; d1_y] ), A is source matrix ([-s0y...])
function FloorTransformation.getSourceMatrix(src_pt1, src_pt2) 
   return torch.Tensor({{src_pt1[1], -src_pt1[2], 1, 0},{src_pt1[2], src_pt1[1], 0, 1},{src_pt2[1],-src_pt2[2], 1,0}, {src_pt2[2], src_pt2[1], 0, 1}})
end

function FloorTransformation.getDestinationMatrix(dest_pt1, dest_pt2)
   return torch.Tensor({{dest_pt1[1]}, {dest_pt1[2]}, {dest_pt2[1]}, {dest_pt2[2]}})
end

function FloorTransformation.getSourceInverseMatrix(src_pt1, src_pt2) 
   return torch.inverse(FloorTransformation.getSourceMatrix(src_pt1, src_pt2))
end

--get the actual transformation matrix T such that T*src points = dest points
function FloorTransformation.getTransformationMatrix(A_inv, b) 
   t = torch.mm(A_inv, b):resize(4);
   return torch.Tensor({{t[1], -t[2], t[3]}, {t[2], t[1], t[4]}, {0, 0, 1}} )
end

--T is 3x3 transformation matrix, mat_src is 2xn matrix
function FloorTransformation.transformSrcMatrix(T, mat_src)
   return torch.mm(T, FloorTransformation.transformXYtoXYZ(mat_src))
end

function FloorTransformation.transformXYtoXYZ(mat)
   return torch.cat(mat, torch.Tensor(1,mat:size(2)):fill(1),1)
end

--each matrix is 3xn, returns the distance between corresponding points in transf_src_mat and dest_mat
--that is returns a matrix of size 1xn where each element represents the distance between the ith point in the transf source matrix and ith 
--point in dest mat
function FloorTransformation.distance(transf_src_mat, dest_mat)
   diff = transf_src_mat-dest_mat
   mul_diff = diff:cmul(diff)
   squared_distance = mul_diff:select(1,1)+mul_diff:select(1,2)
   return squared_distance:apply(math.sqrt)
end

--returns an opencv mat with the image with the following done to it:
--loaded in opencv 
--converted to grayscale
--dilated and eroded to get rid of random noise
function FloorTransformation.imagePreProcessing(imagePath)
   img = opencv.Mat.new(imagePath)
   img:convert("RGB2GRAY");
   opencv.imgproc.dilate(img, FloorTransformation.structElement);
   opencv.imgproc.erode(img, FloorTransformation.structElement);
   return img
end

--get the pairwise distance between all pairs of points in A and B
--if A is mxp and B is nxp then the result will be mxn
function FloorTransformation.getDistance(A, B)
   AdA = torch.sum(torch.cmul(A,A),2) --mx1
   A_new = AdA * torch.Tensor(1,B:size(1)):fill(1) --mxn
   BdB = torch.sum(torch.cmul(B,B),2) --nx1
   B_new = torch.Tensor(A:size(1),1):fill(1) *BdB:t() --mxn
   dis_square = A_new + B_new - torch.mul(A*B:t(), 2)
   return dis_square:apply(math.sqrt)
end
--return 3xn matrix where each coordinate represents x,y,score of a key point using opencv's cornerHarris method
function FloorTransformation.cornerHarris(img)
   scoresMat  = opencv.imgproc.detectCornerHarris(img, 2, 3, .04)
   scoresTorchF = scoresMat:toTensor()
   scoresTorchSquare = torch.DoubleTensor(scoresTorchF:size()):copy(scoresTorchF)
   
   --FloorTransformation.pickLocalMax(scoresTorchSquare)
   thresholdScores= scoresTorchSquare[torch.ge(scoresTorchSquare,scoresTorchSquare:max()*FloorTransformation.threshold_percentage)]
   goodLocationsX, goodLocationsY = FloorTransformation.selectInRectangularMatrix(scoresTorchSquare,scoresTorchSquare:max() * FloorTransformation.threshold_percentage)
   
   scoresTorchSquare = FloorTransformation.pickLocalMaxFast(scoresTorchSquare, goodLocationsX, goodLocationsY)
   thresholdScores= scoresTorchSquare[torch.ge(scoresTorchSquare,scoresTorchSquare:max()*FloorTransformation.threshold_percentage)]
   goodLocationsX, goodLocationsY = FloorTransformation.selectInRectangularMatrix(scoresTorchSquare,scoresTorchSquare:max() * FloorTransformation.threshold_percentage)
   
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

--only keep value greater than threshold
function FloorTransformation.selectInRectangularMatrix(squareMatrix,threshold)
   h=squareMatrix:size(1)
   w=squareMatrix:size(2)
   
   x = torch.Tensor(h*w)
   i = 0
   x:apply(function() i = i + 1; return i end)
   goodLocations = x[torch.ge(squareMatrix,threshold)]
   goodLocationsX = torch.ceil(goodLocations/w)
   goodLocationsY = goodLocations:clone()
   goodLocationsY:apply(function(val) return (val -1)%w+1 end) 

   return goodLocationsX, goodLocationsY

end

return FloorTransformation