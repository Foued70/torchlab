FindTransformation = Class()

local function cloneAndReturnMat(tensor)
	return opencv.Mat.new(tensor:clone())
end
--img_src expected to be dhw
function FindTransformation.findTransformation(min_x, max_x, min_y, max_y, scores_src_torch, scores_dest_torch, parameters)
   local pairwise_dis_src = geom.util.pairwise_distance(scores_src_torch, scores_src_torch)
   local pairwise_dis_dest = geom.util.pairwise_distance(scores_dest_torch, scores_dest_torch)

   local goodLocationsX_src, goodLocationsY_src = image.thresholdReturnCoordinates(pairwise_dis_src,2 *parameters.corr_thresh)
   local goodLocationsX_dest, goodLocationsY_dest = image.thresholdReturnCoordinates(pairwise_dis_dest,2 *parameters.corr_thresh)
   goodLocationsX_src = cloneAndReturnMat(goodLocationsX_src:reshape(goodLocationsX_src:size()[1],1))
   goodLocationsY_src = cloneAndReturnMat(goodLocationsY_src:reshape(goodLocationsY_src:size()[1],1))
   goodLocationsX_dest = cloneAndReturnMat(goodLocationsX_dest:reshape(goodLocationsX_dest:size()[1],1))
   goodLocationsY_dest = cloneAndReturnMat(goodLocationsY_dest:reshape(goodLocationsY_dest:size()[1],1))

   local inliers, best_transformations = opencv.imgproc.findBestTransformation(goodLocationsX_src, goodLocationsY_src, 
      cloneAndReturnMat(scores_src_torch), cloneAndReturnMat(pairwise_dis_src:clone()),
      goodLocationsX_dest, goodLocationsY_dest, cloneAndReturnMat(scores_dest_torch), cloneAndReturnMat(pairwise_dis_dest),
      parameters.corr_thresh, parameters.minInliersForMatch, parameters.maxNumCompute, 
      parameters.cornerDistanceLimit, min_x, max_x, min_y, max_y)
	inliers = inliers/math.min(scores_src_torch:size(1),scores_dest_torch:size(1))
   sort, order = torch.sort(inliers,true)
   transformations = {}
   for i =1, sort:size(1) do
      transformations[i] = best_transformations[order[i]]
   end
   return sort, transformations;
end