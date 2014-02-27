FindTransformation = Class()

local function cloneAndReturnMat(tensor)
	return opencv.Mat.new(tensor:clone())
end
--img_src expected to be dhw
function FindTransformation.findTransformation(source_sweep, dest_sweep, parameters)
   local scores_src_torch, flattenedxy1, flattenedv1, source_corners = source_sweep:get_flattened_and_corners()
   local scores_dest_torch, flattenedxy2, flattenedv2, dest_corners = dest_sweep:get_flattened_and_corners()

    local minT =torch.min(flattenedxy1,1):reshape(2)
    local maxT = torch.max(flattenedxy1,1):reshape(2)
   local pairwise_dis_src = geom.util.pairwise_distance(scores_src_torch, scores_src_torch)
   local pairwise_dis_dest = geom.util.pairwise_distance(scores_dest_torch, scores_dest_torch)
   
   local t1 = torch.cat(source_corners, torch.ones(source_corners:size(1)),2)
   local t2 = torch.cat(torch.ones(dest_corners:size(1)), -dest_corners,2):t()
   local angle_diff = t1*t2
   angle_diff[angle_diff:clone():lt(0)] = angle_diff[angle_diff:clone():lt(0)] +2*math.pi

   local goodLocationsX_src, goodLocationsY_src = image.thresholdReturnCoordinates(pairwise_dis_src,2 *parameters.corr_thresh)
   local goodLocationsX_dest, goodLocationsY_dest = image.thresholdReturnCoordinates(pairwise_dis_dest,2 *parameters.corr_thresh)
   goodLocationsX_src = cloneAndReturnMat(goodLocationsX_src:reshape(goodLocationsX_src:size()[1],1))
   goodLocationsY_src = cloneAndReturnMat(goodLocationsY_src:reshape(goodLocationsY_src:size()[1],1))
   goodLocationsX_dest = cloneAndReturnMat(goodLocationsX_dest:reshape(goodLocationsX_dest:size()[1],1))
   goodLocationsY_dest = cloneAndReturnMat(goodLocationsY_dest:reshape(goodLocationsY_dest:size()[1],1))
    local inliers, best_transformations = opencv.imgproc.findBestTransformation(goodLocationsX_src, goodLocationsY_src, 
      cloneAndReturnMat(scores_src_torch), cloneAndReturnMat(pairwise_dis_src:clone()),
      goodLocationsX_dest, goodLocationsY_dest, cloneAndReturnMat(scores_dest_torch), cloneAndReturnMat(pairwise_dis_dest),
      cloneAndReturnMat(angle_diff),
      parameters.corr_thresh, parameters.minInliersForMatch, parameters.maxNumCompute, 
      parameters.cornerDistanceLimit, minT[1], maxT[1], minT[2], maxT[2])
	inliers = inliers/math.min(scores_src_torch:size(1),scores_dest_torch:size(1))
   sort, order = torch.sort(inliers,true)
   transformations = {}
   for i =1, sort:size(1) do
      transformations[i] = best_transformations[order[i]]
   end
   print(sort)
   return sort, transformations;
end