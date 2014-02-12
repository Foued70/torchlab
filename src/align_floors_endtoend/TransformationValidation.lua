TransformationValidation = Class()
local util_sweep = align_floors_endtoend.util
function TransformationValidation.validate_simple(best_pts, best_transformations, img_src_points, img_dest_points, angle1, angle2, img_src_angles, img_dest_angles)
   local best_overlap = torch.Tensor(best_pts:size())
   local best_pts_temp = torch.Tensor(best_pts:size())

   local transformations_tmp = {}

   local center1, center2, combined_i = util_sweep.warp_and_combined(torch.eye(3), img_src_points, img_dest_points)

   local diff = angle2-angle1
   if(diff < 0) then
      diff = diff + 180
   end
   if(diff > 180) then
      diff = diff -180
   end

   local angle_diff_orig = math.rad(diff)
   local good_counter = 1
   for i=1,table.getn(best_transformations) do
      local center1, center2, combined_i, combined_a = util_sweep.warp_and_combined(best_transformations[i], img_src_points, img_dest_points, img_src_angles, img_dest_angles)

      local angle_between
      acos = torch.acos(best_transformations[i][1][1])
      asin = -torch.asin(best_transformations[i][1][2])
      local angle_between
      if(acos>=0 and asin>=0) then
         angle_between = acos
      elseif(acos>=0 and asin<0) then
         angle_between = math.pi*2-acos
      elseif(acos<0 and asin>=0) then
         angle_between = acos
      elseif(acos<0 and asin<0) then
         angle_between = math.pi*2-acos
      end
      --the closer to zero the better
      local bestdiff = torch.min(torch.abs(torch.Tensor({math.pi/2, math.pi, 3*math.pi/2, 2*math.pi, 0})-torch.abs(angle_diff_orig-angle_between)))
      --if(bestdiff < math.rad(3)) then
         local H=best_transformations[i]
         local angle_diff = combined_a[1]-combined_a[2]
         angle_diff[angle_diff:clone():lt(0)] = angle_diff[angle_diff:clone():lt(0)] +2*math.pi

         local diff_to_truth = (angle_diff-geom.util.get_angle_from_matrix(H)):abs()
         local diff_to_truth2 = ((angle_diff-geom.util.get_angle_from_matrix(H))*-1+2*math.pi):abs()
         local v,temp = torch.cat(diff_to_truth, diff_to_truth2,3):min(3)

         local srci_mat =opencv.Mat.new(combined_i:clone():select(1,1))
         local desi_mat =opencv.Mat.new(combined_i:clone():select(1,2))

         local srci = srci_mat:toTensor()
         local desi = desi_mat:toTensor()
         local score_angle =    ((v:squeeze()[torch.eq(torch.gt(srci,0)+torch.gt(desi,0),2)]*180/math.pi):lt(20):sum())
                    print("good transformation " .. i .. " " .. angle_between .. " " .. torch.eq(torch.gt(srci,0)+torch.gt(desi,0),2):sum() .. " " .. score_angle)

         if(score_angle > 100) then
            best_overlap[good_counter]=score_angle --torch.eq(torch.gt(srci,0)+torch.gt(desi,0),2):sum()

            transformations_tmp[good_counter] = best_transformations[i]
            best_pts_temp[good_counter] = best_pts[i]
            good_counter = good_counter +1
           print("good transformation " .. i .. " " .. angle_between .. " " .. torch.eq(torch.gt(srci,0)+torch.gt(desi,0),2):sum() .. " " .. score_angle)
        end
      --end
      collectgarbage()

   end
   if(good_counter == 1) then
      print("no transformations found!!!")
      return nil,nil
   end
   best_overlap = best_overlap:sub(1,good_counter-1)
   best_pts_temp = best_pts_temp:sub(1,good_counter-1)
   --best_overlap = best_overlap*2/(best_overlap:max()+0.0000001) + best_pts_temp

   sorted,ordering = torch.sort(best_overlap, true)
   transformations = {}
   for i=1,sorted:size(1) do
      transformations[i] = transformations_tmp[ordering[i]];
   end
   return transformations, sorted
end

function TransformationValidation.validate(best_pts, best_transformations, img_src_points, img_dest_points, parameters)
   local combined_tmp = {}
   local image_properties_tmp = {}
   local scores_metrics_temp = {}
   scores_metrics_temp.anglediff_tmp = {}

   local anglediff_tmp = torch.zeros(table.getn(best_transformations))
   
   local best_overlap = torch.Tensor(best_pts:size())
   for k=1,table.getn(best_transformations) do
      local i=k
      local center1, center2, combined_i = util_sweep.warp_and_combined(best_transformations[i], img_src_points, img_dest_points)
     
      image_properties_tmp[i] = {}
      image_properties_tmp[i].src_centers_h = center1[2]
      image_properties_tmp[i].src_centers_w = center1[1]
      image_properties_tmp[i].tgt_centers_h = center2[2]
      image_properties_tmp[i].tgt_centers_w = center2[1]

      combined_tmp[i] = combined_i
      
      scores_metrics_temp.anglediff_tmp[i] = util_sweep.find_angle_difference(opencv.Mat.new(torch.gt(combined_tmp[i][1],0)), opencv.Mat.new(torch.gt(combined_tmp[i][2],0)))

      local srci_mat =opencv.Mat.new(combined_i:clone():select(1,1))
      local desi_mat =opencv.Mat.new(combined_i:clone():select(1,2))

      local srci = srci_mat:toTensor()
      srci:cdiv(srci:clone():add(0.0000000001)):ceil()

      desi_mat =opencv.Mat.new(combined_i:clone():select(1,2))
      local desi = desi_mat:toTensor()
      desi:cdiv(desi:clone():add(0.0000000001)):ceil()
      best_overlap[i]=srci:clone():cmul(desi):sum()
      collectgarbage()
   end
   collectgarbage()
   local combined = {}
   local transformations = {}
   local image_properties = {}
   local scores_metrics = {}
   local numRet = math.min(table.getn(best_transformations),parameters.maxNumReturn)

   scores_metrics.anglediff = torch.zeros(numRet)
   scores_metrics.inliers = torch.zeros(numRet)
   scores_metrics.imgdiff = torch.zeros(numRet)
   scores_metrics.ray = torch.zeros(numRet,9)

   sorted,ordering = torch.sort(best_overlap)

   local i = table.getn(best_transformations)
   local k = 0
   print('validate: num transformations '..i)
   while k<numRet and i > 0 do

      local o = ordering[i]
      
      --if scores_metrics_temp.anglediff_tmp[o] <  parameters.rotation_thresh then
         k = k+1
         scores_metrics.inliers[k] = best_pts[o]
         scores_metrics.imgdiff[k] = 2*sorted[i]/(sorted:max()+0.0000001) + scores_metrics.inliers[k]
         image_properties[k] = image_properties_tmp[o]

         transformations[k] = best_transformations[o]
         combined[k] = combined_tmp[o]      
         --the closer to zero the better
         scores_metrics.anglediff[k] = scores_metrics_temp.anglediff_tmp[o]

         local sch = image_properties[k].src_centers_h
         local scw = image_properties[k].src_centers_w
         local tch = image_properties[k].tgt_centers_h
         local tcw = image_properties[k].tgt_centers_w
         local combcpy = combined[k]:clone()
         combcpy:apply(function(x)
            if x < (0.25 * 255) then
               return 0
            else
               return x
            end
            end)
         --local validation_scores = align_floors_endtoend.validation.compute_score(combcpy,sch,scw,tch,tcw)

         --scores_metrics.ray[k] = (validation_scores * 10000):ceil()
      --end
      i = i-1
      collectgarbage()

   end
   
   if k < numRet then
      scores_metrics.scores = scores_metrics.imgdiff:sub(1,k)
      scores_metrics.inliers = scores_metrics.inliers[{{1,k}}]
      scores_metrics.ray = scores_metrics.ray[{{1,k},{}}]

   end

   combined_tmp = nil
   image_properties_tmp = nil
   scores_metrics_temp = nil
   return transformations, combined, image_properties, scores_metrics
end