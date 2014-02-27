TransformationValidation = Class()
local util_sweep = align_floors_endtoend.util
function TransformationValidation.validate_simple(best_pts, best_transformations, img_src_points, img_dest_points, angle1, angle2, img_src_angles, img_dest_angles)
   local best_overlap = torch.Tensor(best_pts:size())
   local best_pts_temp = torch.Tensor(best_pts:size())

   local transformations_tmp = {}

   local center1, center2, combined_i = util_sweep.warp_and_combined(torch.eye(3), img_src_points, img_dest_points)
   local good_counter = 1
   for i=1,table.getn(best_transformations) do
      local center1, center2, combined_i, combined_a = util_sweep.warp_and_combined(best_transformations[i], img_src_points, img_dest_points, img_src_angles, img_dest_angles)


      --the closer to zero the better
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
         if(score_angle > 100) then
            best_overlap[good_counter]=score_angle --torch.eq(torch.gt(srci,0)+torch.gt(desi,0),2):sum()

            transformations_tmp[good_counter] = best_transformations[i]
            best_pts_temp[good_counter] = best_pts[i]
            good_counter = good_counter +1
 --          print("good transformation " .. i .. " " .. angle_between .. " " .. torch.eq(torch.gt(srci,0)+torch.gt(desi,0),2):sum() .. " " .. score_angle)
        end
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

   sorted_vals,temp = torch.sort(ordering:sub(1,math.min(10,ordering:size(1))))

   print("ordering is", sorted_vals:max())
   transformations = {}
   for i=1,sorted:size(1) do
      transformations[i] = transformations_tmp[ordering[i]];
   end
   return transformations, sorted
end