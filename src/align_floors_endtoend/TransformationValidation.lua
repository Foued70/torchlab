TransformationValidation = Class()

function TransformationValidation.validate_simple(best_pts, best_transformations, img_src_points, img_dest_points, nmlist1, nmlist2)
  local best_overlap = torch.Tensor(best_pts:size())
  local best_pts_temp = torch.Tensor(best_pts:size())

   local transformations_tmp = {}

   local center1, center2, combined_i = align_floors_endtoend.SweepPair.warpAndCombine(torch.eye(3), img_src_points, img_dest_points)
   --local angle_diff_orig = TransformationValidation.findMainDirections(opencv.Mat.new(torch.gt(combined_i[2],0):byte()), opencv.Mat.new(torch.gt(combined_i[1],0):byte()))
    local angle_diff_orig = math.pi-(TransformationValidation.findAngDiff(nmlist1:clone(),nmlist2:clone(),torch.eye(3),torch.eye(3)))
   local good_counter = 1
   for i=1,table.getn(best_transformations) do
      local center1, center2, combined_i = align_floors_endtoend.SweepPair.warpAndCombine(best_transformations[i], img_src_points, img_dest_points)
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
      local bestdiff = torch.min(torch.Tensor({math.pi/2, math.pi, 3*math.pi/2, 2*math.pi})-torch.abs(angle_diff_orig-angle_between))
        bestdiff = math.min(bestdiff, 2*math.pi-diff)

      if(bestdiff < math.rad(3)) then
         local srci_mat =opencv.Mat.new(combined_i:clone():select(1,1))
         local desi_mat =opencv.Mat.new(combined_i:clone():select(1,2))

         local srci = srci_mat:toTensor()
         local desi = desi_mat:toTensor()
         best_overlap[good_counter]=torch.eq(torch.gt(srci,0)+torch.gt(desi,0),2):sum()

         transformations_tmp[good_counter] = best_transformations[i]
         best_pts_temp[good_counter] = best_pts[i]
         good_counter = good_counter +1

      else
        print("skipping " .. i)
         --[[
         print("we didn't make it" .. i)
         if(i==20) then
            print(angle_diff_orig-angle_between)
            image.display(combined_i)
         end]]--
          --     print(best_transformations[i].H)
         --print(math.deg((torch.abs(torch.acos(best_transformations[i].H[1][1])-angle_diff_orig))))
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
      local center1, center2, combined_i = align_floors_endtoend.SweepPair.warpAndCombine(best_transformations[i], img_src_points, img_dest_points)
     
      image_properties_tmp[i] = {}
      image_properties_tmp[i].src_centers_h = center1[2]
      image_properties_tmp[i].src_centers_w = center1[1]
      image_properties_tmp[i].tgt_centers_h = center2[2]
      image_properties_tmp[i].tgt_centers_w = center2[1]

      combined_tmp[i] = combined_i
      
      scores_metrics_temp.anglediff_tmp[i] = TransformationValidation.findAngleDifference(opencv.Mat.new(torch.gt(combined_tmp[i][1],0)), opencv.Mat.new(torch.gt(combined_tmp[i][2],0)))

      local srci_mat =opencv.Mat.new(combined_i:clone():select(1,1))
      local desi_mat =opencv.Mat.new(combined_i:clone():select(1,2))

      --scores_metrics_temp.normaloverlap_tmp[i] = TransformationValidation.getNormalOverlap(srci_mat,desi_mat)


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


function TransformationValidation.findAngleDifference(img_src, img_dest)

	
   angle_options_real = TransformationValidation.findMainDirections(img_src, img_dest)

   return math.min(angle_options_real[1], math.pi/2-angle_options_real[1])
end

function TransformationValidation.findDirections(normalList)
  local nmp = normalList:sub(1,2):clone():cmul(normalList[3]:clone():abs():lt(0.25):type('torch.DoubleTensor'):repeatTensor(2,1,1))
  nmp:cmul(normalList:clone():pow(2):sum(1):squeeze():gt(0.25):type('torch.DoubleTensor'):repeatTensor(2,1,1))
  local nmp_norm = nmp:clone():pow(2):sum(1):sqrt():squeeze()
  nmp:cdiv(nmp_norm:repeatTensor(2,1,1))
  
  local acos = nmp[1]:clone():acos()
  local asin = nmp[2]:clone():asin()
  local thetas = acos:clone():cmul(asin:ge(0):type('torch.DoubleTensor')):add(
                 acos:clone():mul(-1):add(2*math.pi):cmul(asin:lt(0):type('torch.DoubleTensor')))
  
  dirMap = torch.histc(thetas,360,0,2*math.pi)
  
  return dirMap
  
end
function TransformationValidation.findAngDiff(nlist1,nlist2,trans1,trans2)

  local t1 = trans1:clone()
  t1:sub(1,2,3,3):fill(0)
  local t2 = trans2:clone()
  t2:sub(1,2,3,3):fill(0)
  
  local nl1 = t1:clone()*nlist1:transpose(1,2)
  local nl2 = t2:clone()*nlist2:transpose(1,2)
  
  local d,d1 = TransformationValidation.findDirections(nl1):max(1)
  local d,d2 = TransformationValidation.findDirections(nl2):max(1)
  diff = d2[1]-d1[1]
  if(diff < 0) then
    diff = diff + 180
  end
  if(diff > 180) then
    diff = diff -180
  end
  return math.rad(diff)
  --diff = math.abs(d2[1]-d1[1])
  --diff = math.min(diff, math.abs(90-diff), math.abs(180-diff), math.abs(270-diff), math.abs(360-diff))*math.pi/180

--  return diff
   
end

function TransformationValidation.findMainDirections(img_src, img_dest)
   local HoughParameters = {}
   HoughParameters.houghRo = 1
   HoughParameters.houghTheta = math.pi/360
   HoughParameters.houghMinLineLength = 5
   HoughParameters.houghMaxLineGap = 1
   HoughParameters.defaultThreshold = 75
   parameterizedHough = align_floors_endtoend.Hough.new(HoughParameters)

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
