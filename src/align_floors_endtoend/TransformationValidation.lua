TransformationValidation = Class()

function TransformationValidation.validate(best_pts, best_transformations, img_src_points, img_dest_points, parameters)

   local trans1_tmp = {}
   local trans2_tmp = {}
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
      --the closer to zero the better
      
      scores_metrics_temp.anglediff_tmp[i] = TransformationValidation.findAngleDifference(opencv.Mat.new(torch.gt(combined_tmp[i][1],0)), opencv.Mat.new(torch.gt(combined_tmp[i][2],0)))

      srci_mat =opencv.Mat.new(combined_i:clone():select(1,1))
      desi_mat =opencv.Mat.new(combined_i:clone():select(1,2))


      structElement = opencv.imgproc.getDefaultStructuringMat(2) 
      --opencv.imgproc.dilate(srci_mat, structElement);
      --opencv.imgproc.dilate(desi_mat, structElement);

      local srci = srci_mat:toTensor()
      srci:cdiv(srci:clone():add(0.0000000001)):ceil()
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
   scores_metrics.imgdiff = torch.Tensor(numRet)
   scores_metrics.ray = torch.Tensor(numRet,9)
   sorted,ordering = torch.sort(best_overlap)

   local i = table.getn(best_transformations)
   local k = 0
   print('validate: num transformations '..i)
   while k<numRet and i > 0 do

      local o = ordering[i]
      
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

function TransformationValidation.findMainDirections(img_src, img_dest)
   local HoughParameters = {}
   HoughParameters.houghRo = 1
   HoughParameters.houghTheta = math.pi/360
   HoughParameters.houghMinLineLength = 25
   HoughParameters.houghMaxLineGap = 80
   HoughParameters.minThreshold = 10
   HoughParameters.maxThreshold = 150
   HoughParameters.defaultThreshold = 75
   HoughParameters.numLinesDesired = 15
   parameterizedHough = align_floors_endtoend.Hough.new(HoughParameters)

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
   return angle_options_real
end
