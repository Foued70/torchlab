SweepPair = Class()
SweepPair.SWEEPPAIR = "SWEEPPAIR"
local colors = require '../opencv/types/Colors.lua'
local path = require 'path'
local FindTransformation = align_floors_endtoend.FindTransformation
local TransformationValidation = align_floors_endtoend.TransformationValidation
local Sweep = align_floors_endtoend.Sweep
local util_sweep = align_floors_endtoend.util

--transformation from sweep2 to sweep1
function SweepPair:__init(base_dir, sweep1, sweep2)
  if not(sweep1.__classname__ == Sweep.__classname__) or 
    not(sweep2.__classname__ == Sweep.__classname__) then
    error("Expected to be given a sweep for a sweep pair!")
  end
  if not(util.fs.is_dir(base_dir)) then
    error("expected base_dir to be directory in initializer for SweepPair")
  end
  self.sweep1 = sweep1
  self.sweep2 = sweep2
  self.need_recalculating_icp = true
  self.threed_validation_score  = -1
  self.threed_validation_score_nicp  = -1
  self.is_best_picked_set = -1
  self.name = sweep1:get_name() .. "_" .. sweep2:get_name()
  self.base_dir = base_dir
  self:mkdirs()
  self:init_variables()
  self:save_me()
end

function SweepPair.new_or_load(base_dir, sweep1, sweep2)
    if util.fs.is_file(path.join(base_dir, SweepPair.SWEEPPAIR, sweep1:get_name() .. "_" .. sweep2:get_name()..".dat")) then
      print("loading sweep pair from file " .. sweep1:get_name() .. "_" .. sweep2:get_name()..".dat")
      return torch.load(path.join(base_dir, SweepPair.SWEEPPAIR, sweep1:get_name() .. "_" .. sweep2:get_name()..".dat"))
    else
      return SweepPair.new(base_dir, sweep1, sweep2)
    end
  end
  function SweepPair:__write_keys()
    return {'is_best_picked_set', 'parameters', 'transformationInfo', 'ValidationInfo', 'name', 'thresh_res', 'thresh_res_icp', 'base_dir', 'fsave_me', 'sweep1_loc', 'sweep2_loc', 'ValidationInfoSimple',
    'alignmentH_F', 'icpH_F', 'floor_transformation_F', 'need_recalculating_icp', 'threed_validation_score', 'threed_validation_score_nicp', 'best_3d', 'score'}
  end

  function SweepPair:init_variables()
    self.parameters = {}
    --for transformation
    self.parameters.corr_thresh = 10
    self.parameters.minInliersForMatch =2
    self.parameters.maxNumCompute = 100
    self.parameters.cornerDistanceLimit =1

    --validation
    self.parameters.maxNumReturn = 25

    --icp
    self.parameters.icp_correspondence = .1
    self.parameters.icp_max_iterations = 100
    self.parameters.icp_ransac_iterations = 10000
    self.parameters.icp_transform_eps = 1e-6

    self.fsave_me = path.join(self.base_dir, SweepPair.SWEEPPAIR, self.name ..".dat")
    self.sweep1_loc = self:get_sweep1():get_save_location()
    self.sweep2_loc = self:get_sweep2():get_save_location()
    
    self.alignmentH_F = torch.eye(3)
    self.icpH_F = torch.eye(4)
    self.floor = 0
    self.floor_transformation_F = 0;
end

function SweepPair:get_sweep1()
  if not(self.sweep1) then
        self.sweep1 = torch.load(self.sweep1_loc)
  end
  return self.sweep1
end

function SweepPair:get_sweep2()
  if not(self.sweep2) then
        self.sweep2 = torch.load(self.sweep2_loc)
  end
  return self.sweep2
end

function SweepPair:mkdirs()
  util.fs.mkdir_p(path.join(self.base_dir, SweepPair.SWEEPPAIR))
end

function SweepPair:get_all_transformations(recalc)
  if not(self.transformationInfo) or recalc then
    local inliers, transformations = FindTransformation.findTransformation(self:get_sweep2(), self:get_sweep1(), self.parameters)
    local transformationInfo = {}
    transformationInfo["inliers"] = inliers
    transformationInfo["transformations"] = transformations
    self.transformationInfo = transformationInfo
    self:save_me()
  end
  return self.transformationInfo
end

function SweepPair:get_validated_transformations(recalc)
  if not(self.ValidationInfoSimple) or recalc then
    local transformationInfo = self:get_all_transformations()
    local corners1, flattenedxy1, flattenedv1, t,flatteneda1 = self:get_sweep1():get_flattened_and_corners()
    local corners2, flattenedxy2, flattenedv2,t,flatteneda2 = self:get_sweep2():get_flattened_and_corners()

    local transformations, image_properties, scores_metrics
    local angle1 = self:get_sweep1():get_axis_align_angle()
    local angle2 = self:get_sweep2():get_axis_align_angle()

    transformations, imgdiff = 
    TransformationValidation.validate_simple(transformationInfo.inliers, 
      transformationInfo.transformations, flattenedxy1, flattenedxy2, angle1, angle2, flatteneda1, flatteneda2)

    local ValidationInfoSimple = {}

    ValidationInfoSimple["transformations"] = transformations
    ValidationInfoSimple["imgdiff"] = imgdiff
    self.ValidationInfoSimple = ValidationInfoSimple
    self:save_me()
  end
  return self.ValidationInfoSimple
end

function SweepPair:get_validation(recalc)
  local combined
  if not(self.ValidationInfo) or recalc then
    local transformationInfo = self:get_all_transformations()
    local corners1, flattenedxy1, flattenedv1 = self:get_sweep1():get_flattened_and_corners()
    local corners2, flattenedxy2, flattenedv2 = self:get_sweep2():get_flattened_and_corners()
    local transformations, image_properties, scores_metrics
    transformations, combined, image_properties, scores_metrics = 
    TransformationValidation.validate(transformationInfo.inliers, 
      transformationInfo.transformations, flattenedxy1, flattenedxy2, self.parameters)

    local ValidationInfo = {}

    ValidationInfo["transformations"] = transformations
    ValidationInfo["image_properties"] = image_properties
    ValidationInfo["scores_metrics"] = scores_metrics
    self.ValidationInfo = ValidationInfo
    self:save_me()
  end
  return self.ValidationInfo, combined
end

function SweepPair:set_best_picked_diff_transformation(reset)
  if ((not(self.is_best_picked_set ==1) or reset) and self:get_validated_transformations().transformations) then
    local betsI=1
    local bestScore1 = -math.huge
    local bestScore4 = -math.huge
    local startingTransformation = self:get_sweep1():get_floor()-self:get_sweep2():get_floor()
    self:set_z_transformation(startingTransformation) 
    for i=1,math.min(10,table.getn(self:get_validated_transformations().transformations)) do
      self:set_alignment_transformation((self:get_validated_transformations()).transformations[i])
      self.threed_validation_score_nicp  = -1
      score = self:get_3d_validation_score(true, 10)
      if(bestScore1 < score[1] and (bestScore4 < score[4] or score[4]>.4)) then
        bestScore1 = score[1]
        bestScore4 = score[4]
        bestI = i
      end
    end
    self.threed_validation_score_nicp  = -1
    print("best i is", bestI)
    self:set_best_transformation((self:get_validated_transformations()).transformations[bestI], reset)
    self.is_best_picked_set = 1
    self:save_me()
  end
end
function SweepPair:set_best_diff_transformation(i, reset)
  if not(self:get_validated_transformations().transformations) then
    self:set_best_transformation(torch.eye(3))
  else
    self:set_best_transformation((self:get_validated_transformations()).transformations[i], reset)
  end
end


function SweepPair:set_inlier_transformation(i, reset)
  local vinliers, sinliers = torch.sort((self:get_all_transformations()).inliers)
  local i = sinliers[sinliers:size(1)-i+1]
  local bestTransformation = (self:get_all_transformations()).transformations[i]
  self:set_best_transformation(bestTransformation, reset)
end

--transformation from sweep2 to sweep1
function SweepPair:set_best_transformation(H, reset)
  if reset or ((torch.gt(torch.abs(H-self:get_alignment_transformation()), 10^-5):sum()>1)) then 
    --self:set_z_transformation(self:get_sweep1():get_floor()-self:get_sweep2():get_floor()) 
    self:set_icp_transformation(torch.eye(4))      
    self:set_alignment_transformation(H)
    self:find_best_floor(H)
    self.need_recalculating_icp = true
    self.threed_validation_score  = -1
    self.threed_validation_score_nicp  = -1
    self:save_me()     
  end
end

function SweepPair:find_best_floor(H)
    local startingTransformation = self:get_sweep1():get_floor()-self:get_sweep2():get_floor()
    local bestI = 0
    local bestIValue = math.huge*-1
    i=0
    self:set_z_transformation(startingTransformation + i*100) 
    self:set_icp_transformation(torch.eye(4))      
    self.threed_validation_score_nicp  = -1
    local bestSoFar = startingTransformation

    local score = self:get_3d_validation_score(true,10)
    if not(score[1]<.5 or score[4]<.03) then
      if not(score[1]>.9 and score[4]>.1) then
        for i = -5, 5 do
          if(i~=0) then
            self:set_z_transformation(startingTransformation + i*100) 
            self:set_icp_transformation(torch.eye(4))      
            self.threed_validation_score_nicp  = -1
            local score = self:get_3d_validation_score(true,10)
            if (score[1]+3*score[4] > bestIValue) then
              print("best i", 100*i, score[1], score[4])
              bestI = 100*i
              bestIValue = score[1]+3*score[4]
            end
          end
        end
        local shouldTryMore = false
        local startRange = 1 
        local endRange  = 1
        if(bestI == -500) then
          startRange = -20
          endRange = -6
          shouldTryMore = true
        elseif(bestI==500) then
          startRange = 6
          endRange = 20
          shouldTryMore = true
        end
        if(shouldTryMore) then
          print("expanding range")
          for i = startRange, endRange do
            if(i~=0) then
              self:set_z_transformation(startingTransformation + i*100) 
              self:set_icp_transformation(torch.eye(4))      
              self.threed_validation_score_nicp  = -1
              local score = self:get_3d_validation_score(true,10)
              if (score[1]+3*score[4] > bestIValue) then
                print("best i", 100*i, score[1], score[4])
                bestI = 100*i
                bestIValue = score[1]+3*score[4]
              end
            end
          end
        end
      end
      bestSoFar = startingTransformation + bestI
        self:set_z_transformation(bestSoFar) 

      local score = self:get_3d_validation_score(true,10)
      bestI = 0
      if not(score[1]<.6 or score[4]<.05) then
        for i = -2,2 do
          if(i~=0) then
            self:set_z_transformation(bestSoFar+ i*20) 
            self:set_icp_transformation(torch.eye(4))      
            self.threed_validation_score_nicp  = -1
            score = self:get_3d_validation_score(true,10)
            if (score[1]+3*score[4] > bestIValue)  then
              print("best i", 20*i, score[1], score[4])
              bestI = 20*i
              bestIValue = score[1]+3*score[4]
            end
          end
        end
      end
    end
    self.need_recalculating_icp = true
    self.threed_validation_score_nicp  = -1
    print("final i is ", bestSoFar+bestI)

    self:set_z_transformation(bestSoFar+bestI) 
    self:save_me()
end

function SweepPair:get_icp(recalc)
  if(self.need_recalculating_icp) or recalc then
    local t2 = self:get_transformation(false, true, false)

    local transf, converged = self:get_icp_from_points(t2, .02, 5)
    if(converged) then
      self:set_icp_transformation(transf)        
    end
    self.need_recalculating_icp = false
    self:save_me()
  end
end

function SweepPair:get_icp_from_points(t2, downsample_radius, icp_corr)
    downsample_radius = downsample_radius or .01
    icp_corr = icp_corr or 20
    local pc1_pts =  self:get_sweep1():get_pc():get_xyz_list()
    local pc2_pts = self:get_sweep2():get_pc():get_xyz_list_transformed(nil, t2)
    local pc1 = pcl.PCLPointCloud.fromPoints(pc1_pts:contiguous())
    local pc2 = pcl.PCLPointCloud.fromPoints(pc2_pts:contiguous())
    local pc1_d = pc1:uniformSample(downsample_radius*self:get_meter())
    local pc2_d = pc2:uniformSample(downsample_radius*self:get_meter())

    print("Starting icp algorithm")
    log.tic()
    local transf, converged = pcl.PCLPointCloud.doICP(pc2_d, pc1_d, 
      self.parameters.icp_transform_eps, 
      self.parameters.icp_max_iterations, 
      self.parameters.icp_ransac_iterations,
      self.parameters.icp_correspondence*self:get_meter()/icp_corr) --starts w 10 cm, so now .5 cm
    if(converged) then
      print("icp converged in time " .. log.toc() .. " with transformation:")
      print(transf)
    else
      print("icp did not converge in time " .. log.toc())
    end
    return transf, converged
end


--note: could make this more accurate by seeing if there is a closest neighbor closer than the one shot alone the ray..
--(e.g. if we are close to a wall, distance along ray will be very long even if the two walls are close in 3d space)
--this would involve some sort of kdtree or octtree, which if we are making anyway, we should use, but otherwise is 
--too slow if just for this
function SweepPair.find_score(f1, f2, n1, n2, thresh_1)
   mask = torch.ne(f2,0):cmul(torch.ne(f1,0))


   bottomSelectorN =torch.conv2(f1, torch.Tensor({0,1,0,0,0,0,0,0,0}):reshape(3,3),'F'):sub(2,-2,2,-2)
  topSelectorN =torch.conv2(f1, torch.Tensor({0,0,0,0,0,0,0,1,0}):reshape(3,3),'F'):sub(2,-2,2,-2)
   rightSelectorN = torch.conv2(f1, torch.Tensor({0,0,0,1,0,0,0,0,0}):reshape(3,3),'F'):sub(2,-2,2,-2)
  leftSelectorN = torch.conv2(f1, torch.Tensor({0,0,0,0,0,1,0,0,0}):reshape(3,3), 'F'):sub(2,-2,2,-2)

  selector_mask = bottomSelectorN:clone():cmul(topSelectorN):clone():cmul(rightSelectorN):clone():cmul(leftSelectorN)
  good_loc = torch.ne(selector_mask, 0)
   largest_neighbor = torch.cat(torch.cat(torch.cat(bottomSelectorN,topSelectorN,3),rightSelectorN,3),leftSelectorN,3):max(3):squeeze()-f1
  mask = mask:cmul(good_loc):cmul(torch.lt(largest_neighbor,thresh_1))
   f1_zeroed = f1:clone():cmul(mask:double())
   f2_zeroed = f2:clone():cmul(mask:double())

   close_pts = (torch.le(torch.abs(f1_zeroed-f2_zeroed),thresh_1):sum()- (torch.eq(torch.eq(f1_zeroed,0) + torch.eq(f2_zeroed,0),2):sum()))
   deg = util_sweep.angle_between(n1,n2):reshape(f1:size(1),f1:size(2))*180/math.pi
  deg:cmul(mask:double())  
  
  close_pts_normals = ((torch.le(torch.abs(f1_zeroed-f2_zeroed),thresh_1):cmul(torch.lt(deg,45))):cmul(mask):sum())
  return torch.le(f1_zeroed-f2_zeroed,thresh_1):cmul(mask):sum()/mask:sum(), 
          torch.ne(f1_zeroed,0):sum()/torch.gt(f1,0):sum(), 
          close_pts/torch.gt(f1_zeroed,0):sum(),
          close_pts_normals/mask:sum()
end


--sweep1's pc and sweep2's pc should already be in correct place
--threshold in cm
function SweepPair:get_3d_validation_score(noticp, thresh_res, H_i)
  local score
  if ((((self.threed_validation_score_nicp ==-1) or (self.thresh_res ~=thresh_res)) and noticp)
      or ((self.threed_validation_score  == -1 or (self.thresh_res_icp ~=thresh_res)) and not(noticp))) or H_i then
    local thresh_1
    if(not(H_i)) then
      if(noticp) then
        self.thresh_res = thresh_res or 2
        thresh_1 = self:get_scale()*self.thresh_res--.01 -> 10 m, .02  -- equivalent to 2cm
      else
        self.thresh_res_icp = thresh_res or 2
        thresh_1 = self:get_scale()*self.thresh_res_icp--.01 -> 10 m, .02  -- equivalent to 2cm
      end
    else
      thresh_1 = self:get_scale()*(thresh_res or 2)
    end
    local H =H_i or self:get_transformation(false, noticp, false)
    local f1,norm1 = self:get_sweep1():get_depth_image_from_perspective(nil)
    local f2, norm2 = self:get_sweep2():get_depth_image_from_perspective(H)
    _G.f1 = f1
    _G.f2 = f2
    _G.n1 = norm1
    _G.n2 = norm2
    local numRight1, portionSeen1, closePts1, nclosePts1 = SweepPair.find_score(f1, f2, norm1, norm2, thresh_1)
    center = H:sub(1,3,4,4):squeeze()
    local f2, norm2 = self:get_sweep2():get_depth_image_from_perspective(nil)
    local f1,norm1 = self:get_sweep1():get_depth_image_from_perspective(torch.inverse(H))


    local numRight2, portionSeen2, closePts2, nclosePts2= SweepPair.find_score(f2, f1, norm2, norm1, thresh_1)

    print("first score: " .. numRight1 .. " second score: " .. numRight2)
    print("portion seen first  " .. portionSeen1 .. " and second : " .. portionSeen2)
    print("closet pts first  " .. closePts1 .. " and second : " .. closePts2)
    print("normal closet pts first  " .. nclosePts1 .. " and second : " .. nclosePts2)
    local score = torch.Tensor({math.min(numRight1,numRight2),  math.min(portionSeen1,portionSeen2), 
          math.min(closePts1,closePts2), math.min(nclosePts1,nclosePts2)})
    if not(H_i) then
      if(noticp) then
        self.threed_validation_score_nicp = score
      else
          self.threed_validation_score = score
      end
      self:save_me()
    end
  else 
    local threed_validation_score
      if(noticp) then
        threed_validation_score = self.threed_validation_score_nicp
      else
        threed_validation_score = self.threed_validation_score
      end
        print("calculated score in the past ".. threed_validation_score[1] .. " " .. 
          threed_validation_score[2] .. " " .. threed_validation_score[3] .. " " ..
          threed_validation_score[4])
  end
  if (H_i) then
    return score
  elseif(noticp) then
      return self.threed_validation_score_nicp 
  else
    return self.threed_validation_score
  end
end

function SweepPair:save_me()
  torch.save(self.fsave_me, self)
end

--GET AND SET TRANSFORMATIONS
function SweepPair:set_alignment_transformation(H)
  self.alignmentH_F =H
  self:save_me()
  collectgarbage()
end

function SweepPair:get_alignment_transformation()
  return self.alignmentH_F 
end

function SweepPair:set_icp_transformation(H)
  self.icpH_F =H
  self.threed_validation_score = -1 
  self:save_me()
  collectgarbage()
end

function SweepPair:get_icp_transformation()
  return self.icpH_F
end

function SweepPair:set_z_transformation(transf) 
  self.floor_transformation_F = transf
  self:save_me()
end

function SweepPair:get_z_transformation() 
  return self.floor_transformation_F
end

function SweepPair:get_meter()
  return self:get_sweep1():get_pc():get_meter()
end
function SweepPair:get_scale()
  return self:get_sweep1().scale
end
--first is whether we want sweep 1 or sweep 2
--dontUseICP is true if don't want to add icp
function SweepPair:get_transformation(first, dontUseICP, inverse) 
  local myTransformation
  if(first) then
    local sweepToUse = self:get_sweep1()
    if(inverse) then
      sweepToUse = self:get_sweep2()
    end  
    myTransformation = util_sweep.get_2D_to_3D_transformation(self:get_scale(), 
      sweepToUse:get_initial_transformation(), 
      -sweepToUse:get_floor()) --bring floor to 0
    return myTransformation
  end
  if(dontUseICP) then
    myTransformation= util_sweep.get_2D_to_3D_transformation(self:get_scale(), self:get_alignment_transformation(), self:get_z_transformation())
  else
      myTransformation= self:get_icp_transformation()*util_sweep.get_2D_to_3D_transformation(self:get_scale(), self:get_alignment_transformation(), self:get_z_transformation())
  end
  if(inverse) then
    local t_rot = myTransformation:sub(1,3,1,3)
    local new_t = torch.eye(4)
    new_t[{{1,3},{4}}]=myTransformation:sub(1,3,4,4)*-1

    local new_rot = torch.eye(4)
    new_rot[2][1]=-myTransformation[2][1]
    new_rot[1][2]=-myTransformation[1][2]
    new_rot[1][1]=myTransformation[1][1]
    new_rot[2][2]=myTransformation[2][2]

    myTransformation = torch.inverse(myTransformation)--(new_t*new_rot):contiguous()
  end
  return myTransformation
end

-----------------------------------------------------------------------------------------
--display helper methods
function SweepPair:display(i, is_diff, is_inlier)
  --sweep1 and sweep2 will already have incorporated the best transformation, no need to save
  i=i or 1
  if(is_diff) then
    self:display_transformation((self:get_validated_transformations()).transformations[i])
    return self:get_validated_transformations().transformations[i]
  elseif(is_inlier) then
    local vinliers, sinliers = torch.sort((self:get_all_transformations()).inliers)
    local i = sinliers[sinliers:size(1)-i+1]
    local bestTransformation = (self:get_all_transformations()).transformations[i]
    self:display_transformation((self:get_all_transformations()).transformations[i])
    return self:get_all_transformations().transformations[i]
  end
end


function SweepPair:display_transformation(H)
  local corners1, flattenedxy1, flattenedv1 = self:get_sweep1():get_flattened_and_corners()
  local corners2, flattenedxy2, flattenedv2  = self:get_sweep2():get_flattened_and_corners()
  local temp, temp, combined = util_sweep.warp_and_combined(H, flattenedxy1, flattenedxy2)
  image.display(combined)
end

function SweepPair:display_current(icp)
  if(icp) then
    self:display_transformation(util_sweep.get_3D_to_2D_transformation(self:get_scale(), self:get_transformation()))
  else
    self:display_transformation(self:get_alignment_transformation())
  end
end

-----------------------------------------------------------------------------------------
--SAVE HELPERS
function SweepPair:save_current(fname)
  self:saveCombinedTransformationH(fname, util_sweep.get_3D_to_2D_transformation(self:get_scale(), self:get_transformation()))
end

function SweepPair:save_combined_transformation_H(fname, H)
  local corners1, flattenedxy1, flattenedv1 = self:get_sweep1():get_flattened_and_corners()
  local corners2, flattenedxy2, flattenedv2  = self:get_sweep2():get_flattened_and_corners()
  local temp, temp, combined = util_sweep.warp_and_combined(H, flattenedxy1, flattenedxy2)
  image.save(fname, combined)
end

