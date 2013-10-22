SweepPair = Class()
SweepPair.SWEEPPAIR = "SWEEPPAIR"
local colors = require '../opencv/types/Colors.lua'
local path = require 'path'
FindTransformation = align_floors_endtoend.FindTransformation
TransformationValidation = align_floors_endtoend.TransformationValidation
Sweep = align_floors_endtoend.Sweep

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
  self.euc_validation_score  = -1
  self.name = sweep1:getName() .. "_" .. sweep2:getName()
  self.base_dir = base_dir
  self:mkdirs()
  self:init_variables()
  self:saveMe()
end

function SweepPair.newOrLoad(base_dir, sweep1, sweep2)
    if util.fs.is_file(path.join(base_dir, SweepPair.SWEEPPAIR, sweep1:getName() .. "_" .. sweep2:getName()..".dat")) then
      print("loading sweep pair from file " .. sweep1:getName() .. "_" .. sweep2:getName()..".dat")
      return torch.load(path.join(base_dir, SweepPair.SWEEPPAIR, sweep1:getName() .. "_" .. sweep2:getName()..".dat"))
    else
      return SweepPair.new(base_dir, sweep1, sweep2)
    end
  end
  function SweepPair:__write_keys()
    return {'parameters', 'transformationInfo', 'ValidationInfo', 'name', 'base_dir', 'fsave_me', 'sweep1_loc', 'sweep2_loc', 'ValidationInfoSimple',
    'alignmentH_F', 'icpH_F', 'floor_transformation_F', 'need_recalculating_icp', 'threed_validation_score', 'euc_validation_score'}
  end

  function SweepPair:init_variables()
    self.parameters = {}
    --for transformation
    self.parameters.corr_thresh = 2
    self.parameters.minInliersForMatch =2
    self.parameters.maxNumCompute = 100
    self.parameters.cornerDistanceLimit =1

    --validation
    self.parameters.maxNumReturn = 25
    self.parameters.rotation_thresh = 2*math.pi * (10/360)

    --icp
    self.parameters.icp_correspondence = .1
    self.parameters.icp_max_iterations = 100
    self.parameters.icp_ransac_iterations = 10000
    self.parameters.icp_transform_eps = 1e-6

    --euclidean validation score
    self.parameters.euc_valid_max_range = .2
    self.fsave_me = path.join(self.base_dir, SweepPair.SWEEPPAIR, self.name ..".dat")
    self.sweep1_loc = self:getSweep1():getSaveLocation()
    self.sweep2_loc = self:getSweep2():getSaveLocation()
    
    self.alignmentH_F = torch.eye(3)
    self.icpH_F = torch.eye(4)
    self.floor = 0
    self.floor_transformation_F = 0;
end

function SweepPair:getSweep1()
  if not(self.sweep1) then
        self.sweep1 = torch.load(self.sweep1_loc)
  end
  return self.sweep1
end

function SweepPair:getSweep2()
  if not(self.sweep2) then
        self.sweep2 = torch.load(self.sweep2_loc)
  end
  return self.sweep2
end

function SweepPair:mkdirs()
  util.fs.mkdir_p(path.join(self.base_dir, SweepPair.SWEEPPAIR))
end

function SweepPair:getAllTransformations(recalc)
  if not(self.transformationInfo) or recalc then
    local corners1, flattenedxy1, flattenedv1 = self:getSweep1():getFlattenedAndCorners()
    local corners2, flattenedxy2, flattenedv2 = self:getSweep2():getFlattenedAndCorners()

    local minT =torch.min(flattenedxy2,1):reshape(2)
    local maxT = torch.max(flattenedxy2,1):reshape(2)
    local inliers, transformations = FindTransformation.findTransformation(minT[1], maxT[1], minT[2], maxT[2], corners2, corners1, self.parameters)

    local transformationInfo = {}

    transformationInfo["inliers"] = inliers
    transformationInfo["transformations"] = transformations
    self.transformationInfo = transformationInfo
    self:saveMe()
  end
  return self.transformationInfo
end

--not returning combined,don't want ot keep track of dozens of images in memory...
function SweepPair:getValidationDiff(recalc)
  if not(self.ValidationInfoSimple) or recalc then
    local transformationInfo = self:getAllTransformations()
    local corners1, flattenedxy1, flattenedv1 = self:getSweep1():getFlattenedAndCorners()
    local corners2, flattenedxy2, flattenedv2 = self:getSweep2():getFlattenedAndCorners()
    local transformations, image_properties, scores_metrics
    local nmlist1 = self:getSweep1():getPC():get_normal_list()
    local nmlist2 = self:getSweep2():getPC():get_normal_list()

    transformations, imgdiff = 
    TransformationValidation.validate_simple(transformationInfo.inliers, 
      transformationInfo.transformations, flattenedxy1, flattenedxy2, nmlist1, nmlist2)

    local ValidationInfoSimple = {}

    ValidationInfoSimple["transformations"] = transformations
    ValidationInfoSimple["imgdiff"] = imgdiff
    self.ValidationInfoSimple = ValidationInfoSimple
    self:saveMe()
  end
  return self.ValidationInfoSimple
end

--not returning combined,don't want ot keep track of dozens of images in memory...
function SweepPair:getValidation(recalc)
  local combined
  if not(self.ValidationInfo) or recalc then
    local transformationInfo = self:getAllTransformations()
    local corners1, flattenedxy1, flattenedv1 = self:getSweep1():getFlattenedAndCorners()
    local corners2, flattenedxy2, flattenedv2 = self:getSweep2():getFlattenedAndCorners()
    local transformations, image_properties, scores_metrics
    transformations, combined, image_properties, scores_metrics = 
    TransformationValidation.validate(transformationInfo.inliers, 
      transformationInfo.transformations, flattenedxy1, flattenedxy2, self.parameters)

    local ValidationInfo = {}

    ValidationInfo["transformations"] = transformations
    ValidationInfo["image_properties"] = image_properties
    ValidationInfo["scores_metrics"] = scores_metrics
    self.ValidationInfo = ValidationInfo
    self:saveMe()
  end
  return self.ValidationInfo, combined
end

function SweepPair:setBestDiffTransformation(i)
  if not(self:getValidationDiff().transformations) then
    self:setBestTransformationH(torch.eye(3))
  else
    self:setBestTransformationH((self:getValidationDiff()).transformations[i])
  end
end

--[[]]
--gets the transformation in the highest inlier category with best angle diff
function SweepPair:setBestInlierAndAngle()
	
	local anglediff = 5/180*math.pi
	local vinliers, sinliers = torch.sort((self:getAllTransformations()).inliers,true)
	local transformations = self:getAllTransformations().transformations
	
	local nmlist1 = self:getSweep1():getPC():get_normal_list()
	local nmlist2 = self:getSweep2():getPC():get_normal_list()
	
	local satisfy = false
	local diff = 0
	
	local ct = #transformations
	local mindiff = 100
	local minct = 0
	local minsat = false
	
	--print(vinliers)
	
	while (true) do
	
	  if ct <= 0 then
		self:setInlierTransformation(#transformations-minct+1)
		print('NO GOOD CANDIDATE FOUND!!!')
		print(vinliers[minct],sinliers[minct])
		print(transformations[sinliers[minct] ])
		return
	  end
	
	  local H2 = transformations[sinliers[ct] ]
	    
	  diff = align_floors_endtoend.TransformationValidation.findAngDiff(nmlist1:clone(),nmlist2:clone(),torch.eye(3),H2)
	  print(diff,ct, sinliers[ct],vinliers[ct])
	  
	  if (diff < anglediff) then
    	satisfy = true
  	  else
  	    satisfy = false
  	  end
	  
	  if mindiff > diff then
	    print('mindiff > diff')
	    mindiff = diff
	    minct = ct
	    minsat = satisfy
	  end
	  
	  if minsat then
	    if ct == 1 or vinliers[ct-1] < vinliers[ct] then
	      print('next vinlier changes')
  	      break
  	    end
  	  end
  	  
	  if satisfy then
	    if vinliers[minct] > vinliers[ct] then
  	      print('minct was a different level')
  	      mindiff = diff
  	      minct = ct
  	    end
	  end
	  
	  ct = ct - 1
		
	end
	
	print('GOOD CANDIDATE FOUND!!!')
	print(mindiff, minct, sinliers[minct],vinliers[minct])
	print(transformations[sinliers[minct] ])
	self:setInlierTransformation(#transformations-minct+1)
end

function SweepPair:setInlierTransformation(i)
  local vinliers, sinliers = torch.sort((self:getAllTransformations()).inliers)
  local i = sinliers[sinliers:size(1)-i+1]
  local bestTransformation = (self:getAllTransformations()).transformations[i]
  self:setBestTransformationH(bestTransformation)
end

--transformation from sweep2 to sweep1
function SweepPair:setBestTransformationH(H)
  if(torch.gt(torch.abs(H-self:getAlignmentTransformation()), 10^-5):sum()>1) or 
    (torch.abs(self:getSweep1():getFloor()-self:getSweep2():getFloor() - self:getZTransformation()) > 10^-5) then 
    self:setAlignmentTransformation(H)
    self:setZTransformation(self:getSweep1():getFloor()-self:getSweep2():getFloor()) 
    self:setICPTransformation(torch.eye(4))      
    self.need_recalculating_icp = true
    self.threed_validation_score  = -1
    self.euc_validation_score = -1
    self:saveMe()     
  end
end

function SweepPair:getIcp(recalc)
  if(self.need_recalculating_icp) or recalc then
    local t = self:getTransformation(true,false, false)
    local t2 = self:getTransformation(false, true, false)
    local pc1_pts, pc1_rgb =       self:getSweep1():getPoints(t)
    local pc1 = pcl.PCLPointCloud.fromPoints(pc1_pts, pc1_rgb)
    local pc2_pts, pc2_rgb =self:getSweep2():getPoints(t*t2)
    local pc2 = pcl.PCLPointCloud.fromPoints(pc2_pts, pc2_rgb)

    print("Starting icp algorithm")
    log.tic()
    local transf, converged = pcl.PCLPointCloud.doICP(pc2, pc1, 
      self.parameters.icp_transform_eps, 
      self.parameters.icp_max_iterations, 
      self.parameters.icp_ransac_iterations,
      self.parameters.icp_correspondence*self:getScale()/.01)
    --converged = converged and not((torch.abs(transf[3][1]) > .05) or (torch.abs(transf[3][2]) > .05))
    if(converged) then
      transf[{3,{1,2}}]:fill(0)
      transf[{{1,2},3}]:fill(0)

      print("icp converged in time " .. log.toc() .. " with transformation:")
      print(transf)
      self:setICPTransformation(transf)        
      
    else
      print("icp did not converge in time " .. log.toc())
    end
    self.need_recalculating_icp = false
    self:saveMe()
  end
end

--note: could make this more accurate by seeing if there is a closest neighbor closer than the one shot alone the ray..
--(e.g. if we are close to a wall, distance along ray will be very long even if the two walls are close in 3d space)
--this would involve some sort of kdtree or octtree, which if we are making anyway, we should use, but otherwise is 
--too slow if just for this
function SweepPair.findScore(f1, f2, n1, n2, thresh_1)
  local mask = torch.ne(f2,0):cmul(torch.ne(f1,0))
  local f1_zeroed = f1:clone():cmul(mask:double())
  local close_pts = (torch.le(torch.abs(f1_zeroed-f2),thresh_1):sum()- (torch.eq(torch.eq(f1_zeroed,0) + torch.eq(f2,0),2):sum()))
  local dot = n1:clone():cmul(n2):sum(2):squeeze()
  local deg = (torch.acos(dot):squeeze():reshape(f1:size(1),f1:size(2))*180/math.pi)
  deg:cmul(mask:double())  
  local close_pts_normals = ((torch.le(torch.abs(f1_zeroed-f2),thresh_1):cmul(torch.lt(deg,45))):sum()- (torch.eq(torch.eq(f1_zeroed,0) + torch.eq(f2,0),2):sum()))
  return (torch.le(f1_zeroed-f2,thresh_1):sum()-torch.eq(f1_zeroed,0):sum())/(torch.gt(f1_zeroed,0):sum()), 
          torch.gt(f1_zeroed,0):sum()/torch.gt(f1,0):sum(), 
          close_pts/torch.gt(f1_zeroed,0):sum(),
          close_pts_normals/torch.gt(f1_zeroed,0):sum()
end

--sweep1's pc and sweep2's pc should already be in correct place
--threshold in cm
function SweepPair:get3dValidationScore(noticp, thresh_res)
  if self.threed_validation_score  == -1 or self.thresh_res ~=thresh_res then
    self.thresh_res = thresh_res or 2
    local H =self:getTransformation(false, noticp, false)
    local thresh_1 = self:getScale()*self.thresh_res--.01 -> 10 m, .02  -- equivalent to 2cm
    local f1,c1,pts1,norm1 = self:getSweep1():getDepthImage(nil, nil, 3)

    local f2,c2, pts2, norm2 = self:getSweep2():getDepthImage(H, torch.zeros(3), 3)
    local numRight1, portionSeen1, closePts1, nclosePts1 = SweepPair.findScore(f1, f2, norm1, norm2, thresh_1)

    local f2,c2,pts2, norm2 = self:getSweep2():getDepthImage(H, nil, 3)
    c2 = H:sub(1,3,4,4):squeeze()
    local f1,c1,pts1,norm1 = self:getSweep1():getDepthImage(nil, c2, 3)

    local numRight2, portionSeen2, closePts2, nclosePts2= SweepPair.findScore(f2, f1, norm2, norm1, thresh_1)

    print("first score: " .. numRight1 .. " second score: " .. numRight2)
    print("portion seen first  " .. portionSeen1 .. " and second : " .. portionSeen2)
    print("closet pts first  " .. closePts1 .. " and second : " .. closePts2)
    print("normal closet pts first  " .. nclosePts1 .. " and second : " .. nclosePts2)

    self.threed_validation_score = torch.Tensor({(numRight1+numRight2)/2,  (portionSeen1+portionSeen2)/2, 
      (closePts1 + closePts2)/2, (nclosePts1+nclosePts2)/2})
    self:saveMe()
  else 
    print("calculated score in the past ".. self.threed_validation_score[1] .. " " .. 
      self.threed_validation_score[2] .. " " .. self.threed_validation_score[3] .. " " ..
      self.threed_validation_score[4])
  end
  return self.threed_validation_score
end

function SweepPair:saveMe()
  torch.save(self.fsave_me, self)
end

--GET AND SET TRANSFORMATIONS
function SweepPair:setAlignmentTransformation(H)
  self.alignmentH_F =H
  self:saveMe()
  collectgarbage()
end

function SweepPair:getAlignmentTransformation()
  return self.alignmentH_F 
end

function SweepPair:setICPTransformation(H)
  self.icpH_F =H
  self.threed_validation_score  = -1 
  self:saveMe()
  collectgarbage()
end

function SweepPair:getICPTransformation()
  return self.icpH_F
end

function SweepPair:setZTransformation(transf) 
  self.floor_transformation_F = transf
  self:saveMe()
end

function SweepPair:getZTransformation() 
  return self.floor_transformation_F
end

function SweepPair:getScale()
  return self:getSweep1().scale
end
--first is whether we want sweep 1 or sweep 2
--dontUseICP is true if don't want to add icp
function SweepPair:getTransformation(first, dontUseICP, inverse) 
  collectgarbage()
  local myTransformation
  if(first) then
    local sweepToUse = self:getSweep1()
    if(inverse) then
      sweepToUse = self:getSweep2()
    end  
    myTransformation = Sweep.get2DTo3DTransformation(self:getScale(), 
      sweepToUse:getInitialTransformation(), 
      -sweepToUse:getFloor()) 
    return myTransformation
  end
  if(dontUseICP) then
    myTransformation= Sweep.get2DTo3DTransformation(self:getScale(), self:getAlignmentTransformation(), self:getZTransformation())
  else
    myTransformation= self:getICPTransformation()*Sweep.get2DTo3DTransformation(self:getScale(), self:getAlignmentTransformation(), self:getZTransformation())
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


--display helper methods
function SweepPair:displayDiffTransformationI(i)
  --sweep1 and sweep2 will already have incorporated the best transformation, no need to save
  self:displayTransformationH((self:getValidationDiff()).transformations[i])
  return self:getValidationDiff().transformations[i]
end


function SweepPair:displayInlierTransformationI(i)
  --sweep1 and sweep2 will already have incorporated the best transformation, no need to save
  local vinliers, sinliers = torch.sort((self:getAllTransformations()).inliers)
  local i = sinliers[sinliers:size(1)-i+1]
  local bestTransformation = (self:getAllTransformations()).transformations[i]
  self:displayTransformationH((self:getAllTransformations()).transformations[i])
  return self:getAllTransformations().transformations[i]
end

function SweepPair:displayTransformationH(H)
  local corners1, flattenedxy1, flattenedv1 = self:getSweep1():getFlattenedAndCorners()
  local corners2, flattenedxy2, flattenedv2  = self:getSweep2():getFlattenedAndCorners()
  local temp, temp, combined = SweepPair.warpAndCombine(H, flattenedxy1, flattenedxy2)
  image.display(combined)
end

function SweepPair:displayCurrent(icp)
  if(icp) then
    self:displayTransformationH(Sweep.get3Dto2DTransformation(self:getScale(), self:getTransformation()))
  else
    self:displayTransformationH(self:getAlignmentTransformation())
  end
end

--SAVE HELPERS
function SweepPair:saveCurrent(fname)
  self:saveCombinedTransformationH(fname, Sweep.get3Dto2DTransformation(self:getScale(), self:getTransformation()))
end

function SweepPair:saveCombinedTransformationH(fname, H)
  local corners1, flattenedxy1, flattenedv1 = self:getSweep1():getFlattenedAndCorners()
  local corners2, flattenedxy2, flattenedv2  = self:getSweep2():getFlattenedAndCorners()
  local temp, temp, combined = SweepPair.warpAndCombine(H, flattenedxy1, flattenedxy2)
  image.save(fname, combined)
end

--warps 2 into 1 using H
function SweepPair.warpAndCombine(H, flattenedxy1, flattenedxy2)
  flattenedxy2 = Sweep.applyToPointsReturn2d(H,flattenedxy2:t()):t()
  local src_center =    Sweep.applyToPointsReturn2d(H,torch.zeros(2,1))
  local dest_center = torch.zeros(2,1)
  local combinedMin = torch.cat(torch.min(flattenedxy1,1), torch.min(flattenedxy2,1), 1)
  local minT = torch.min(combinedMin, 1):reshape(2)

  local combinedMax = torch.cat(torch.max(flattenedxy1,1), torch.max(flattenedxy2,1), 1)
  local maxT = torch.max(combinedMax, 1):reshape(2)

  local H_translation = Sweep.getRotationMatrix(0,torch.Tensor({-minT[1]+1, -minT[2]+1}))

  flattenedxy1 = Sweep.applyToPointsReturn2d(H_translation, flattenedxy1:t()):t()
  flattenedxy2 = Sweep.applyToPointsReturn2d(H_translation,flattenedxy2:t()):t()

  local size_us = (maxT-minT+1):ceil():reshape(2)
  local su1 = size_us[1]
  local su2 = size_us[2]
  local combined = torch.zeros(3, su1, su2)


  for i = 1, flattenedxy2:size(1) do
  local f = flattenedxy2[i]
  local hh = f[1]
  local ww = f[2]
  local hh_l = math.max(1,math.floor(hh))
  local ww_l = math.max(1,math.floor(ww))
  local hh_h = math.min(su1,math.ceil(hh))
  local ww_h = math.min(su2,math.ceil(ww))

  if hh_h - hh < hh - hh_l then
    hh = hh_l
  else
    hh = hh_h
  end

  if ww_h - ww < ww - hh_l then
    ww = ww_l
  else
    ww = ww_h
  end

  combined[2][hh][ww]=255
  end
  for i = 1, flattenedxy1:size(1) do
  local f = flattenedxy1[i]
  local hh = f[1]
  local ww = f[2]
  local hh_l = math.max(1,math.floor(hh))
  local ww_l = math.max(1,math.floor(ww))
  local hh_h = math.min(su1,math.ceil(hh))
  local ww_h = math.min(su2,math.ceil(ww))

  if hh_h - hh < hh - hh_l then
    hh = hh_l
  else
    hh = hh_h
  end

  if ww_h - ww < ww - hh_l then
    ww = ww_l
  else
    ww = ww_h
  end

  combined[1][hh][ww]=255
  end
  src_center = Sweep.applyToPointsReturn2d(H_translation,src_center):reshape(2)
  dest_center = Sweep.applyToPointsReturn2d(H_translation,dest_center):reshape(2)
  return src_center, dest_center, combined
end
