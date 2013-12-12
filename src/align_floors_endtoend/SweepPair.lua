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
    'alignmentH_F', 'icpH_F', 'ficpH_F', 'floor_transformation_F', 'need_recalculating_icp', 'threed_validation_score', 'best_3d', 'score'}
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

local function select3d(from, selectPts)
    local normals_n = torch.zeros(torch.gt(selectPts,0):sum(),3)
    normals_n[{{},1}] = from[{{},1}][selectPts]
    normals_n[{{},2}] = from[{{},2}][selectPts]
    normals_n[{{},3}] = from[{{},3}][selectPts]
    return normals_n
end
--not returning combined,don't want ot keep track of dozens of images in memory...
function SweepPair:getValidationDiff(recalc)
  if not(self.ValidationInfoSimple) or recalc then
    local transformationInfo = self:getAllTransformations()
    local corners1, flattenedxy1, flattenedv1 = self:getSweep1():getFlattenedAndCorners()
    local corners2, flattenedxy2, flattenedv2 = self:getSweep2():getFlattenedAndCorners()

    local transformations, image_properties, scores_metrics
    local angle1 = self:getSweep1():getAxisAlignAngle()
    local angle2 = self:getSweep2():getAxisAlignAngle()

    transformations, imgdiff = 
    TransformationValidation.validate_simple(transformationInfo.inliers, 
      transformationInfo.transformations, flattenedxy1, flattenedxy2, angle1, angle2)

    local ValidationInfoSimple = {}

    ValidationInfoSimple["transformations"] = transformations
    ValidationInfoSimple["imgdiff"] = imgdiff
    self.ValidationInfoSimple = ValidationInfoSimple
    self:saveMe()
  end
  return self.ValidationInfoSimple
end

function SweepPair:cutBadAngles(recalc)
  if(not(self.best_3d) or recalc) then
    local transformations = self:getValidationDiff(recalc).transformations
    local okOptions = {}
    for i=1,math.min(3,table.getn(transformations)) do
        self:setBestTransformationH(transformations[i])
        local score1 = self:get3dValidationScore()
        if(score1[4]>.05) then
          self:getIcp(true)
          local score2 = self:get3dValidationScore()
          local score
          if(score1[1]+.5*score1[4] > score2[1]+.5*score2[4]) then
            score = score1
            self:setICPTransformation(torch.eye(4))
          else
            score = score2
          end
          if(score[1] > .85 and score[4]>.15) then
            self.best_3d = i
            self.score = score
            self:saveMe()
            return i
          end
        end
    end
    print "no options"
    self.best_3d=-1
    self:saveMe()
    return nil
  else
    if(self.best_3d == -1) then
      return nil
    else
      return self.best_3d, self.score
    end
  end
end

function SweepPair:setBest3DDiffTransformation(recalc)
  local i = self:cutBadAngles(recalc)
  if not(i) then
    self:setBestTransformationH(torch.eye(3))
    return false
  else
    self:setBestTransformationH((self:getValidationDiff()).transformations[i])
    return true
  end

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
    self:saveMe()     
  end
end

function SweepPair:getFinalIcp()
    self:setFinalICPTransformation(torch.eye(4))
    local t2 = self:getTransformation(false, false, false)
    local pc1_pts =  self:getSweep1():getPC().points
    local pc2_pts = self:getSweep2():getPC():get_global_points_H(t2)
    local transf, converged = self:getICPFromPoints(pc1_pts, pc2_pts)
    if(converged) then
      self:setFinalICPTransformation(transf)        
    end
    self:saveMe()
end

function SweepPair:getIcp(recalc)
  if(self.need_recalculating_icp) or recalc then
    local t2 = self:getTransformation(false, true, false)
    local pc1_pts =  self:getSweep1():getPC().points
    local pc2_pts = self:getSweep2():getPC():get_global_points_H(t2)
    local transf, converged = self:getICPFromPoints(pc1_pts, pc2_pts, .02, 5)
    if(converged) then
      self:setICPTransformation(transf)        
    end

    self.need_recalculating_icp = false
    self:saveMe()
  end
end

function SweepPair:getICPFromPoints(pc1_pts, pc2_pts, downsample_radius, icp_corr)
    downsample_radius = downsample_radius or .005
    icp_corr = icp_corr or 20
    local pc1 = pcl.PCLPointCloud.fromPoints(pc1_pts:contiguous())
    local pc2 = pcl.PCLPointCloud.fromPoints(pc2_pts:contiguous())
    local pc1_d = pc1:uniformSample(downsample_radius*self:getMeter())
    local pc2_d = pc2:uniformSample(downsample_radius*self:getMeter())

    print("Starting icp algorithm")
    log.tic()
    local transf, converged = pcl.PCLPointCloud.doICP(pc2_d, pc1_d, 
      self.parameters.icp_transform_eps, 
      self.parameters.icp_max_iterations, 
      self.parameters.icp_ransac_iterations,
      self.parameters.icp_correspondence*self:getMeter()/icp_corr) --starts w 10 cm, so now .5 cm
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
function SweepPair:get3dValidationScore(noticp, thresh_res, sweep_size)
  if self.threed_validation_score  == -1 or self.noticp ~= noticp or self.thresh_res ~=thresh_res then
    local sweep_size = sweep_size or 3
    self.thresh_res = thresh_res or 2
    self.noticp = noticp
    local H =self:getTransformation(false, noticp, false)
    local thresh_1 = self:getScale()*self.thresh_res--.01 -> 10 m, .02  -- equivalent to 2cm
    local f1,norm1 = self:getSweep1():getDepthImage(nil, nil, sweep_size)

    local f2, norm2 = self:getSweep2():getDepthImage(H, torch.zeros(3), sweep_size)
    local numRight1, portionSeen1, closePts1, nclosePts1 = SweepPair.findScore(f1, f2, norm1, norm2, thresh_1)
    center = H:sub(1,3,4,4):squeeze()
    local f2, norm2 = self:getSweep2():getDepthImage(H, center, sweep_size)
    local f1,norm1 = self:getSweep1():getDepthImage(nil, center, sweep_size)

    local numRight2, portionSeen2, closePts2, nclosePts2= SweepPair.findScore(f2, f1, norm2, norm1, thresh_1)

    print("first score: " .. numRight1 .. " second score: " .. numRight2)
    print("portion seen first  " .. portionSeen1 .. " and second : " .. portionSeen2)
    print("closet pts first  " .. closePts1 .. " and second : " .. closePts2)
    print("normal closet pts first  " .. nclosePts1 .. " and second : " .. nclosePts2)

    self.threed_validation_score = torch.Tensor({math.min(numRight1,numRight2),  math.min(portionSeen1,portionSeen2), 
      math.min(closePts1,closePts2), math.min(nclosePts1,nclosePts2)})
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

function SweepPair:setFinalICPTransformation(H)
  self.ficpH_F =H
  self:saveMe()
  collectgarbage()
end
function SweepPair:getFinalICPTransformation()
  return self.ficpH_F
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

function SweepPair:getMeter()
  return self:getSweep1():getPC().meter
end
function SweepPair:getScale()
  return self:getSweep1().scale
end
--first is whether we want sweep 1 or sweep 2
--dontUseICP is true if don't want to add icp
function SweepPair:getTransformation(first, dontUseICP, inverse) 
  local myTransformation
  if(first) then
    local sweepToUse = self:getSweep1()
    if(inverse) then
      sweepToUse = self:getSweep2()
    end  
    myTransformation = util_sweep.get2DTo3DTransformation(self:getScale(), 
      sweepToUse:getInitialTransformation(), 
      -sweepToUse:getFloor()) --bring floor to 0
    return myTransformation
  end
  if(dontUseICP) then
    myTransformation= util_sweep.get2DTo3DTransformation(self:getScale(), self:getAlignmentTransformation(), self:getZTransformation())
  else
      myTransformation= self:getICPTransformation()*util_sweep.get2DTo3DTransformation(self:getScale(), self:getAlignmentTransformation(), self:getZTransformation())
    if(self.ficpH_F) then
      myTransformation = self:getFinalICPTransformation()*myTransformation
    end
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
  local temp, temp, combined = util_sweep.warpAndCombine(H, flattenedxy1, flattenedxy2)
  image.display(combined)
end

function SweepPair:displayCurrent(icp)
  if(icp) then
    self:displayTransformationH(util_sweep.get3Dto2DTransformation(self:getScale(), self:getTransformation()))
  else
    self:displayTransformationH(self:getAlignmentTransformation())
  end
end

-----------------------------------------------------------------------------------------
--SAVE HELPERS
function SweepPair:saveCurrent(fname)
  self:saveCombinedTransformationH(fname, util_sweep.get3Dto2DTransformation(self:getScale(), self:getTransformation()))
end

function SweepPair:saveCombinedTransformationH(fname, H)
  local corners1, flattenedxy1, flattenedv1 = self:getSweep1():getFlattenedAndCorners()
  local corners2, flattenedxy2, flattenedv2  = self:getSweep2():getFlattenedAndCorners()
  local temp, temp, combined = util_sweep.warpAndCombine(H, flattenedxy1, flattenedxy2)
  image.save(fname, combined)
end

