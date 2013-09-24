SweepPair = Class()
SweepPair.COMBINED = "COMBINED"
SweepPair.TRANSFORM = "TRANSFORM_ALL"
SweepPair.VALIDATION = "VALIDATION"
SweepPair.ICP = "ICP"
SweepPair.ALIGNED = "ALIGNED"

local path = require 'path'
function SweepPair:__init(base_dir, sweep1, sweep2, steps_from_orig, forward)
    if not(sweep1.type == "Sweep") or not(sweep2.type == "Sweep") then
        error("Expected to be given a sweep for a sweep pair!")
    end
    if not(util.fs.is_dir(base_dir)) then
        error("expected base_dir to be directory in initializer for SweepPair")
    end
    self.sweep1 = sweep1
    self.sweep2 = sweep2
    if forward then
        self.forward = true
    else
        self.forward = false
    end
    
    self.steps_from_orig = steps_from_orig
    
    self.name = sweep1:getName() .. "_" .. sweep2:getName()
    self.base_dir = base_dir
    self.ftransformation = path.join(self.base_dir, SweepPair.TRANSFORM, self.name .. ".dat")
    self.fvalidation = path.join(self.base_dir, SweepPair.VALIDATION, self.name .. ".dat")
    self.faligned_pcd = path.join(self.base_dir, SweepPair.ALIGNED, self.name .. ".pcd")
    self.faligned_h = path.join(self.base_dir, SweepPair.ALIGNED, self.name .. ".txt")
    self.ficp_pcd = path.join(self.base_dir, SweepPair.ICP, self.name .. ".pcd")
    self.ficp_h = path.join(self.base_dir, SweepPair.ICP, self.name .. ".txt")

    util.fs.mkdir_p(path.join(self.base_dir, SweepPair.TRANSFORM))
    util.fs.mkdir_p(path.join(self.base_dir, SweepPair.VALIDATION))
    util.fs.mkdir_p(path.join(self.base_dir, SweepPair.ICP))
    util.fs.mkdir_p(path.join(self.base_dir, SweepPair.ALIGNED))

    local parameters = {}
    parameters.corr_thresh = 2
    parameters.minInliersForMatch =2
    parameters.maxNumCompute = 50
    parameters.cornerDistanceLimit =1
    self.fTParameters = path.join(self.base_dir, SweepPair.TRANSFORM, self.name .. '_' .. 'properties.dat')
    torch.save(self.fTParameters, parameters)

    local parameters = {}
    parameters.warpWithBorders = false
    parameters.maxNumReturn = 25
    parameters.rotation_thresh = 2*math.pi * (4.5/360)
    self.fVParameters = path.join(self.base_dir, SweepPair.VALIDATION, self.name .. '_' .. 'properties.dat')
    torch.save(self.fVParameters, parameters)


    local parameters = {}
    parameters.scale = .01
    parameters.icp_correspondence = .2
    parameters.icp_max_iterations = 100
    parameters.icp_ransac_iterations = 10000
    parameters.icp_transform_eps = 1e-6
    self.fIParameters = path.join(self.base_dir, SweepPair.ICP, self.name .. '_' .. 'properties.dat')
    torch.save(self.fIParameters, parameters)

end

function SweepPair:getAllTransformations()
    if not(self.transformationInfo) then
        if util.fs.is_file(self.ftransformation) then
            self.transformationInfo = torch.load(self.ftransformation)
        else                
            local corners1, flattenedxy1, flattenedv1
            if self.forward then
                corners1, flattenedxy1, flattenedv1 = self.sweep1:flattenAndCorners(true,true)
            else
                corners1, flattenedxy1, flattenedv1 = self.sweep1:flattenAndCorners(true,false)
            end
            
            local corners2, flattenedxy2, flattenedv2 = self.sweep2:flattenAndCorners()
            local parameters = torch.load(self.fTParameters)
            
            local minT =torch.min(flattenedxy2,1):reshape(2)
            local maxT = torch.max(flattenedxy2,1):reshape(2)
            local inliers, transformations = align_floors_endtoend.FindTransformation.findTransformation(minT[1], maxT[1], minT[2], maxT[2], corners2, corners1, parameters)
            
            local transformationInfo = {}
            
            transformationInfo["inliers"] = inliers
            transformationInfo["transformations"] = transformations
            
            transformationInfo["parameters"] = parameters
            torch.save(self.ftransformation,transformationInfo)
            self.transformationInfo = transformationInfo
        end
    end
    return self.transformationInfo
end

function SweepPair:doValidation()
    if true then --not(self.ValidationInfo) then
        if false then -- util.fs.is_file(self.fvalidation) then
            self.ValidationInfo = torch.load(self.fvalidation)
        else
            local transformationInfo = self:getAllTransformations()
            local corners1, flattenedxy1, flattenedv1
            if self.forward then
                corners1, flattenedxy1, flattenedv1 = self.sweep1:flattenAndCorners(true,true)
            else
                corners1, flattenedxy1, flattenedv1 = self.sweep1:flattenAndCorners(true,false)
            end
            local corners2, flattenedxy2, flattenedv2 = self.sweep2:flattenAndCorners()
            local parameters = torch.load(self.fVParameters)
            
            local transformations, combined, image_properties, scores_metrics = 
                align_floors_endtoend.TransformationValidation.validate(transformationInfo.inliers, 
                transformationInfo.transformations, flattenedxy2, flattenedxy1, parameters)
                
            local ValidationInfo = {}

            ValidationInfo["transformations"] = transformations
            ValidationInfo["combined"] = combined
            ValidationInfo["image_properties"] = image_properties
            ValidationInfo["scores_metrics"] = scores_metrics
            
            ValidationInfo["parameters"] = parameters
            --torch.save(self.fvalidation, ValidationInfo)
            self.ValidationInfo = ValidationInfo
        end
    end
    return self.ValidationInfo
end

function SweepPair:setBestDiffTransformation(i)

	local bestTransformation = (self:doValidation()).transformations[i]
    if self.forward then
       	self.sweep2:setAlignmentTransformation(bestTransformation, self.steps_from_orig, true)
    else    
    	self.sweep2:setAlignmentTransformation(bestTransformation, self.steps_from_orig, false)
	end
        
end

function SweepPair:setBestTransformation()
	
	local parameters = torch.load(self.fVParameters)
	local anglediff = parameters.rotation_thresh
	local valid = self:doValidation()
	local transformations = valid.transformations
	local scores_metric = valid.scores_metrics.anglediff
	local ct = 1
	while scores_metric[ct] > anglediff do
		ct = ct + 1
		if ct > scores_metric:size(1) then
			self:setBestDiffTransformation(1)
			print('NO GOOD CANDIDATE FOUND!!!')
			print(scores_metric[1])
			return
		end
	end
	self:setBestDiffTransformation(ct)
	
end

function SweepPair:setInlierTransformation(i)

    local vinliers, sinliers = torch.sort((self:getAllTransformations()).inliers)
    local i = sinliers[sinliers:size(1)-i+1]
    local bestTransformation = (self:getAllTransformations()).transformations[i]
    
    if self.forward then
        self.sweep2:setAlignmentTransformation(bestTransformation, self.steps_from_orig, true)
    else
        self.sweep2:setAlignmentTransformation(bestTransformation, self.steps_from_orig, false)
    end

end

function SweepPair:setBestTransformationH(H)

    if self.forward then
    	print('setting forward trans')
    	print(H)
        self.sweep2:setAlignmentTransformation(H, self.steps_from_orig, true)
    else
	    print('setting backward trans')
	    print(H)
        self.sweep2:setAlignmentTransformation(H, self.steps_from_orig, false)
    end

end

function SweepPair:doIcp2d()
    
    local corners1, flattenedxy1, flattenedv1
    if self.forward then
        self.sweep2:setICPTransformation(geom.Homography.new(torch.eye(3)),self.steps_from_orig,true)
        corners1, flattenedxy1, flattenedv1 = self.sweep1:flattenAndCorners(true,true)
    else
        self.sweep2:setICPTransformation(geom.Homography.new(torch.eye(3)),self.steps_from_orig,false)
        corners1, flattenedxy1, flattenedv1 = self.sweep1:flattenAndCorners(true,false)
    end
        
    local corners2, flattenedxy2, flattenedv2  = self.sweep2:flattenAndCorners()
    _G.flattenedxy1 = flattenedxy1
    _G.flattenedxy2 = flattenedxy2
    H_icp= align_floors_endtoend.ICP2D.align(flattenedxy2, flattenedxy1)
    
    if self.forward then
        self.sweep2:setICPTransformation(H_icp, self.steps_from_orig, true)
    else
        self.sweep2:setICPTransformation(H_icp, self.steps_from_orig, false)
    end

end

--[[
function SweepPair:doIcp()
    points1= self.sweep1:transformIn3D()
    points2= self.sweep2:transformIn3D()
    H_icp= align_floors_endtoend.ICP2D.align(points1, points2)


--  self.sweep1:setICPTransformation(geom.Homography.new(torch.eye(3)))

--
    homography = self:getBestTransformation()
    local parameters = torch.load(self.fIParameters)
    local sweep_1_pcd =self.sweep1:getPCDLocation()
    local sweep_2_pcd = self.sweep2:getPCDLocation()

    local zShift = self.sweep2:getFloor()-self.sweep1:getFloor()
    self.sweep1:savePCD()
    self.sweep2:savePCD()
    align_floors_endtoend.Icp.alignAndICP(sweep_1_pcd, sweep_2_pcd, homography, self.faligned_pcd, 
        self.faligned_h, self.ficp_pcd, self.ficp_h, zShift, parameters)
--
end
--[[]]

function SweepPair:displayTransformation(i)
    --sweep1 and sweep2 will already have incorporated the best transformation, no need to save
    self:displayResultH((self:getAllTransformations()).transformations[i])
end

function SweepPair:displayResultH(H)
    local corners1, flattenedxy1, flattenedv1
    if self.forward then
        corners1, flattenedxy1, flattenedv1 = self.sweep1:flattenAndCorners(true,true)
    else
        corners1, flattenedxy1, flattenedv1 = self.sweep1:flattenAndCorners(true,false)
    end
    local corners2, flattenedxy2, flattenedv2  = self.sweep2:flattenAndCorners()
    local temp, temp, combined = SweepPair.warpAndCombine(H, flattenedxy1, flattenedxy2)
    image.display(combined)
end

function SweepPair:displayResult()
    local corners1, flattenedxy1, flattenedv1
    local corners2, flattenedxy2, flattenedv2
    if self.forward then
        corners1, flattenedxy1, flattenedv1 = self.sweep1:flattenAndCorners(true,true)
        corners2, flattenedxy2, flattenedv2 = self.sweep2:flattenAndCorners(true,true)
    else
        corners1, flattenedxy1, flattenedv1 = self.sweep1:flattenAndCorners(true,false)
        corners2, flattenedxy2, flattenedv2 = self.sweep2:flattenAndCorners(true,false)
    end
    local H = geom.Homography.new(0,torch.Tensor({0,0}))
    local temp, temp, combined = SweepPair.warpAndCombine(H, flattenedxy1, flattenedxy2)
    image.display(combined)
end

function SweepPair.warpAndCombine(H, flattenedxy1, flattenedxy2)
   flattenedxy2 = H:applyToPointsReturn2d(flattenedxy2:t()):t()
   local src_center =    H:applyToPointsReturn2d(torch.zeros(2,1))
   local dest_center = torch.zeros(2,1)
   local combinedMin = torch.cat(torch.min(flattenedxy1,1), torch.min(flattenedxy2,1), 1)
   local minT = torch.min(combinedMin, 1):reshape(2)

   local combinedMax = torch.cat(torch.max(flattenedxy1,1), torch.max(flattenedxy2,1), 1)
   local maxT = torch.max(combinedMax, 1):reshape(2)

   local H_translation = geom.Homography.new(0,torch.Tensor({-minT[1]+1, -minT[2]+1}))

   flattenedxy1 = H_translation:applyToPointsReturn2d(flattenedxy1:t()):t()
   flattenedxy2 = H_translation:applyToPointsReturn2d(flattenedxy2:t()):t()
   
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
      
      combined[1][hh][ww]=255
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
      
      combined[2][hh][ww]=255
   end
   src_center = H_translation:applyToPointsReturn2d(src_center):reshape(2)
   dest_center = H_translation:applyToPointsReturn2d(dest_center):reshape(2)
   return src_center, dest_center, combined
end