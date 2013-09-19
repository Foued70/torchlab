Sweep = Class()

Sweep.XYZ = "XYZ"
Sweep.PCD = "PCD"
Sweep.OD = "OD"
Sweep.CORNERS = "CORNERS"
Sweep.FLATTENED = "FLATTENED"
Sweep.TRANSFORM = "TRANSFORM"
Sweep.TRANSFORMED_XYZ = "TRANSFORMXYZ"

local path = require 'path'
local pcl = PointCloud.PointCloud
function Sweep:__init(base_dir, name)
	if not(path.extname(name) == "" or path.extname(name) == name) then
		error("expected name without extension to be given to Sweep initializer")
	end
	if not(util.fs.is_dir(base_dir)) then
		error("expected base_dir to exist for initializer for Sweep")
	end
	self.base_dir = base_dir
	self.name = name
	self.fod = path.join(self.base_dir, Sweep.OD, self.name.. ".od")
	self.fxyz = path.join(self.base_dir, Sweep.XYZ, self.name.. ".xyz")
	if not(util.fs.is_file(self.fxyz)) then
		error("expected sweep to exist in XYZ folder of base_dir " .. self.fxyz)
	end

	self.ftransform = path.join(self.base_dir, Sweep.TRANSFORM, self.name .. ".dat")

	self.fpcd = path.join(self.base_dir, Sweep.PCD, self.name .. ".pcd")
	self.fcorners = path.join(self.base_dir, Sweep.CORNERS, self.name .. ".dat")
	self.fflattened = path.join(self.base_dir, Sweep.FLATTENED, self.name .. ".dat")
	self.type = "Sweep"

	util.fs.mkdir_p(path.join(self.base_dir, Sweep.OD))

	local parameters = {}
	parameters.scale = .01
	parameters.numCorners = 30
	self.pproperties = path.join(self.base_dir, Sweep.OD, self.name .. "_properties.dat")
	torch.save(self.pproperties, parameters)

	util.fs.mkdir_p(path.join(self.base_dir, Sweep.XYZ))
	util.fs.mkdir_p(path.join(self.base_dir, Sweep.PCL))
	util.fs.mkdir_p(path.join(self.base_dir, Sweep.CORNERS))
	util.fs.mkdir_p(path.join(self.base_dir, Sweep.FLATTENED))
	util.fs.mkdir_p(path.join(self.base_dir, Sweep.PCD))
	util.fs.mkdir_p(path.join(self.base_dir, Sweep.Sweep.TRANSFORM))

end
function Sweep:getName()
	return self.name
end
function Sweep:getBaseDir()
	return self.base_dir
end

function Sweep:getPC()
	local pc
	if(util.fs.is_file(self.fod)) then
		pc = pcl.new(self.fod)
	else
		pc = pcl.new(self.fxyz)
		pc:write(self.fod)
		pc = pc
	end
	return pc
end

function Sweep:savePCD()
	if not(util.fs.is_file(self.fpcd)) then
		util.fs.exec("~/cloudlab/bin/read_xyz_write_pcd faro " .. self.fxyz .. " " .. self.fpcd)
	end
end
function Sweep:getPCDLocation()
	return self.fpcd
end

function Sweep:getFloor()
	if not(self.floor) then
		if util.fs.is_file(self.ftransform) then
			temp = 	torch.load(self.ftransform)
			self.alignmentH = temp[1]
			self.icpH = temp[2]
			self.floor = temp[3]
		else
			require 'gnuplot'
			local faro_points = self:getPC()
			local  y, i = torch.sort(faro_points.points:select(2, 3), 1);
			local sorted_face_centers = faro_points.points:index(1, i);

			local hist = gnuplot.histc(sorted_face_centers:select(2,3),80);
			local ycum = torch.cumsum(hist.raw, 1)
			local yh, ih = torch.sort(hist.raw, 1, true);

			local histmax, histmaxind = hist.raw:max();
			local histimg = hist.raw:resize(1, 80)
			local kernel = torch.Tensor({-0.25, -0.25, 1, -0.25, -0.25}):resize(1,5)
			local dominance = image.convolve(histimg, kernel, 'same'):resize(80)/histmax;
			a = torch.range(1, dominance:size(1))
			a = a[torch.gt(dominance,.07)]
			self.floor = hist[a[1]].val 
		end
	end
	return self.floor
end

function Sweep:getCeiling()
	require 'gnuplot'
	local faro_points = self:getPC()
	local  y, i = torch.sort(faro_points.points:select(2, 3), 1);
	local sorted_face_centers = faro_points.points:index(1, i);

	local hist = gnuplot.histc(sorted_face_centers:select(2,3),80);
	local ycum = torch.cumsum(hist.raw, 1)
	local yh, ih = torch.sort(hist.raw, 1, true);

	local histmax, histmaxind = hist.raw:max();
	local histimg = hist.raw:resize(1, 80)
	local kernel = torch.Tensor({-0.25, -0.25, 1, -0.25, -0.25}):resize(1,5)
	local dominance = image.convolve(histimg, kernel, 'same'):resize(80)/histmax;
	a = torch.range(1, dominance:size(1))
	a = a[torch.gt(dominance,.07)]
	return hist[a[a:size(1)]].val 
end
function Sweep:getTransformation()
	return geom.Homography.new(self:getICPTransformation().H*self:getAlignmentTransformation().H)
end

function Sweep:setAlignmentTransformation(H)
	self.alignmentH =H
	torch.save(self.ftransform, {self.alignmentH, self:getICPTransformation(), self:getFloor()})
end
function Sweep:setICPTransformation(H)
	self.icpH =H
	torch.save(self.ftransform, {self:getAlignmentTransformation(), self.icpH, self:getFloor()})
end

function Sweep:getAlignmentTransformation()
	if not(self.alignmentH) then
		if util.fs.is_file(self.ftransform) then
			temp = 	torch.load(self.ftransform)
			self.alignmentH = temp[1]
			self.icpH = temp[2]
			self.floor = temp[3]
		else
			self.alignmentH = geom.Homography.new(torch.eye(3))
		end
	end
	return self.alignmentH
end
function Sweep:getICPTransformation()
	if not(self.icpH) then
		if util.fs.is_file(self.ftransform) then
			temp = 	torch.load(self.ftransform)
			self.alignmentH = temp[1]
			self.icpH = temp[2]
			self.floor = temp[3]
		else
			self.icpH = geom.Homography.new(torch.eye(3))
		end
	end
	return self.icpH
end

local function get2DTransformationToZero(img)
   return geom.Homography.new(0,torch.Tensor({-img:size(1)/2, -img:size(2)/2}))
end

function Sweep:flattenAndCorners(global)
	if not(self.flattened and self.corners) then
		if (util.fs.is_file(self.fcorners) and util.fs.is_file(self.fflattened)) then
			self.corners = torch.load(self.fcorners)
			local flat = torch.load(self.fflattened)
			self.flattenedxy = flat[1]
			self.flattenedv = flat[2]
		else
			local p = self:getPC()
			local parameters = torch.load(self.pproperties)
			local flattened, corners = p:make_flattened_images(parameters.scale, nil, parameters.numCorners)
			corners = ((get2DTransformationToZero(flattened[1])):applyToPointsReturn2d(corners:t())):t()
			local flattenedx, flattenedy, flattenedv = image.thresholdReturnCoordinates(flattened[1],0)
			local flattenedxy = torch.cat(flattenedx, flattenedy,2)			
			flattenedxy = ((get2DTransformationToZero(flattened[1])):applyToPointsReturn2d(flattenedxy:t())):t()

			torch.save(self.fflattened,{flattenedxy, flattenedv})
			torch.save(self.fcorners,corners) 	--can optionally append more corners from Corners.lua here!

			self.corners = corners
			self.flattenedxy = flattenedxy
			self.flattenedv = flattenedv
		end
	end
	if(global) then
		return (self:getTransformation()):applyToPointsReturn2d(self.corners:t()):t(), 
			(self:getTransformation()):applyToPointsReturn2d(self.flattenedxy:t()):t(),	self.flattenedv 
	else
		return self.corners, self.flattenedxy, self.flattenedv
	end
end

function Sweep:transformIn3D()
	local parameters = torch.load(self.pproperties)
	local pc1 = self:getPC()
	local scale  = torch.eye(3)
   	scale[1][1] = 1/parameters.scale
   	scale[2][2] = 1/parameters.scale
   	
   	scale = geom.Homography.new(scale)

   	local iscale  = torch.eye(3)
   	iscale[1][1] = parameters.scale
   	iscale[2][2] = parameters.scale
   	
   	iscale = geom.Homography.new(iscale)
   	
   	local bestT = iscale:combineWith(self:getTransformation():combineWith(scale))
   	local rot = bestT.H --bestT:getEquivalentPCL()

   	local combined = torch.eye(4)
   	combined[{{1,2},{1,2}}] = rot[{{1,2},{1,2}}]
   	combined[1][4] = rot[1][3]
   	combined[2][4]= rot[2][3]

	local shift=  - self:getFloor(pc1)
   	combined[3][4] = shift

	local rotq  = geom.quaternion.from_rotation_matrix(combined)
	local trans = combined[{{1,3},4}]:clone()

	pc1:set_local_to_global_pose(trans)
	pc1:set_local_to_global_rot(rotq)
	pc1:write(self.fod)
end

function Sweep:axisAlign()
	
	local HoughParameters = {}
	HoughParameters.houghRo = 1
	HoughParameters.houghTheta = math.pi/360
	HoughParameters.houghMinLineLength = 25
	HoughParameters.houghMaxLineGap = 80
	HoughParameters.minThreshold = 10
	HoughParameters.maxThreshold = 150
	HoughParameters.defaultThreshold = 75
	HoughParameters.numLinesDesired = 15

	--initialize based on parameters
	local parameterizedHough = align_floors_endtoend.Hough.new(HoughParameters)

	Hough = align_floors_endtoend.Hough
	temp, flattenedxy = self:flattenAndCorners()
	local domAngle1_src,domAngle2_src = parameterizedHough:getMainDirections(opencv.Mat.new(Sweep.flattened2Image(flattenedxy):byte()))
	H = torch.eye(3)
	H[1][1] = torch.cos(domAngle1_src)
	H[2][2] = H[1][1]
	H[2][1] = -torch.sin(domAngle1_src)
	H[1][2] = -H[2][1]
	self:setAlignmentTransformation(geom.Homography.new(H))
end

function Sweep.flattened2Image(flattenedxy)
	local combinedMin = torch.min(flattenedxy,1)
	local minT = torch.min(combinedMin, 1)
	
	local combinedMax = torch.max(flattenedxy,1)
	local maxT = torch.max(combinedMax, 1)

	flattenedxy = flattenedxy-torch.repeatTensor(minT, flattenedxy:size(1), 1)+1
	
	local size_us = (maxT-minT+1):ceil():reshape(2)
	local combined = torch.zeros(size_us[1], size_us[2])
	for i = 1, flattenedxy:size(1) do
		combined[flattenedxy[i][1]][flattenedxy[i][2]]=1
	end
	return combined
end