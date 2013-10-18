Sweep = Class()

Sweep.XYZ = "XYZ"
Sweep.PNG = "PNG"
Sweep.OD = "OD"
Sweep.SWEEP = "SWEEP"

Hough = align_floors_endtoend.Hough
local path = require 'path'
local pcl = PointCloud.PointCloud

function Sweep:__init(base_dir, name)
    if not(path.extname(name) == "" or path.extname(name) == name) then
        error("expected name without extension to be given to Sweep initializer")
    end
    if not(util.fs.is_dir(base_dir)) then
        error("expected base_dir to exist for initializer for Sweep")
    end
    if not(util.fs.is_file(path.join(base_dir, Sweep.XYZ, name .. ".xyz"))) then
        error("expected sweep to exist in XYZ folder of base_dir " .. path.join(base_dir, Sweep.XYZ, name.. ".xyz"))
    end
    self.base_dir = base_dir
    self.name = name
    self:mkdirs()
    self:init_variables()
    self:saveMe()
end

function Sweep.newOrLoad(base_dir, name)
    if(util.fs.is_file(path.join(base_dir, Sweep.SWEEP, name .. ".dat"))) then
        print("loading sweep from file " .. name)
        return torch.load(path.join(base_dir, Sweep.SWEEP, name .. ".dat"))
    else
        return Sweep.new(base_dir, name)
    end
end
function Sweep:mkdirs()
    util.fs.mkdir_p(path.join(self.base_dir, Sweep.OD))
    util.fs.mkdir_p(path.join(self.base_dir, Sweep.SWEEP))
end

function Sweep:init_variables()
    self.parameters = {}
    self.parameters.scale = 10 -- in milimeters
    self.parameters.numCorners = 30
    self.parameters.octTreeRes = .02
    self.initial = torch.eye(3)
    self.fod = path.join(self.base_dir, Sweep.OD, self.name.. ".od")
    self.fxyz = path.join(self.base_dir, Sweep.XYZ, self.name.. ".xyz")
    self.fpng = path.join(self.base_dir,Sweep.PNG, self.name..".png")
    self.fsave_me = path.join(self.base_dir, Sweep.SWEEP, self.name .. ".dat")
end

function Sweep:__write_keys()
  return {'base_dir', 'name', 'parameters', 
    'initial', 'floor',
    'corners', 'flattenedxy', 'flattenedv', 
    'fod', 'fxyz', 'fpng', 'fsave_me',
    }
end

function Sweep:getName()
    return self.name
end
function Sweep:getBaseDir()
    return self.base_dir
end
function Sweep:getSaveLocation()
    return self.fsave_me
end

function Sweep:getPC(H)
    local pc
    local rgb
    if (util.fs.is_file(self.fod)) then
        pc = pcl.new(self.fod)
    else
        pc = pcl.new(self.fxyz)
        if util.fs.is_dir(path.join(self.base_dir,Sweep.PNG)) and util.fs.is_file(self.fpng) then
            pc:load_rgb_map(self.fpng)
        end
        pc:save_to_od(self.fod)
        pc = pc
    end
    collectgarbage()
    if (H) then
        pc:set_pose_from_rotation_matrix(H)
        collectgarbage()
        pc_new, faro_estimate_new= pc:get_global_points_as_pc(), pc:estimate_global_faro_pose()
        pc:set_pose_from_rotation_matrix(torch.eye(4))
        return pc_new, faro_estimate_new        
    else
        return pc, pc:estimate_faro_pose()
    end
end

function Sweep:getNormalMap(H)
    collectgarbage()
    local pc = self:getPC()
    local idx, mask = pc:get_index_and_mask()
    mask = mask*-1+1
    --mask = idx:cmul(mask:long()):reshape(mask:size(1)*mask:size(2))  
    --mask = mask[torch.gt(mask,0)]
    if (H) then
        local H_new = torch.eye(4)
        H_new[{{1,2},{1,2}}] = H:sub(1,2,1,2)
        pc:set_pose_from_rotation_matrix(H_new)
        collectgarbage()
        normal = pc:get_global_normal_map()
        pc:set_pose_from_rotation_matrix(torch.eye(4))
        return normal, mask    
    else
        return pc:get_normal_map(), mask
    end
end
local function select3d(from, selectPts)
    local normals_n = torch.zeros(torch.gt(selectPts,0):sum(),3)
    normals_n[{{},1}] = from[{{},1}][selectPts]
    normals_n[{{},2}] = from[{{},2}][selectPts]
    normals_n[{{},3}] = from[{{},3}][selectPts]
    return normals_n
end

local function select3d2(from, selectPts, normals_n)
    normals_n[{{},1}][selectPts] = from[{{},1}]
    normals_n[{{},2}][selectPts] = from[{{},2}]
    normals_n[{{},3}][selectPts] = from[{{},3}]
    return normals_n
end
--this is for faro, that goes -60 to 90 degrees
function Sweep:getDepthImage(H, center, res, res2)
    res = res or 1
    local p, center_n= self:getPC(H)
    center_n = center or center_n
    local pts = (p.points-center_n:reshape(1,3):repeatTensor(p.points:size(1),1))
    local normals,mask = self:getNormalMap(H)
    local ns1 = normals:size(2)
    local ns2 = normals:size(3)
    normals = normals:reshape(3,ns1*ns2):t()  
    mask = mask:reshape(1,ns1*ns2):squeeze()
    normals = select3d(normals, mask:byte())
    
    local dis = pts:clone():pow(2):sum(2):pow(.5)

    local azimuth = torch.atan2(pts:select(2,2):clone(), pts:select(2,1):clone()) +math.pi --range 0 to 2*math.pi                   
    local elevation = torch.acos(pts:select(2,3):clone():cdiv(dis))  -- 0 to math.pi*150/180

    local size_x = 360
    local size_y = 150
    local min_x = 1
    local max_x = size_x*res
    local min_y = 1
    local max_y = size_y*res

    azimuth = (azimuth * (max_x-min_x)/(size_x*math.pi/180) + min_y)
    elevation = (elevation * (max_y-min_y)/(size_y*math.pi/180) + min_y)

    local azimuthGood = torch.eq(torch.ge(azimuth, min_x)+torch.le(azimuth, max_x),2)
    azimuth = azimuth[azimuthGood]
    elevation = elevation[azimuthGood]
    dis = dis[azimuthGood]
    pts = select3d(pts, azimuthGood)
    normals = select3d(normals, azimuthGood)
    local elevationGood = torch.eq(torch.ge(elevation, min_y)+torch.le(elevation, max_y),2)
    azimuth = azimuth[elevationGood]:long()
    elevation = elevation[elevationGood]:long()
    dis = dis[elevationGood]
    pts = select3d(pts, elevationGood)
    normals = select3d(normals, elevationGood)

    local indexVal = elevation*size_x*res+azimuth
    local temp, order = torch.sort(dis,true)

    indexVal = indexVal[order]
    dis = dis[order]
    pts = pts[order]
    normals = normals[order]

    local combined = torch.zeros(size_x*res*size_y*res)
    combined[indexVal] = dis
    combined:resize(size_y*res, size_x*res)

    local combined_pts = torch.zeros(size_x*res*size_y*res,3)
    combined_pts = select3d2(pts, indexVal, combined_pts)

    local combined_norms = torch.zeros(size_x*res*size_y*res,3)
    combined_norms = select3d2(normals, indexVal, combined_norms)
    return combined, center_n, combined_pts, combined_norms
end

function Sweep:getFloor(recalc)
    if not(self.floor) or recalc then
        local faro_points = self:getPC()
        local  y, i = torch.sort(faro_points.points:select(2, 3), 1);
        local sorted_face_centers = faro_points.points:index(1, i);

        local tens = sorted_face_centers:select(2,3):clone():squeeze()
        local max = tens:max()
        local min = tens:min()
        local bins = 80
        local hist = torch.histc(tens,bins,min,max);
        
        local histv = torch.range(1,bins)
        local binwidth = (max-min)/bins
        histv:apply(function(i)
            return min + (i-0.5)*binwidth
        end)

        local ycum = torch.cumsum(hist:clone(), 1)
        local yh, ih = torch.sort(hist:clone(), 1, true);

        local histmax, histmaxind = hist:max();
        local histimg = hist:resize(1, 80)
        
        local kernel = torch.Tensor({-0.25, -0.25, 1, -0.25, -0.25}):resize(1,5)
        local dominance = image.convolve(histimg, kernel, 'same'):resize(80)/histmax;
        a = torch.range(1, dominance:size(1))
        a = a[torch.gt(dominance,.07)]
        self.floor = histv[a[1]]
    end
    self:saveMe()
    collectgarbage()
    return self.floor
end

function Sweep:setInitialTransformation(H)
    self.initial =H
    self:saveMe()
    collectgarbage()
end

function Sweep:getInitialTransformation()
    return self.initial
end

function Sweep:saveMe()
    torch.save(self.fsave_me, self)
end


function Sweep:getFlattenedAndCorners(recalc, numCorners)
    if not(self.flattenedxy and self.flattenedv and self.corners) or recalc then
        local p = self:getPC()
        local flattened, corners = p:get_flattened_images(self.parameters.scale, numCorners or self.parameters.numCorners)
        corners = applyToPointsReturn2d(Sweep.get2DTransformationToZero(flattened[1]),corners:t()):t()
        local flattenedx, flattenedy, flattenedv = image.thresholdReturnCoordinates(flattened[1],0)
        local flattenedxy = torch.cat(flattenedx, flattenedy,2)         
        flattenedxy = applyToPointsReturn2d(Sweep.get2DTransformationToZero(flattened[1]),flattenedxy:t()):t()
        self.corners = corners
        self.flattenedxy = flattenedxy
        self.flattenedv = flattenedv
        p:write(self.fod)
        self:saveMe()
    end
    collectgarbage()
    return self.corners, self.flattenedxy, self.flattenedv
end

function Sweep:flattenAndCornersTo(forward,name, numCorners, H)
    corners, flattenedxy, flattenedv = self:getFlattenedAndCorners(true, numCorners)
    return Sweep.applyToPointsReturn2d(H, self.corners:t()):t(), 
            Sweep.applyToPointsReturn2d(H,self.flattenedxy:t()):t(), self.flattenedv
end
function Sweep.get2DTo3DTransformation(scale, transformation, zshift)
    local scaleT  = torch.eye(3)
    scaleT[1][1] = 1/scale
    scaleT[2][2] = 1/scale
    

    local iscaleT  = torch.eye(3)
    iscaleT[1][1] = scale
    iscaleT[2][2] = scale
    
    local rot = iscaleT*transformation * scaleT

    local combined = torch.eye(4)
    combined[{{1,2},{1,2}}] = rot[{{1,2},{1,2}}]
    combined[1][4] = rot[1][3]
    combined[2][4]= rot[2][3]
    combined[3][4] = zshift
    return combined
end

function Sweep.get3Dto2DTransformation(scale, transformation)
    local scaleT  = torch.eye(3)
    scaleT[1][1] = 1/scale
    scaleT[2][2] = 1/scale
    
    local iscaleT  = torch.eye(3)
    iscaleT[1][1] = scale
    iscaleT[2][2] = scale
    
    local newTransform = torch.eye(3)
    newTransform[{{1,2},{1,2}}] = transformation[{{1,2},{1,2}}]
    newTransform[{{1,2},3}] = transformation[{{1,2},4}]
    return scaleT * newTransform * iscaleT  
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
    local parameterizedHough = Hough.new(HoughParameters)

    temp, flattenedxy = self:getFlattenedAndCorners()
    local domAngle1_src,domAngle2_src = parameterizedHough:getMainDirections(opencv.Mat.new(Sweep.flattened2Image(flattenedxy):byte()))
    H = torch.eye(3)
    H[1][1] = torch.cos(domAngle1_src)
    H[2][2] = H[1][1]
    H[2][1] = -torch.sin(domAngle1_src)
    H[1][2] = -H[2][1]
    return H
end

function Sweep.flattened2Image(flattenedxy,corners)
    local combinedMin = torch.min(flattenedxy,1)
    local minT = torch.min(combinedMin, 1)
    if corners then
    	local cMin = torch.min(corners,1)
    	minT[1][1] = math.min(minT[1][1],cMin[1][1])
    	minT[1][2] = math.min(minT[1][2],cMin[1][2])
    end
    
    local combinedMax = torch.max(flattenedxy,1)
    local maxT = torch.max(combinedMax, 1)
    if corners then
    	local cMax = torch.max(corners,1)
    	maxT[1][1] = math.max(maxT[1][1],cMax[1][1])
    	maxT[1][2] = math.max(maxT[1][2],cMax[1][2])
    end

    flattenedxy = flattenedxy-torch.repeatTensor(minT, flattenedxy:size(1), 1)+1
    
    local size_us = (maxT-minT+1):ceil():reshape(2)
    local combined = torch.zeros(size_us[1], size_us[2])
    for i = 1, flattenedxy:size(1) do
        combined[math.max(1,flattenedxy[i][1])][math.max(1,flattenedxy[i][2])]=1
    end

    if corners then
    	corners_orig = corners
    	corners = corners - torch.repeatTensor(minT, corners:size(1),1)+1
		combined = combined:repeatTensor(3,1,1)
    	for i = 1, corners:size(1) do
	    	combined:sub(1,1,math.max(1,corners[i][1]-3),math.min(size_us[1],corners[i][1]+3),math.max(1,corners[i][2]-3),math.min(size_us[2],corners[i][2]+3)):fill(1)
	    	combined:sub(2,2,math.max(1,corners[i][1]-3),math.min(size_us[1],corners[i][1]+3),math.max(1,corners[i][2]-3),math.min(size_us[2],corners[i][2]+3)):fill(0)
	    	combined:sub(3,3,math.max(1,corners[i][1]-3),math.min(size_us[1],corners[i][1]+3),math.max(1,corners[i][2]-3),math.min(size_us[2],corners[i][2]+3)):fill(0)
	    end
	end
    return combined
end

function Sweep:show_flattened()
	local c,fxy,fv = self:getFlattenedAndCorners()
	image.display(Sweep.flattened2Image(fxy,c))
end

function Sweep.applyToPointsReturn2d(H, mat_src)
   if (mat_src:size(1) == 2) then
      return torch.mm(H, torch.cat(mat_src, torch.ones(1,mat_src:size(2)),1))[{{1,2},{}}]
   elseif (mat_src:size(1) == 3) then
      return torch.mm(H, mat_src)[{{1,2},{}}]
   end   
end
function Sweep.get2DTransformationToZero(img)
   return Sweep.getRotationMatrix(0,torch.Tensor({-img:size(1)/2, -img:size(2)/2}))
end

function Sweep.getRotationMatrix(angle, translation)
    local cosA = math.cos(angle)
    local sinA = math.sin(angle)
    return torch.Tensor({{cosA, -sinA, translation[1]},{sinA, cosA, translation[2]},{0, 0, 1}})
end

--old 
--[[
function Sweep:getTree(H)
    local p, scanner_pose = self:getPC(H)
    log.trace("building tree ...")log.tic()
    local t = octomap.Tree.new(self.parameters.octTreeRes)
    max_radius = p:get_max_radius()
    t:add_points(p.points,scanner_pose, max_radius)
    log.trace(" - in ".. log.toc())

    t:stats()
    t:info()
    return {t, scanner_pose, p:get_max_radius()}
end

--this is for faro, that goes -60 to 90 degrees
function Sweep:shootRay(H, center_n, max_range_in, tree_in, size_1, size_2)
    local tree, center, max_range
    if not(tree_in) then
        local tree_return = self:getTree(H)
        tree = tree_return[1]
        camera_pose = center_n or tree_return[2]
        max_range = max_range_in or tree_return[3]

    else
        tree = tree_in
        camera_pose = center_n
        max_range = max_range_in
    end
    if(center_n and not(max_range_in)) then
        local p =self:getPC(H)
        max_range = (p.points-center_n:reshape(1,3):repeatTensor(p.points:size(1),1)):pow(2):sum(2):pow(.5):max()
    end
    if (not tree) then
        error("error getting tree")
    end
    local size_1 = size_1 or 180
    local size_2 = size_2 or 360
    --do negative 60 to positive 90
    local proj = projection.SphericalProjection.new(size_1,size_2,150/180*math.pi, 2*math.pi)
    local angles = proj:angles_map(nil, nil, nil, 15*math.pi/180, nil)

    local dirs = geom.util.spherical_angles_to_unit_cartesian(angles)
    local frame = tree:ray_trace(camera_pose, dirs, max_range)
    collectgarbage()
    return frame, tree, camera_pose, max_range
end
]]--