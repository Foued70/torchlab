Sweep = Class()

Sweep.XYZ = "XYZ"
Sweep.PNG = "PNG"
Sweep.OD = "OD"
Sweep.SWEEP = "SWEEP"

local path = require 'path'
local pcl = PointCloud.PointCloud

function Sweep:__init(base_dir, name, extname)
    if not(path.extname(name) == "" or path.extname(name) == name) then
        error("expected name without extension to be given to Sweep initializer")
    end
    if not(util.fs.is_dir(base_dir)) then
        error("expected base_dir to exist for initializer for Sweep")
    end
    if not(util.fs.is_file(path.join(base_dir, Sweep.XYZ, name .. ".xyz"))) then
        error("expected sweep to exist in XYZ folder of base_dir " .. path.join(base_dir, Sweep.XYZ, name.. ".xyz"))
    end
    self.extname = extname or ".xyz"
    self.base_dir = base_dir
    self.name = name
    self:mkdirs()
    self:init_variables()
    self:saveMe()
end

function Sweep.newOrLoad(base_dir, name, extname)
    if(util.fs.is_file(path.join(base_dir, Sweep.SWEEP, name .. ".dat"))) then
        print("loading sweep from file " .. name)
        return torch.load(path.join(base_dir, Sweep.SWEEP, name .. ".dat"))
    else
        return Sweep.new(base_dir, name, extname)
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
    self.fxyz = path.join(self.base_dir, Sweep.XYZ, self.name.. self.extname)
    self.fpng = path.join(self.base_dir,Sweep.PNG, self.name..".png")
    self.fsave_me = path.join(self.base_dir, Sweep.SWEEP, self.name .. ".dat")
end

function Sweep:__write_keys()
  return {'base_dir', 'name', 'parameters', 
    'floor',
    'corners', 'flattenedxy', 'flattenedv', 
    'fod', 'fxyz', 'fpng', 'fsave_me', 'scale', 'extname', 'angle'
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

function Sweep:getPoints(H)
    local pc = self:getPC()
    local rgb = pc.rgb or torch.zeros(pc.points:size())
    collectgarbage()
    H = H or torch.eye(4)
    H = H:contiguous()
    --pc:set_pose_from_rotation_matrix(H)
    local points = pc:get_global_points_H(H)
    --pc:set_pose_from_rotation_matrix(torch.eye(4))
    return points, rgb
end

function Sweep:getPC(recalc)
    local pc
    local rgb
    if (util.fs.is_file(self.fod) and not(recalc)) then
        pc = pcl.new(self.fod)
    else
        pc = pcl.new(self.fxyz)
        --pc:downsampleBy(4)
        if util.fs.is_dir(path.join(self.base_dir,Sweep.PNG)) and util.fs.is_file(self.fpng) then
            pc:load_rgb_map(self.fpng)
        end
        pc:save_to_od(self.fod)
        pc = pc
    end
    collectgarbage()
    return pc
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
function Sweep:getDepthImage(H, center_i, res)
    local res = res or 1
    local center = center_i or torch.zeros(3)
    if(H and not(center_i)) then
        center = H:sub(1,3,4,4):squeeze()
    end
    local pts= self:getPoints(H) --pts should already be centered
    pts = pts-center:reshape(1,3):repeatTensor(pts:size(1),1)
    local pc = self:getPC()
    local idx, mask = pc:get_index_and_mask()
    mask = mask*-1+1
    pc:set_pose_from_rotation_matrix(H or torch.eye(4))
    local normals = pc:get_global_normal_map(H or torch.eye(4))

    pc:set_pose_from_rotation_matrix(torch.eye(4))
    pc:save_to_od(self.fod)

    local ns1 = normals:size(2)
    local ns2 = normals:size(3)
    normals = normals:reshape(3,ns1*ns2):t()  
    mask = mask:reshape(1,ns1*ns2):squeeze()
    normals = select3d(normals, mask:byte())
    
    local dis = pts:clone():norm(2,2)

    local azimuth  = (torch.atan2(pts:select(2,2):clone(), pts:select(2,1):clone()) +math.pi)*-1+2*math.pi--range 0 to 2*math.pi                   

    local elevation = torch.acos(pts:select(2,3):clone():cdiv(dis))  -- 0 to math.pi*150/180
    --image.display(azimuth)
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

    return combined, center, combined_pts, combined_norms
end

function Sweep:getAxisAlignAngle(recalc)
    if(not(self.angle) or recalc) then
        local norm_map = self:getPC():get_normal_map()
         norm_map = norm_map:reshape(norm_map:size(1),norm_map:size(2)*norm_map:size(3)):t()
         norm_map = select3d(norm_map, torch.ne(norm_map:clone():norm(2,2):squeeze(),0))
        local angle_az = geom.util.unit_cartesian_to_spherical_angles(norm_map:t():contiguous())
        local az = angle_az[1][torch.eq(torch.lt(angle_az[2], math.pi/4)+torch.gt(angle_az[2],-math.pi/4),2)]
        local t,angle1 = torch.histc(az,361,-math.pi,math.pi):max(1)
        self.angle= angle1:squeeze()-181
        self:saveMe()
    end
    return self.angle
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
        
        local binwidth = (max-min)/(bins-2)
        local histv = torch.linspace(min+binwidth/2, max-binwidth/2, bins)

        local ycum = torch.cumsum(hist:clone(), 1)
        local yh, ih = torch.sort(hist:clone(), 1, true);

        local histmax, histmaxind = hist:max();
        local histimg = hist:resize(1, bins)
        
        local kernel = torch.Tensor({-0.25, -0.25, 1, -0.25, -0.25}):resize(1,5)
        local dominance = image.convolve(histimg, kernel, 'same'):resize(bins)/histmax;
        a = torch.range(1, dominance:size(1))
        a = a[torch.gt(dominance,.07)]
        self.floor = histv[a[1]]
    end
    self:saveMe()
    collectgarbage()
    return self.floor
end

function Sweep:getInitialTransformation()
    local H = torch.eye(3,3)
    local angle = math.rad(self:getAxisAlignAngle())
    H[1][1] = math.cos(angle)
    H[2][2] = math.cos(angle)
    H[2][1] = math.sin(angle)
    H[1][2] = -math.sin(angle)  
    return H
end

function Sweep:saveMe()
    torch.save(self.fsave_me, self)
end


function Sweep:getFlattenedAndCorners(recalc, numCorners)
    if not(self.flattenedxy and self.flattenedv and self.corners) or recalc then
        local p = self:getPC()
        local flattened, corners, temp, scale = p:get_flattened_images(self.parameters.scale, numCorners or self.parameters.numCorners)

        corners = applyToPointsReturn2d(Sweep.get2DTransformationToZero(flattened[1]),corners:t()):t()
        local flattenedx, flattenedy, flattenedv = image.thresholdReturnCoordinates(flattened[1],0)
        local flattenedxy = torch.cat(flattenedx, flattenedy,2)         

        flattenedxy = applyToPointsReturn2d(Sweep.get2DTransformationToZero(flattened[1]),flattenedxy:t()):t()
        self.corners = corners
        self.flattenedxy = flattenedxy
        self.flattenedv = flattenedv
        self.scale = scale
        p:save_to_od(self.fod)
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

function Sweep.flattened2Image(flattenedxy,corners, minT, maxT)
    local combinedMin = torch.min(flattenedxy,1)
    local minT = minT or torch.min(combinedMin, 1)
    if corners then
    	local cMin = torch.min(corners,1)
    	minT[1][1] = math.min(minT[1][1],cMin[1][1])
    	minT[1][2] = math.min(minT[1][2],cMin[1][2])
    end
    
    local combinedMax = torch.max(flattenedxy,1)
    local maxT = maxT or torch.max(combinedMax, 1)
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

function Sweep:getTree(H, add_empties, res)
    log.trace("building tree ...")log.tic()
    local pc = self:getPC()
    local t = octomap.Tree.new(res or self.parameters.octTreeRes*pc.meter)
    local center = torch.zeros(3)
    if(H) then
        center = H:sub(1,3,4,4):squeeze():contiguous()
    end
    pts = self:getPoints(H)
    t:add_points(pts:contiguous(),center, pc:get_max_radius())
    if(add_empties) then
        t:add_points_only_empties(self:getZeroAzimuthAndElevation(H):contiguous(),center, pc:get_max_radius())
    end    
    log.trace(" - in ".. log.toc())

    t:stats()
    t:info()
    return t
end
function Sweep:getZeroAzimuthAndElevation(H)
    local center = torch.zeros(3)
    if(H) then
        center = H:sub(1,3,4,4):squeeze()
    end
    local pc = self:getPC()
    pc:set_pose_from_rotation_matrix(H or torch.eye(4))
    local pts = pc:get_global_from_3d_pts(pc:get_xyz_map_no_mask(),H or torch.eye(4))
    pts = pts-center:reshape(3,1,1):repeatTensor(1,pts:size(2),pts:size(3))
    local idx, mask = pc:get_index_and_mask()
    local dis = pc:get_depth_map_no_mask()
    pc:set_pose_from_rotation_matrix(torch.eye(4))

    pts[3][mask] =0
    local azimuth = torch.atan2(pts:select(1,2):clone(), pts:select(1,1):clone())+math.pi--range 0 to 2*math.pi                   
    local elevation = torch.acos(pts:select(1,3):clone():cdiv(dis+10^-6))  -- 0 to math.pi*150/180
    elevation[mask]=  0

    azimuth[mask] = math.huge   
    local minaz = azimuth:min(1):squeeze()
    azimuth[mask] = -math.huge   
    local maxaz = azimuth:max(1):squeeze()
    azimuth[mask] = 0 

    local shouldShift = torch.gt((maxaz-minaz), 2*math.pi*.9):repeatTensor(azimuth:size(1),1)
    local azimuthShifted = (azimuth + (shouldShift:double()*math.pi))
    azimuthShifted:apply(function(x) return (x%(2*math.pi)) end)
    azimuthShifted = (azimuthShifted:sum(1):cdiv(torch.eq(mask,0):double():sum(1)):squeeze())

    local predictedAzimuth = azimuthShifted:repeatTensor(azimuth:size(1),1)- (shouldShift:double()*math.pi)

    local predictedElevation = (elevation:sum(2):cdiv(torch.eq(mask,0):double():sum(2)):squeeze()):repeatTensor(elevation:size(2),1):t()

    local dis = pc:get_max_radius()*.95
    local zs = torch.cos(predictedElevation)*dis
    local xs = ((zs:clone():pow(2)*-1+dis^2):cdiv(torch.tan(predictedAzimuth):pow(2)+1)):pow(.5) --x^2+y^2 = 1-z^2, y/x=torch.tan(azimuth)
    local posX = torch.eq(torch.gt(predictedAzimuth,math.pi/2)+torch.lt(predictedAzimuth,3*math.pi/2),2)
    local negX = posX*-1+1
    xs[negX] = xs[negX]*-1
    local negY = torch.lt(predictedAzimuth,math.pi)
    ys = torch.abs(xs:clone():cmul(torch.tan(predictedAzimuth))) --positive if azimuth is 0,pi
    ys[negY] = ys[negY]*-1
    return torch.cat(torch.cat(xs[mask], ys[mask],2), zs[mask],2)+center:reshape(1,3):repeatTensor(xs[mask]:size(1),1)
end
