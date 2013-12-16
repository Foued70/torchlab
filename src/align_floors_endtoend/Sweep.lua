Sweep = Class()

Sweep.PNG = "PNG"
Sweep.OD = "OD"
Sweep.SWEEP = "SWEEP"

local path = require 'path'
local pcl = PointCloud.PointCloud
local util_sweep = align_floors_endtoend.util
function Sweep:__init(base_dir, name, xyz_file)
    if not(path.extname(xyz_file) == ".xyz") then
        error("expected extention to be .xyz")
    end
    if not(util.fs.is_dir(base_dir)) then
        error("expected base_dir to exist for initializer for Sweep")
    end
    if not(util.fs.is_file(xyz_file)) then
        error("expected xyz file to exist")
    end
    self.extname = path.extname(xyz_file)
    self.base_dir = base_dir
    self.name = name
    self.fxyz = xyz_file
    self:mkdirs()
    self:init_variables()
    self:saveMe()
end

function Sweep.newOrLoad(base_dir, name, xyz_dir)
    if(util.fs.is_file(path.join(base_dir, Sweep.SWEEP, name .. ".dat"))) then
        print("loading sweep from file " .. name)
        return torch.load(path.join(base_dir, Sweep.SWEEP, name .. ".dat"))
    else
        return Sweep.new(base_dir, name, xyz_dir)
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

--this is for faro, that goes -60 to 90 degrees
function Sweep:getDepthImage(H, center_i, res)
    local res = res or 1
    local center = center_i or torch.zeros(3)
    if(H and not(center_i)) then
        center = H:sub(1,3,4,4):squeeze()
    end
    local pts= self:getPC():get_global_points_H(H or torch.eye(4)) --pts should already be centered
    pts = pts-center:reshape(1,3):repeatTensor(pts:size(1),1)
    local pc = self:getPC()
    local idx, mask = pc:get_index_and_mask()
    mask = mask*-1+1
    local normals = pc:get_global_normal_map_H(H or torch.eye(4))
    normals = util.torch.select3d(normals, mask)
    
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

    local good = torch.eq(torch.ge(azimuth, min_x)+torch.le(azimuth, max_x)+torch.ge(elevation, min_y)+torch.le(elevation, max_y),4)
    azimuth = azimuth[good]:long()
    elevation = elevation[good]:long()
    dis = dis[good]
    --pts = util.torch.select3d(pts, good)
    normals = util.torch.select3d(normals, good)
    local indexVal = elevation*size_x*res+azimuth
    local temp, order = torch.sort(dis,true)

    indexVal = indexVal[order]
    dis = dis[order]
    --pts = pts[order]
    normals = normals[order]

    local combined = torch.zeros(size_x*res*size_y*res)
    combined[indexVal] = dis
    combined:resize(size_y*res, size_x*res)

    --local combined_pts = util.torch.assign3d(pts, indexVal)
    local combined_norms = util.torch.assign3d(normals, indexVal, torch.zeros(size_x*res*size_y*res,3))
    return combined, combined_norms
end

--returns in degrees
function Sweep:getAxisAlignAngle(recalc)
    if(not(self.angle) or recalc) then
        local norm_map = self:getPC():get_normal_map()
         norm_map = util.torch.select3d(norm_map, torch.ne(norm_map:clone():norm(2,1):squeeze(),0))
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
        local a = torch.range(1, dominance:size(1))
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
    return util_sweep.getRotationMatrix(angle, torch.Tensor({0,0}))
end

function Sweep:saveMe()
    torch.save(self.fsave_me, self)
end

function Sweep:getFlattenedAndCorners(recalc, numCorners)
    if not(self.flattenedxy and self.flattenedv and self.corners) or recalc then
        local p = self:getPC()
        local flattened, corners, temp, scale = p:get_flattened_images(self.parameters.scale, numCorners or self.parameters.numCorners)

        corners = util_sweep.applyToPointsReturn2d(util_sweep.get2DTransformationToZero(flattened[1]),corners:t()):t()
        local flattenedx, flattenedy, flattenedv = image.thresholdReturnCoordinates(flattened[1],0)
        local flattenedxy = torch.cat(flattenedx, flattenedy,2)         

        flattenedxy = util_sweep.applyToPointsReturn2d(util_sweep.get2DTransformationToZero(flattened[1]),flattenedxy:t()):t()
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
    return util_sweep.applyToPointsReturn2d(H, self.corners:t()):t(), 
            util_sweep.applyToPointsReturn2d(H,self.flattenedxy:t()):t(), self.flattenedv
end

function Sweep:show_flattened()
	local c,fxy,fv = self:getFlattenedAndCorners()
	image.display(util_sweep.flattened2Image(fxy,c))
end

-----------------------------------------------------------------------------------------------------------------------------
--old stuff w octree
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
    local pts = pc:get_global_from_3d_pts(pc:get_xyz_map_no_mask(),H or torch.eye(4))
    pts = pts-center:reshape(3,1,1):repeatTensor(1,pts:size(2),pts:size(3))
    local idx, mask = pc:get_index_and_mask()
    local dis = pc:get_depth_map_no_mask()

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
