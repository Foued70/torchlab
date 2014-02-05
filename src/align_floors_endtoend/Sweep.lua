Sweep = Class()

Sweep.PNG = "PNG"
Sweep.OD = "OD"
Sweep.SWEEP = "SWEEP"

local path = require 'path'
local pcl = PointCloud.PointCloud
local util_sweep = align_floors_endtoend.util
local PointCloudLoader = PointCloud.loader.load_pobot_ascii

function Sweep:__init(base_dir, name, xyz_file)
    if not(util.fs.is_dir(base_dir)) then
        error("expected base_dir to exist for initializer for Sweep")
    end
    if not(util.fs.is_dir(xyz_file)) and not(util.fs.is_file(xyz_file)) then
        error("expected xyz file or dir to exist ")
    end
    self.floader_input = xyz_file
    self.base_dir = base_dir
    self.name = name
    self:mkdirs()
    self:init_variables()
    self:save_me()
end

function Sweep.new_or_load(base_dir, name, xyz_file)
    if (util.fs.is_file(path.join(base_dir, Sweep.SWEEP, name .. ".dat"))) then
        return torch.load(path.join(base_dir, Sweep.SWEEP, name .. ".dat"))
    else
        return Sweep.new(base_dir, name, xyz_file)
    end
end
function Sweep:mkdirs() 
    util.fs.mkdir_p(path.join(self.base_dir, Sweep.OD))
    util.fs.mkdir_p(path.join(self.base_dir, Sweep.SWEEP))
end

function Sweep:init_variables()
    self.parameters = {}
    self.parameters.scale = 10 -- in milimeters
    self.parameters.numCorners = 100
    self.parameters.octTreeRes = .02
    self.initial = torch.eye(3)
    self.fod = path.join(self.base_dir, Sweep.OD, self.name.. ".od")
    self.fpng = path.join(self.base_dir,Sweep.PNG, self.name..".png")
    self.fsave_me = path.join(self.base_dir, Sweep.SWEEP, self.name .. ".dat")
end

function Sweep:__write_keys()
  return {'base_dir', 'name', 'parameters', 
    'floor',
    'corners', 'flattenedxy', 'flattenedv', 'flattened_angles',
    'fod', 'floader_input', 'fpng', 'fsave_me', 'scale', 'angle','corner_angles', 'combined', 'combined_norms'
    }
end

function Sweep:get_name()
    return self.name
end
function Sweep:get_base_dir()
    return self.base_dir
end
function Sweep:get_save_location()
    return self.fsave_me
end

function Sweep:get_pc(recalc)
    local pc
    local rgb
    if (util.fs.is_file(self.fod) and not(recalc)) then
        pc = PointCloud.pointcloud.new(self.fod)--PointCloud.PointCloud.load_od(self.fod)
    else
        loader = PointCloudLoader(self.floader_input)
        pc = PointCloud.pointcloud.new(loader)
        if util.fs.is_dir(path.join(self.base_dir,Sweep.PNG)) and util.fs.is_file(self.fpng) then
            pc:load_rgb_map(self.fpng)
        end
        pc:save_to_data_file(self.fod)
        pc = pc
    end
    collectgarbage()
    return pc
end

--goes -60 to 90 degrees
function Sweep:get_depth_image_from_perspective(H)
    local combined, combined_norms
    if ((H== torch.eye(4)) or not(H)) and self.combined_norms and self.combined then
        return self.combined, self.combined_norms
    else
        local pc = self:get_pc()
        local dmsk, cmsk, imsk, nmsk = pc:get_valid_masks()
        local pts= pc:get_xyz_list_transformed(nmsk, H or torch.eye(4)) --pts should already be centered
        local normals = pc:get_normal_list_transformed(nmsk, H or torch.eye(4))
        local dis = pts:clone():norm(2,2)

        local azimuth  = (torch.atan2(pts:select(2,2):clone(), pts:select(2,1):clone()) +math.pi)*-1+2*math.pi--range 0 to 2*math.pi                   
        local elevation = torch.acos(pts:select(2,3):clone():cdiv(dis))  -- 0 to math.pi*150/180

        local size_x = pc:get_depth_map():size(2) --should be sized based on max and min difference... because of overlap
        local size_y = pc:get_depth_map():size(1)
        azimuth = (azimuth * (size_x-1)/(360*math.pi/180) + 1)
        elevation = (elevation * (size_y-1)/(150*math.pi/180) + 1) --to do not hard code here

        local good = torch.eq(
                        torch.ge(azimuth, 1)+torch.le(azimuth, size_x)+torch.ge(elevation, 1)+torch.le(elevation, size_y),4)
        azimuth = azimuth[good]:long()
        elevation = elevation[good]:long()
        dis = dis[good]
        normals = util.torch.select3d(normals, good)
        local indexVal = elevation*size_x+azimuth

    --[[ optimize to pick closest one to center of point
        local temp, order = torch.sort(dis,true)
        indexVal = indexVal[order]
        normals = normals[order]
        dis = temp
    --]]
        combined = torch.zeros(size_x*size_y)
        combined[indexVal] = dis
        combined:resize(size_y, size_x)

        combined_norms = util.torch.assign3d(normals, indexVal, torch.zeros(size_x*size_y,3))
    end
    if ((H== torch.eye(4)) or not(H)) then
        self.combined = combined
        self.combined_norms = combined_norms
        self:save_me()
    end
    return combined, combined_norms
end

--returns in degrees
function Sweep:get_axis_align_angle(recalc)
    if(not(self.angle) or recalc) then
        local norm_list = self:get_pc():get_normal_list()
        local angle_az = geom.util.unit_cartesian_to_spherical_angles(norm_list:t():contiguous())
        local az = angle_az[1][torch.eq(torch.lt(angle_az[2], math.pi/4)+torch.gt(angle_az[2],-math.pi/4),2)]
        local t,angle1 = torch.histc(az,361,-math.pi,math.pi):max(1)
        self.angle= angle1:squeeze()-181
        self:save_me()
    end
    return self.angle
end
function Sweep:get_floor(recalc)
    if not(self.floor) or recalc then
        local pc = self:get_pc()
        local  y, i = torch.sort(pc:get_xyz_list():select(2, 3), 1);
        local sorted_face_centers = pc:get_xyz_list():index(1, i);

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
    self:save_me()
    collectgarbage()
    return self.floor
end

function Sweep:get_initial_transformation()
    local H = torch.eye(3,3)
    local angle = math.rad(self:get_axis_align_angle())
    return util_sweep.get_rotation_matrix(angle, torch.Tensor({0,0}))
end

function Sweep:save_me()
    torch.save(self.fsave_me, self)
end

function Sweep:get_flattened_and_corners(recalc, numCorners)
    if not(self.flattenedxy and self.flattenedv and self.corners) or recalc then
        local pc = self:get_pc()
        local flattened, corners, temp, scale,image_theta, it_mask, nm_flat = pc:get_flattened_image(self.parameters.scale, numCorners or self.parameters.numCorners)
                self.minT = torch.Tensor({-flattened[1]:size(1)/2, -flattened[1]:size(2)/2})
        self.corner_angles =  image_theta:reshape(image_theta:size(1)*image_theta:size(2)):index(1,((corners:select(2,1)-1)*image_theta:size(2)+corners:select(2,2)):long())

     
        corners = util_sweep.apply_to_points_return2d(util_sweep.get_2D_transformation_to_zero(flattened[1]),corners:t()):t()
        
        local flattenedx, flattenedy, flattenedv = image.thresholdReturnCoordinates(flattened[1],0)
        local flattenedxy = torch.cat(flattenedx, flattenedy,2)         
        self.flattened_angles = image_theta:reshape(image_theta:size(1)*image_theta:size(2)):index(1,((flattenedxy:select(2,1)-1)*image_theta:size(2)+flattenedxy:select(2,2)):long())

        flattenedxy = util_sweep.apply_to_points_return2d(util_sweep.get_2D_transformation_to_zero(flattened[1]),flattenedxy:t()):t()
        self.corners = corners
        self.flattenedxy = flattenedxy
        self.flattenedv = flattenedv

        self.scale = scale
        pc:save_to_data_file(self.fod)
        self:save_me()
    end
    collectgarbage()
    return self.corners, self.flattenedxy, self.flattenedv, self.corner_angles, self.flattened_angles
end

function Sweep:show_flattened()
	local c,fxy,fv = self:get_flattened_and_corners()
	image.display(util_sweep.flattened_to_image(fxy,c))
end
