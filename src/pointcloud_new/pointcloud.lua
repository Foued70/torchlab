local io     = require 'io'
local path   = require 'path'
local ffi    = require 'ffi'
local ctorch = util.ctorch
local log    = require '../util/log'
local Mat    = opencv.Mat

ffi.cdef
[[
    int get_step_maps(int* step_up, int* step_down, int* step_left, int* step_right, 
                  double* xyz, double* theta,
                  double min_step_size, double max_step_size, int height, int width);
                  
    int get_normal_theta_map(double* theta_map, double* centered_point_map, 
                  int* step_left, int* step_right,
                  int height, int width);
    int get_normal_phi_map(double* phi_map, double* centered_point_map, 
                int* step_up, int* step_down,
                int height, int width);
]]

local libpc   = util.ffi.load('libpcd')


-- angular to radians and back
local pi = math.pi
local r2d = 180/pi
local d2r = pi/180

pointcloud = Class()

function pointcloud:__init(input_or_filename)

   log.tic()
   
   if input_or_filename then
     if type(input_or_filename) == 'string' then
       if util.fs.is_file(input_or_filename) then
         self:load_input(torch.load(input_or_filename))
       else
         error('pointcloud.__init(): no such file - '..input_or_filename)
       end
     else
       self:load_input(input_or_filename)
     end
   end
   
   collectgarbage()
   
end

function pointcloud:load_input(input)
  
  local start = log.toc()
  
  self.height    = input.height
  self.width     = input.width
  self.count     = input.count
  self.meter     = input.meter
  self.centroid  = input.centroid
  self.meta      = input.meta
  
  self.depth_map     = input.depth_map:clone():contiguous()
  self.rgb_map       = input.rgb_map:clone():contiguous()
  self.intensity_map = input.intensity_map:clone():contiguous()
  self.xyz_phi_map   = input.xyz_phi_map:clone():contiguous()
  self.xyz_theta_map = input.xyz_theta_map:clone():contiguous()
  
  self.depth_valid_mask     = input.depth_valid_mask:clone():contiguous()
  self.rgb_valid_mask       = input.rgb_valid_mask:clone():contiguous()
  self.intensity_valid_mask = input.intensity_valid_mask:clone():contiguous()
  
  self.depth_inverse_mask     = self.depth_valid_mask:eq(0):clone():contiguous()
  self.rgb_inverse_mask       = self.rgb_valid_mask:eq(0):clone():contiguous()
  self.intensity_inverse_mask = self.intensity_valid_mask:eq(0):clone():contiguous()
  
  if input.xyz_map then
    self.xyz_map = input.xyz_map
  end
  
  if input.global_pose then
    self:set_global_pose(input.global_pose)
  end
  
  if input.global_rotation then
    self:set_global_rotation(input.global_rotation)
  end
  
  self:load_derived()
  
  local stop = log.toc()
  
  print('pointcloud:load_input() - '..(stop-start))
  
  collectgarbage()
  
end

function pointcloud:load_derived()
  
  self:get_index_map()
  self:get_hw_indices_map()
  self:get_xyz_map()
  self:get_xyz_stats()
  self:get_normal_map()
  
  collectgarbage()
  
end

function pointcloud:get_xyz_stats()
  if (not self.xyz_radius) then
    local xyz_map   = self:get_xyz_map()
    self.xyz_minval = xyz_map:clone():min(2):min(3):squeeze():clone():contiguous()
    self.xyz_maxval = xyz_map:clone():max(2):max(3):squeeze():clone():contiguous()
  
    local d = torch.zeros(2,3)
    d[1] = (self.centroid   - self.xyz_minval):clone()
    d[2] = (self.xyz_maxval - self.centroid  ):clone()
  
    self.xyz_radius = d:max(1):squeeze():clone():contiguous()
    
    d       = nil
    xyz_map = nil
  end
  collectgarbage()
  return self.xyz_minval, self.xyz_maxval, self.xyz_radius
end

function pointcloud:load_rgb_map(rgb_map, rgb_valid_mask)
  
  if type(rgb_map) == 'string' then
    rgb_map = image.load(rgb_map)
  end
  self.rgb_map = rgb_map:clone():contiguous()
  
  if rgb_valid_mask then
    if type(rgb_valid_mask) == 'string' then
      rgb_valid_mask = torch.load(rgb_valid_mask):gt(0)
    end
    self.rgb_valid_mask = rgb_valid_mask:clone():contiguous()
  else
    self.rgb_valid_mask = self.rgb_map:clone():pow(2):sum(1):squeeze():gt(0):clone():contiguous()
  end
  self.rgb_inverse_mask = self.rgb_valid_mask:eq(0):clone():contiguous()
  
end






-- mapping functions

function pointcloud:get_valid_masks()
  if (not self.normal_valid_mask) then
    self:get_normal_map()
  end
  collectgarbage()
  return self.depth_valid_mask:clone(), self.rgb_valid_mask:clone(), self.intensity_valid_mask:clone(), self.normal_valid_mask:clone()
end

function pointcloud:get_inverse_masks()
  if (not self.normal_inverse_mask) then
    self:get_normal_map()
  end
  collectgarbage()
  return self.depth_inverse_mask:clone(), self.rgb_inverse_mask:clone(), self.intensity_inverse_mask:clone(), self.normal_inverse_mask:clone()
end

function pointcloud:get_depth_map()
  return self.depth_map:clone(), self.depth_valid_mask:clone(), self.depth_inverse_mask:clone()
end

function pointcloud:get_rgb_map()
  return self.rgb_map:clone(), self.rgb_valid_mask:clone(), self.rgb_inverse_mask:clone()
end

function pointcloud:get_intensity_map()
  return self.intensity_map:clone(), self.intensity_valid_mask:clone(), self.intensity_inverse_mask:clone()
end

function pointcloud:get_index_map()
  if (not self.index_map) then
    local vmask           = self:get_valid_masks()
    self.index_map        = torch.zeros(self.height,self.width)
    self.index_map[vmask] = torch.range(1,self.count)
    self.index_map        = self.index_map:int()
    vmask                 = nil
  end
  collectgarbage()
  return self.index_map:clone()
end

function pointcloud:get_hw_indices_map()
  if (not self.hw_indices_map) then
    local height = self.height
    local width  = self.width
    local count  = self.count
    local vmask  = self:get_valid_masks()
    local hh     = torch.range(1, height):repeatTensor(width, 1):t():clone():contiguous()
    local ww     = torch.range(1, width):repeatTensor(height, 1):clone():contiguous()
    
    self.hw_indices_map    = torch.zeros(2, height,width)
    self.hw_indices_map[1] = hh:clone()
    self.hw_indices_map[2] = ww:clone()
    self.hw_indices_map    = self.hw_indices_map:clone():short() 
  
    hh    = nil
    ww    = nil
    vmask = nil
  end
  collectgarbage()
  return self.hw_indices_map:clone()
end

function pointcloud:get_xyz_map()
  if (not self.xyz_map) then
    local height             = self.height
    local width              = self.width
    local xyz_phi_map        = self.xyz_phi_map:clone()
    local xyz_theta_map      = self.xyz_theta_map:clone()
    local dmap, vmask, rmask = self:get_depth_map()
    
    self.xyz_map        = torch.zeros(3, height, width)
    self.xyz_map[1]     = xyz_phi_map:clone():cos():cmul(
                          xyz_theta_map:clone():cos()   )
    self.xyz_map[2]     = xyz_phi_map:clone():cos():cmul(
                          xyz_theta_map:clone():sin()   )
    self.xyz_map[3]     = xyz_phi_map:clone():sin()
    self.xyz_map        = self.xyz_map:cmul(dmap:repeatTensor(3,1,1)):clone():contiguous()
    
    self.xyz_map[rmask:repeatTensor(3,1,1)] = 0
    
    xyz_phi_map   = nil
    xyz_theta_map = nil
    dmap          = nil
    vmask         = nil
    rmask         = nil
  end
  collectgarbage()
  return self.xyz_map:clone(), self.xyz_phi_map:clone(), self.xyz_theta_map:clone()
end

function pointcloud:get_lookup_maps()
  if (not self.map_u) then
  
		local height = self.height
		local width  = self.width
		local meter  = self.meter
	
		local mind = 0.025 * meter
		local maxd = 0.100 * meter
	
		local xyz_map, xyz_phi_map, xyz_theta_map = self:get_xyz_map()
	
		self.map_u = torch.zeros(2, height, width):int():contiguous()
		self.map_d = torch.zeros(2, height, width):int():contiguous()
		self.map_l = torch.zeros(2, height, width):int():contiguous()
		self.map_r = torch.zeros(2, height, width):int():contiguous()
	  
		libpc.get_step_maps(torch.data(self.map_u), torch.data(self.map_d), 
		                    torch.data(self.map_l), torch.data(self.map_r), 
                        torch.data(xyz_map), torch.data(xyz_theta_map), 
                        mind, maxd, height, width)
	
		xyz_map       = nil
		xyz_phi_map   = nil
		xyz_theta_map = nil		
  end
  
  collectgarbage()
  return self.map_u:clone(), self.map_d:clone(), self.map_l:clone(), self.map_r:clone()
end

function pointcloud:get_normal_map()
  if (not self.normal_map) then
    local height  = self.height
    local width   = self.width
    local xyz_map = self:get_xyz_map()
    
    local mu,md,ml,mr = self:get_lookup_maps(force)
    
    self.normal_phi_map   = torch.zeros(height,width):clone():contiguous()
    self.normal_theta_map = torch.zeros(height,width):clone():contiguous()
    
    libpc.get_normal_phi_map(torch.data(self.normal_phi_map),
                      torch.data(xyz_map:clone():contiguous()),
                      torch.data(mu:contiguous()), torch.data(md:contiguous()),
                      height,width)
    libpc.get_normal_theta_map(torch.data(self.normal_theta_map),
                      torch.data(xyz_map:clone():contiguous()),
                      torch.data(ml:contiguous()), torch.data(mr:contiguous()),
                      height,width)
    
    self.normal_inverse_mask = self.normal_phi_map:le(-5):add(self.normal_theta_map:le(-5)):gt(0):clone():contiguous()
    self.normal_valid_mask   = self.normal_inverse_mask:eq(0):clone():contiguous()
    
    self.normal_phi_map[self.normal_inverse_mask]   = 0
    self.normal_theta_map[self.normal_inverse_mask] = 0
    
    self.normal_map    = torch.zeros(3,height,width)
    self.normal_map[1] = self.normal_phi_map:clone():cos():cmul(
                            self.normal_theta_map:clone():cos()):clone():contiguous()
    self.normal_map[2] = self.normal_phi_map:clone():cos():cmul(
                            self.normal_theta_map:clone():sin()):clone():contiguous()
    self.normal_map[3] = self.normal_phi_map:clone():sin():clone():contiguous()
    
    self.normal_residual_map = self.normal_map:clone():cmul(
                                  xyz_map:clone()):sum(1):squeeze():clone():contiguous()
                                  
    local neg = self.normal_residual_map:lt(0)
    
    for i = 1,3 do
      self.normal_map[i][neg]  = -self.normal_map[i][neg]
      self.normal_map[i][self.normal_inverse_mask] = 0
    end
    
    self.normal_residual_map = self.normal_map:clone():cmul(self.xyz_map:clone()):sum(1):squeeze()
    
    xyz_map = nil
    neg     = nil
    mu      = nil
    md      = nil
    ml      = nil
    mr      = nil
    
  end
  collectgarbage()
  return self.normal_map:clone(), self.normal_phi_map:clone(), self.normal_theta_map:clone(), self.normal_residual_map:clone(), self.normal_valid_mask:clone(), self.normal_inverse_mask:clone()
end

-- list functions

function mapToList(map, msk)

  local map_dim = map:dim()
  local msk_dim = msk:dim()
  
  local cnt = msk:double():sum()
  local lst = torch.zeros(cnt)
  
  if map_dim < msk_dim then
    error('map has fewer dimensions then mask')
  elseif map_dim > msk_dim + 1 then
    error('map has more dimensions then mask + 1')
  elseif map_dim == msk_dim then
    lst = map[msk]:clone():contiguous()
  else
    local channels = map:size(1)
    lst = torch.zeros(channels,cnt)
    for c = 1,channels do
      lst[c] = map[c][msk]:clone():contiguous()
    end
    lst = lst:t():clone():contiguous()
  end
  
  collectgarbage()
  return lst
  
end

function pointcloud:get_depth_list(omsk)
  local dmap, vmsk = self:get_depth_map()
  omsk             = omsk or vmsk
  local dlist      = mapToList(dmap, omsk)
  dmap = nil
  vmsk = nil
  collectgarbage()
  return dlist:clone():contiguous()
end

function pointcloud:get_rgb_list(omsk)
  local cmap, vmsk = self:get_rgb_map()
  omsk             = omsk or vmsk
  local clist      = mapToList(cmap, omsk)
  cmap = nil
  vmsk = nil
  collectgarbage()
  return clist:clone():contiguous()
end

function pointcloud:get_intensity_list(omsk)
  local imap, vmsk = self:get_intensity_map()
  omsk             = omsk or vmsk
  local ilist      = mapToList(imap, omsk)
  imap = nil
  vmsk = nil
  collectgarbage()
  return ilist:clone():contiguous()
end

function pointcloud:get_hw_list(omsk)
  local imap       = self:get_hw_indices_map()
  local vmsk       = self:get_valid_masks()
  omsk             = omsk or vmsk
  local ilist      = mapToList(imap, omsk)
  imap = nil
  vmsk = nil
  collectgarbage()
  return ilist:clone():contiguous()
end

function pointcloud:get_xyz_list(omsk)
  local pmap, phmap, thmap = self:get_xyz_map()
  local vmsk               = self:get_valid_masks()
  omsk                     = omsk or vmsk
  local plist              = mapToList( pmap, omsk)
  local phlist             = mapToList(phmap, omsk)
  local thlist             = mapToList(thmap, omsk)
  pmap  = nil
  phmap = nil
  thmap = nil
  vmsk  = nil
  collectgarbage()
  return plist:clone():contiguous(), phlist:clone():contiguous(), thlist:clone():contiguous()
end

function pointcloud:get_normal_list(omsk)
  local nmap, phmap, thmap, rmap, vmsk = self:get_normal_map()
  
  omsk                                = omsk or vmsk
  local nlist                         = mapToList( nmap, omsk)
  local phlist                        = mapToList(phmap, omsk)
  local thlist                        = mapToList(thmap, omsk)
  local rlist                         = mapToList( rmap, omsk)
  nmap  = nil
  phmap = nil
  thmap = nil
  rmap  = nil
  vmsk  = nil
  collectgarbage()
  return nlist:clone():contiguous(), phlist:clone():contiguous(), thlist:clone():contiguous(), rlist:clone():contiguous()
end

-- pose functions

function pointcloud:set_global_pose(pose)
  self.global_pose = pose:clone()
end

function pointcloud:set_global_rotation(quaternion)
  self.global_rotation = quaternion:clone()
end

function pointcloud:get_global_pose()
  if (not self.global_pose) then
    self:set_global_pose(torch.zeros(3))
  end
  collectgarbage()
  return self.global_pose:clone()
end

function pointcloud:get_global_rotation()
  if (not self.global_rotation) then
    self:set_global_rotation(torch.Tensor({0,0,0,1}))
  end
  collectgarbage()
  return self.global_rotation:clone()
end

function pointcloud:set_global_pose_and_rotation_from_matrix(mat)
  self:set_global_rotation(geom.quaternion.from_rotation_matrix(mat))
  self:set_global_pose(mat[{{1,3},4}]:clone())
end

-- write and save functions

function write_to_ascii(fout, count, xyz_list, rgb_list, intensity_list, normal_list, hw_list)
  
  local use_xyz       = not (xyz_list       == nil)
  local use_rgb       = not (rgb_list       == nil)
  local use_intensity = not (intensity_list == nil)
  local use_normal    = not (normal_list    == nil)
  local use_hw        = not (hw_list        == nil)
  
  for c = 1,count do
  
    local str = ''
    
    if use_hw then
      local elt = hw_list[c]
      str = str..elt[1]..' '..elt[2]..' '
      elt = nil
    end
    
    if use_xyz then
      local elt = xyz_list[c]
      str = str..elt[1]..' '..elt[2]..' '..elt[3]..' '
      elt = nil
    end
    
    if use_rgb then
      local elt = rgb_list[c]
      str = str..elt[1]..' '..elt[2]..' '..elt[3]..' '
      elt = nil
    end
    
    if use_normal then
      local elt = normal_list[c]
      str = str..elt[1]..' '..elt[2]..' '..elt[3]..' '
      elt = nil
    end
    
    if use_intensity then
      local elt = intensity_list[c]
      str = str..elt..' '
      elt = nil
    end
    
    str = str..'\n'
    
    fout:write(str)
    
  end
  
  collectgarbage()
  
end

function write_to_xyz(filename, count, xyz_list, rgb_list, intensity_list, normal_list, hw_list)
  
  local fout = io.open(filename,'w')
  
  write_to_ascii(fout, count, xyz_list, rgb_list, intensity_list, normal_list, hw_list)
  
  fout:close()
  collectgarbage()

end

function write_to_ply(filename, count, xyz_list, rgb_list, intensity_list, normal_list, hw_list)
  
  local use_xyz       = not (xyz_list       == nil)
  local use_rgb       = not (rgb_list       == nil)
  local use_intensity = not (intensity_list == nil)
  local use_normal    = not (normal_list    == nil)
  local use_hw        = not (hw_list        == nil)
  
  local fout = io.open(filename,'w')
  
  fout:write('ply\n')
  fout:write('format ascii 1.0\n')
  fout:write('element vertex '..count..'\n')
  
  if use_hw then
    fout:write('property int height\n')
    fout:write('property int width\n')
  end
  
  if use_xyz then
    fout:write('property float x\n')
    fout:write('property float y\n')
    fout:write('property float z\n')
  end
  
  if use_rgb then
    fout:write('property uchar red\n')
    fout:write('property uchar green\n')
    fout:write('property uchar blue\n')
  end
  
  if use_normal then
    fout:write('property float nx\n')
    fout:write('property float ny\n')
    fout:write('property float nz\n')
  end
  
  if use_intensity then
    fout:write('property uchar intensity\n')
  end
  
  fout:write('end_header\n')
  
  write_to_ascii(fout, count, xyz_list, rgb_list, intensity_list, normal_list, hw_list)
  
  fout:close()
  collectgarbage()

end

function pointcloud:save_setup(use_global, use_rgb, use_intensity, use_normal, use_hw)

  local dmsk, cmsk, imsk, nmsk = self:get_valid_masks()
  
  use_rgb        = use_rgb       and (cmsk:double():sum() > 0)
  use_intensity  = use_intensity and (imsk:double():sum() > 0)
  use_normal     = use_normal    and (nmsk:double():sum() > 0)
  
  local vmsk           = dmsk:clone()
  
  if use_rgb then
    vmsk           = vmsk:cmul(cmsk)
  end
  if use_intensity then
    vmsk           = vmsk:cmul(imsk)
  end
  if use_normal then
    vmsk = vmsk:cmul(nmsk)
  end
  
  dmsk = nil
  cmsk = nil
  imsk = nil
  nmsk = nil
  
  local count          = vmsk:double():sum()
  
  local xyz_list       = self:get_xyz_list(vmsk)
  local rgb_list       = nil
  local intensity_list = nil
  local normal_list    = nil
  local hw_list        = nil
  
  if use_rgb then
    rgb_list       = self:get_rgb_list(vmsk):mul(255):byte()
  end
  if use_intensity then
    intensity_list = self:get_intensity_list(vmsk):mul(255):byte()
  end
  if use_normal then
    normal_list    = self:get_normal_list(vmsk)
  end
  if use_hw then
    hw_list        = self:get_hw_list(vmsk)
  end
  
  if use_global then
    local pose = self:get_global_pose()
    local rot  = self:get_global_rotation()
    xyz_list   = geom.quaternion.rotate_translate(rot,pose,xyz_list)
    if use_normal then
      normal_list = geom.quaternion.rotate_translate(rot,torch.zeros(3),normal_list)
    end
  end
  
  collectgarbage()
  return count, xyz_list, rgb_list, intensity_list, normal_list, hw_list
end

function pointcloud:save_to_xyz_file(filename, use_global, use_rgb, use_intensity, use_normal, use_hw)

  local count, xyz_list, rgb_list, intensity_list, normal_list, hw_list = self:save_setup(use_global, use_rgb, use_intensity, use_normal, use_hw)
  
  write_to_xyz(filename, count, xyz_list, rgb_list, intensity_list, normal_list, hw_list)
  
end

function pointcloud:save_to_ply_file(filename, use_global, use_rgb, use_intensity, use_normal, use_hw)

  local count, xyz_list, rgb_list, intensity_list, normal_list, hw_list = self:save_setup(use_global, use_rgb, use_intensity, use_normal, use_hw)
  
  write_to_ply(filename, count, xyz_list, rgb_list, intensity_list, normal_list, hw_list)
  
end

function pointcloud:save_to_data_file(filename)
  
  torch.save(filename,self)
  
end