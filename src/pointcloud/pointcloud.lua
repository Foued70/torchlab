local io     = require 'io'
local path   = require 'path'
local ffi    = require 'ffi'
local ctorch = util.ctorch
local log    = require '../util/log'
local Mat    = opencv.Mat

ffi.cdef
[[
    int get_step_maps(int* step_up, int* step_down, int* step_left, int* step_right, 
                  double* xyz, double* theta_map, char * vmask,
                  double min_step_size, double max_step_size, int height, int width);
                  
    int get_normal_theta_map(double* theta_map, double* centered_point_map, 
                  int* step_left, int* step_right,
                  int height, int width);
    int get_normal_phi_map(double* phi_map, double* centered_point_map, 
                int* step_up, int* step_down,
                int height, int width);
    int flatten_image(double* imagez, double* image_corner,
                                          double* coord_map, double *points,
                                          char* connection_map, char* corners_map,
                                          double* corners_map_filled,
                                          int pan_hght, int pan_wdth,
                                          int img_hght, int img_wdth);
    int flatten_image_with_theta(double* imagez, double* image_corner, double* corners_map_filled,
                                          double* imaget, double* theta_cnt,
                                          double* coord_map, double *points,
                                          char* connection_map, char* corners_map, double* theta,
                                          double* weights,
                                          int pan_hght, int pan_wdth,
                                          int img_hght, int img_wdth);
]]

local libpc   = util.ffi.load('libpcd')


-- angular to radians and back
local pi = math.pi
local r2d = 180/pi
local d2r = pi/180
local very_small_number = 0.00000000000000001

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
  self.def_mind  = 0.025 * self.meter
  self.def_maxd  = 0.100 * self.meter
  
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

function pointcloud:get_noise()

  local height        = self.height
  local width         = self.width
  
  local imask         = self:get_inverse_masks()
  local xyz,phi,theta = self:get_xyz_map()
  local z             = xyz[3]:clone()
  
  local phi_max       = phi:max()
  local phi_min       = phi:min()
  local phi_rng       = phi_max-phi_min
  local phi_stp       = phi_rng/(height-1)
  local phi_thr       = phi_stp/2
  
  local top_row_mask  = phi:le(phi_max):cmul(phi:ge(phi_max-phi_thr))
  
  top_row_mask[imask] = 0
  
  topnoise = z[top_row_mask]:std()
  
  imask        = nil
  xyz          = nil
  phi          = nil
  theta        = nil
  top_row_mask = nil
  collectgarbage()
  
  return topnoise
  
end

function pointcloud:get_center_transformed(H)
  local center = self.centroid:repeatTensor(1,1)
  local H_new = H:clone()
  return (H_new*torch.cat(center,torch.ones(center:size(1),1)):t()):t():sub(1,-1,1,3):squeeze()
end

function pointcloud:fix_phi(phi_0)
  phi_0 = phi_0 or 0
  self.xyz_phi_map = self.xyz_phi_map + phi_0
  self.xyz_phi_map[self.depth_inverse_mask] = 0
  self:get_xyz_map()
  self:get_xyz_stats()
  self:get_normal_map()
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

function pointcloud:get_xyz_map(force)
  if force or (not self.xyz_map) then
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

function pointcloud:get_lookup_maps(force, mind, maxd, vmask)
  if force or (not self.map_u) then
  
		local height = self.height
		local width  = self.width
		local meter  = self.meter
	
		mind  = mind  or self.def_mind
		maxd  = maxd  or (4 * mind)
		
		self.cur_mind = mind
		self.cur_maxd = maxd
		
		local dm, dvm = self:get_depth_map()
		vmask = vmask or dvm:clone()
		vmask:cmul(dvm)
		
		dm  = nil
		dvm = nil
		collectgarbage()
	
		local xyz_map, xyz_phi_map, xyz_theta_map = self:get_xyz_map()
		
		self.map_u = nil
		self.map_d = nil
		self.map_l = nil
		self.map_r = nil
		collectgarbage()
	
		self.map_u = torch.zeros(2, height, width):int():contiguous()
		self.map_d = torch.zeros(2, height, width):int():contiguous()
		self.map_l = torch.zeros(2, height, width):int():contiguous()
		self.map_r = torch.zeros(2, height, width):int():contiguous()
	  
		libpc.get_step_maps(torch.data(self.map_u), torch.data(self.map_d), 
		                    torch.data(self.map_l), torch.data(self.map_r), 
                        torch.data(xyz_map), torch.data(xyz_theta_map), 
                        torch.data(vmask:clone():contiguous()), 
                        mind, maxd, height, width)
	
		xyz_map       = nil
		xyz_phi_map   = nil
		xyz_theta_map = nil		
  end
  
  collectgarbage()
  return self.map_u:clone(), self.map_d:clone(), self.map_l:clone(), self.map_r:clone()
end

function pointcloud:get_normal_map(force, mind, maxd, vmask)
  if force or (not self.normal_map) then
    local height  = self.height
    local width   = self.width
    local xyz_map = self:get_xyz_map()
    
    local mu,md,ml,mr = self:get_lookup_maps(force, mind, maxd, vmask)
    
    self.normal_phi_map = nil
    self.normal_theta_map = nil
    self.normal_map = nil
    self.normal_inverse_mask = nil
    self.normal_valid_mask = nil
    collectgarbage()
    
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
    
    self.normal_inverse_mask = self.normal_phi_map:le(-500):add(self.normal_theta_map:le(-500)):gt(0):clone():contiguous()
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
                                  xyz_map:clone()):sum(1):mul(-1):squeeze():clone():contiguous()
                                  
    local neg = self.normal_residual_map:lt(0)
    
    for i = 1,3 do
      self.normal_map[i][neg]  = -self.normal_map[i][neg]
      self.normal_map[i][self.normal_inverse_mask] = 0
    end
    
    self.normal_phi_map[neg] = -self.normal_phi_map[neg]
    self.normal_theta_map[neg] = self.normal_theta_map[neg] + math.pi
    self.normal_theta_map[self.normal_theta_map:gt(math.pi)] = self.normal_theta_map[self.normal_theta_map:gt(math.pi)]- 2*math.pi
    
    self.normal_residual_map = self.normal_map:clone():cmul(
                                  xyz_map:clone()):sum(1):mul(-1):squeeze():clone():contiguous()
    
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

function pointcloud:get_density_map()

  local map_u, map_d, map_l, map_r = self:get_lookup_maps()

  local l = map_d[1] - map_u[1]
  l[(map_d[1] - map_l[1]):eq(0)] = l[(map_d[1] - map_r[1]):eq(0)]:mul(2)
  l[(map_r[1] - map_u[1]):eq(0)] = l[(map_r[1] - map_u[1]):eq(0)]:mul(2)
  
  local w = map_r[2] - map_l[2]
  w[(map_d[2] - map_l[2]):eq(0)] = w[(map_d[2] - map_r[2]):eq(0)]:mul(2)
  w[(map_r[2] - map_u[2]):eq(0)] = w[(map_r[2] - map_u[2]):eq(0)]:mul(2)
  w[w:lt(0)] = w[w:lt(0)]:add(self.width)
  
  mind = self.cur_mind

  local density = w:clone():cmul(l):double():div(math.pow(self.cur_mind/10,2)*2*math.pi):clone():contiguous()
  
  l     = nil
  w     = nil
  map_u = nil
  map_d = nil
  map_l = nil
  map_r = nil
  collectgarbage()
  
  return density
  
end

function pointcloud:get_reverse_density_map()
  local dense = self:get_density_map()
  local mask = self:get_inverse_masks()
  dense[dense:eq(0)] = dense[dense:gt(0)]:min()/10
  local rd = torch.ones(dense:size()):cdiv(dense)
  rd[mask] = 0
  
  dense = nil
  mask = nil
  collectgarbage()
  
  return rd
  
end

function pointcloud:get_area_map()
  local nmp            = self:get_normal_map()
	local depth          = self:get_depth_map()
	local imask          = self:get_inverse_masks()
	local xyz, phi       = self:get_xyz_map()
	local xyz_unit       = xyz:clone():cdiv(depth:repeatTensor(3,1,1))
	for d = 1,3 do
	  xyz_unit[d][imask] = 0
	end
	
	local dotp           = nmp:clone():cmul(xyz_unit):sum(1):abs():squeeze()
	
	dotp:pow(2):mul(-1):add(1)
	dotp[dotp:gt(1)]     = 1
	dotp[dotp:lt(0)]     = 0
	dotp:sqrt()
	
	local depth_2d       = depth:clone():cmul(phi:clone():cos())
	local r1             = depth_2d:clone():mul(2*math.pi/self.width)/10
	local r2             = depth:clone():mul((phi:max() - phi:min())/self.height)/10
	local area           = r1:clone():cmul(r2):mul(math.pi):cmul(dotp)
	
	nmp      = nil
	depth    = nil
	imask    = nil
	xyz      = nil
	phi      = nil
	xyz_unit = nil
	dotp     = nil
	depth_2d = nil
	r1       = nil
	r2       = nil
	collectgarbage()
	
	return area
end

function pointcloud:get_depthxy_map()
  local depth           = self:get_depth_map()
  local xyz, phi, theta = self:get_xyz_map()
  local depth2d         = depth:clone():cmul(phi:clone():cos())
  depth = nil
  xyz   = nil
  phi   = nil
  theta = nil
  return depth2d
end

function pointcloud:get_xyz_map_transformed(H)
  local vmsk     = self:get_valid_masks()
  local xyz_list = self:get_xyz_list_transformed(vmsk,H):t():clone()
  local xyz_map  = torch.zeros(3,self.height,self.width)
  local c = self:get_center_transformed(H)
  for d = 1,3 do
    xyz_map[d][vmsk] = xyz_list[d]
  end
  dmap = self:get_depth_map()
  d2_map = ((xyz_map[1]-c[1]):clone():pow(2) + (xyz_map[2]-c[2]):clone():pow(2)):sqrt()
  phi_map = (xyz_map[3]-c[3]):cdiv(dmap):clone():asin()
  tht_map = (xyz_map[1]-c[1]):clone():cdiv(d2_map):acos()
  mask    = (xyz_map[2]-c[2]):lt(0)
  tht_map[mask] = -tht_map[mask]
  phi_map[vmsk:eq(0)] = 0
  tht_map[vmsk:eq(0)] = 0
  return xyz_map:clone(), phi_map:clone(), tht_map:clone()
end

function pointcloud:get_normal_map_transformed(H)
  local vmsk     = self:get_valid_masks()
  local nrm_list = self:get_normal_list_transformed(vmsk,H):t():clone()
  local nrm_map  = torch.zeros(3,self.height,self.width)
  for d = 1,3 do
    nrm_map[d][vmsk] = nrm_list[d]
  end
  dmap    = nrm_map:norm(2,1):squeeze()
  d2_map  = nrm_map:sub(1,2):clone():norm(2,1):squeeze()
  phi_map = (nrm_map[3]):cdiv(dmap):clone():asin()
  tht_map = (nrm_map[1]):clone():cdiv(d2_map):acos()
  mask    = (nrm_map[2]):lt(0)
  tht_map[mask] = -tht_map[mask]
  phi_map[vmsk:eq(0)] = 0
  tht_map[vmsk:eq(0)] = 0
  return nrm_map:clone(), phi_map:clone(), tht_map:clone()
end


function pointcloud:get_depth_map_unclipped()
  local dmap, vmsk, rmsk = self:get_depth_map()
  local max_depth = dmap[vmsk]:max()
  dmap[rmsk]      = 1.25 * max_depth
  vmsk = nil
  rmsk = nil 
  collectgarbage()
  return dmap
end

function pointcloud:get_xyz_map_unclipped(H)
  if H then
    local xyz_list, phi_list, theta_list = self:get_xyz_list_unclipped(H)
    local xyz   = xyz_list:clone():reshape(3,self.height,self.width):clone():contiguous()
    local phi   = phi_list:clone():reshape(self.height,self.width):clone():contiguous()
    local theta = theta_list:clone():reshape(self.height,self.width):clone():contiguous()
    xyz_list = nil
    phi_list = nil
    theta_list = nil
    collectgarbage()
    return xyz, phi, theta
  else
    local dmap = self:get_depth_map_unclipped()
    local theta = torch.range(0,self.width-1):repeatTensor(self.height,1):mul(-2*math.pi/self.width) + math.pi
    local phi   = torch.range(0,self.height-1):repeatTensor(self.width,1):t():mul(-self.meta.elevation_per_point)+math.pi/2
    local xyz   = torch.zeros(3,self.height,self.width)
    xyz[1] = dmap:clone():cmul(phi:clone():cos()):cmul(theta:clone():cos())
    xyz[2] = dmap:clone():cmul(phi:clone():cos()):cmul(theta:clone():sin())
    xyz[3] = dmap:clone():cmul(phi:clone():sin())
    dmap = nil
    collectgarbage()
    return xyz, phi, theta
  end

end

function pointcloud:get_depthxy_map_unclipped()
  local depth           = self:get_depth_map_unclipped()
  local xyz, phi, theta = self:get_xyz_map_unclipped()
  local depth2d         = depth:clone():cmul(phi:clone():cos())
  depth = nil
  xyz   = nil
  phi   = nil
  theta = nil
  return depth2d
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

function pointcloud:get_xyz_list_transformed(omsk, H)
  local points = self:get_xyz_list(omsk)
  return (H*torch.cat(points,torch.ones(points:size(1),1)):t()):t():sub(1,-1,1,3)
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

function pointcloud:get_normal_list_transformed(omsk, H)
  local normals = self:get_normal_list(omsk)
  local H_new = H:clone()
  H_new[{{1,3},{4}}] = 0
  return (H_new*torch.cat(normals,torch.ones(normals:size(1),1)):t()):t():sub(1,-1,1,3)
end

function pointcloud:get_depthxy_list(omsk)
  omsk  = omsk or self:get_valid_masks()
  local dmap = self:get_depthxy_map()
  local depthxy_list = mapToList(dmap, omsk)
  dmap = nil
  collectgarbage()
  return depthxy_list
end

function pointcloud:get_area_list(omsk)
    omsk  = omsk or self:get_valid_masks()
    local amap = self:get_area_map()
    local area_list = mapToList(amap, omsk)
    amap = nil
    collectgarbage()
    return area_list
end
function pointcloud:get_depth_list_unclipped()
  return mapToList(self:get_depth_map_unclipped(), torch.ones(self.height, self.width):byte())
end

function pointcloud:get_xyz_list_unclipped(H)
  local xyz, phi, tht = self:get_xyz_map_unclipped()
  local msk = torch.ones(self.height, self.width):byte()
  local xyz_list = mapToList(xyz,msk):clone()
  local phi_list = mapToList(phi,msk):clone()
  local tht_list = mapToList(tht,msk):clone()
  if H then
    local c        = self:get_center_transformed(H)
    xyz_list       = (H*torch.cat(xyz_list,torch.ones(xyz_list:size(1),1)):t()):t():sub(1,-1,1,3)
    local xyzt     = xyz_list:t():clone()
    local dpt_list = ((xyzt[1] - c[1]):pow(2) + (xyzt[2] - c[2]):pow(2) + (xyzt[3] - c[3]):pow(2)):sqrt()
    local d2d_list = ((xyzt[1] - c[1]):pow(2) + (xyzt[2] - c[2]):pow(2)):sqrt()
    
    phi_list       = (xyzt[3]-c[3]):cdiv(dpt_list):asin()
    tht_list       = (xyzt[1]-c[1]):cdiv(d2d_list):acos()
    local mask     = (xyzt[2]-c[2]):lt(0)
    tht_list[mask] = -tht_list[mask]
    mask           = d2d_list:eq(0)
    tht_list[mask] = 0
    mask           = dpt_list:gt(0)    
    dpt_list = nil
    d2d_list = nil
    xyzt     = nil
    c        = nil
    mask     = nil
    collectgarbage()
  end
  xyz = nil
  phi = nil
  tht = nil
  msk = nil
  return xyz_list, phi_list, tht_list
end

function pointcloud:get_depthxy_list_unclipped()
  return mapToList(self:get_depthxy_map_unclipped(), torch.ones(self.height, self.width):byte())
end


-- flatten functions

local function adjust_dtheta(dtheta)
  local mid = dtheta:gt(-math.pi):cmul(dtheta:le(math.pi)):type('torch.DoubleTensor')
  local top = dtheta:gt(math.pi):type('torch.DoubleTensor')
  local bot = dtheta:le(-math.pi):type('torch.DoubleTensor')
  mid:cmul(dtheta:clone())
  top:cmul(dtheta:clone():add(-math.pi*2))
  bot:cmul(dtheta:clone():add(math.pi*2))
  dtheta = mid+top+bot
  mid = nil
  top = nil
  bot = nil
  collectgarbage()
  return dtheta
end

local function get_theta_corners(thresh,xyz,depth,stheta,height,width)
  for i=1,3 do
    xyz[i]:cdiv(depth)
  end
  
  local phi_xyz = torch.asin(xyz[3]:clone())
  
  local theta_xyz = torch.acos(xyz[1]:clone():cdiv(phi_xyz:clone():cos()))
  theta_xyz[phi_xyz:clone():cos():eq(0)] = 0
  theta_xyz[xyz[2]:clone():lt(0)] = theta_xyz[xyz[2]:clone():lt(0)]:mul(-1)
  
  local kernel_l = torch.Tensor({{0,0,0},{1,0,0},{0,0,0}})
  local kernel_r = torch.Tensor({{0,0,0},{0,0,1},{0,0,0}})

  conv = torch.conv2(stheta:clone(), kernel_l, 'F')
  theta_l = (conv:sub(2,height+1,2,width+1):clone())
  
  conv = torch.conv2(stheta:clone(), kernel_r, 'F')
  theta_r = (conv:sub(2,height+1,2,width+1):clone())
  
  dtheta = (theta_r-theta_l):abs()
  
  mask_lt = dtheta:lt(2*math.pi/3)
  mask_md = dtheta:ge(2*math.pi/3):cmul(dtheta:le(4*math.pi/3))
  mask_gt = dtheta:gt(4*math.pi/3)
  
  dtheta[mask_md] = 0
  dtheta[mask_gt] = dtheta[mask_gt]:mul(-1):add(2*math.pi)
  
  local localmax = torch.zeros(height,width)
  
  torch.range(1,height):apply(function(h)
    local dh = dtheta[h]
    local dwn = dh[1]
    local dwp = 0
    local dwc = 0
    
    torch.range(1,width):apply(function(w)
      if w == 1 then
        dwc = dwn
        dwn = dh[w+1]
      elseif w < width then
        dwp = dwc
        dwc = dwn
        dwn = dh[w+1]
        
        if dwc > dwp and dwc > dwn then
          localmax[h][w] = 1
        end
      end
      
    end)
  end)
  
  dtheta:cmul(localmax)
  
  local mask_theta = dtheta:gt(thresh)
  
  collectgarbage()
  
  return mask_theta
  
end

local function get_plane_corners(thresh,xyz,depth,snormal,sdd,mask0,height,width)
  local kernel
  local conv
  local mask1
  local mask2
  local mask3
  local mask4
  local mask5
  local mask6
  local mask7
  local mask8
  
  local dplane_ll
  local dplane_rr
  
  kernel = torch.Tensor({{0,0,0},{1,0,0},{0,0,0}})
  conv = torch.conv2(depth:clone(),kernel,'F')
  mask1 = conv:sub(2,height+1,2,width+1):clone():eq(0)
  mask1 = (mask1-mask0):eq(1)
  
  kernel = torch.Tensor({{0,0,0},{0,0,1},{0,0,0}})
  conv = torch.conv2(depth:clone(),kernel,'F')
  mask2 = conv:sub(2,height+1,2,width+1):clone():eq(0)
  mask2 = (mask2-mask0):eq(1)
  
  mask1 = (mask1+mask2):gt(0)
  mask2 = mask1:clone()
  
  torch.range(1,height):apply(function(h)
    local m1 = mask1:select(1,h)
    local m2 = mask2:select(1,h)
    torch.range(1,width):apply(function(w)
      if w == 1 or w == width then
        m2[w]=0
      elseif m1[w-1] == 1 and m1[w+1] == 1 and m1[w] == 0 then
        m2:sub(w-1,w+1):fill(0)
        m1[w]=1
      end
    end)
  end)
  
  kernel = torch.zeros(5,5)
  kernel[3][1] = 1
  
  conv = torch.Tensor(3,height+4,width+4)
  wght = torch.conv2(torch.ones(xyz[1]:size()), kernel, 'F')
  for i = 1,3 do
    conv[i] = torch.conv2(xyz[i]:clone(), kernel, 'F')
    conv[i][wght:gt(0)] = conv[i][wght:gt(0)]:cdiv(wght[wght:gt(0)])
  end
  dplane_ll = conv:sub(1,3,3,height+2,3,width+2):clone()
  dplane_ll = dplane_ll:clone():cmul(snormal):sum(1):squeeze():add(sdd:clone())
  mask3 = dplane_ll:gt(thresh):type('torch.DoubleTensor')
  
  kernel = torch.zeros(5,5)
  kernel[3][5] = 1
  
  conv = torch.Tensor(3,height+4,width+4)
  wght = torch.conv2(torch.ones(xyz[1]:size()), kernel, 'F')
  for i = 1,3 do
    conv[i] = torch.conv2(xyz[i]:clone(), kernel, 'F')
    conv[i][wght:gt(0)] = conv[i][wght:gt(0)]:cdiv(wght[wght:gt(0)])
  end
  local dplane_rr = conv:sub(1,3,3,height+2,3,width+2):clone()
  dplane_rr = dplane_rr:clone():cmul(snormal):sum(1):squeeze():add(sdd:clone())
  mask4 = dplane_rr:gt(thresh):type('torch.DoubleTensor')
  
  local mask7 = (mask3+mask4):gt(0)
  mask7[mask2]=1
  mask7[mask0]=0
  mask7:select(2,1):fill(0)
  mask7:select(2,width):fill(0)

  local mask8 = mask7:clone():mul(-1):add(1)
  
  mask8[mask1]=0
  
  collectgarbage()
  
  return mask7,mask8
  
end

local function get_corners(xyz,depth,snormal,stheta,sdd,mask0,theta_thresh,plane_thresh,height,width)
  
  local mask_theta = get_theta_corners(theta_thresh, xyz:clone(), depth:clone(), stheta:clone(), height, width)
  local mask_plane,mask_same_plane = get_plane_corners(plane_thresh, xyz:clone(), depth:clone(), snormal:clone(), sdd:clone(), mask0, height, width)
  mask_theta:cmul(mask_same_plane)
  local mask_corner = (mask_theta+mask_plane):gt(0)
  
  collectgarbage()
  
  return mask_corner
end

local function get_connections(xyz,depth,snormal,sdd,plane_thresh,height,width)
  
  local dpt_thresh_1 = 3*2*math.pi*20*1000/width
  local dpt_thresh_2 = 2*math.pi*15/width
  local kernel
  local conv
  local dplane_l
  local dpoint_l
  
  kernel = torch.zeros(5,5)
  kernel[3][1] = 1
  
  conv = torch.Tensor(3,height+4,width+4)
  wght = torch.conv2(torch.ones(xyz[1]:size()), kernel, 'F')
  for i = 1,3 do
    conv[i] = torch.conv2(xyz[i]:clone(), kernel, 'F')
    conv[i][wght:gt(0)] = conv[i][wght:gt(0)]:cdiv(wght[wght:gt(0)])
  end
  dplane_l = conv:sub(1,3,3,height+2,3,width+2):clone()
  dplane_l = dplane_l:clone():cmul(snormal):sum(1):squeeze():add(sdd:clone())
  local mask_con = dplane_l:clone():abs():lt(plane_thresh)
  
  dpoint_l = conv:sub(1,3,3,height+2,3,width+2):clone()
  wght = wght:sub(3,height+2,3,width+2):clone():repeatTensor(3,1,1)
  dpoint_l = dpoint_l:add(xyz:clone():mul(-1):cmul(wght)):norm(2,1):squeeze()
  local mask_point_1 = dpoint_l:gt(dpt_thresh_1)
  dpoint_l:cdiv(depth)
  local mask_point_2 = dpoint_l:gt(dpt_thresh_2)
  
  mask_con[mask_point_1] = 0
  mask_con[mask_point_2] = 0
  
  wght = nil
  conv = nil
  dplane_l = nil
  dpoint_l = nil
  mask_point_1 = nil
  pask_point_2 = nil
  collectgarbage()
  
  return mask_con
  
end

function pointcloud:get_connections_and_corners()
  
  local height = self.height
  local width  = self.width
  local imask  = self:get_inverse_masks()
  
  local xyz    = self:get_xyz_map()  
  local depth  = self:get_depth_map()
  
  local noise             = self:get_noise()
  local thresh_theta      = math.pi/4
  local thresh_plane_conn = math.min(0.25*self.meter,0.01*self.meter+3*noise)
  local thresh_plane_corn = math.min(0.50*self.meter,0.10*self.meter + 3*noise)
  local thresh_phi        = math.max(3*math.pi/8, math.pi/2-math.pi/8 - noise*math.pi/20)
  local thresh_height     = 0.1*height
  
  local nmp, nphi, ntht, nrsd, nvmsk, nimsk = self:get_normal_map()
  
  local phmask = nphi:clone():abs():ge(thresh_phi)
  local hmask  = {{1,thresh_height},{}}
  
  local mask_corner = get_corners(xyz:clone(),  depth:clone(), nmp:clone(),
                                  ntht:clone(), nrsd:clone(),  imask,
                                  thresh_theta, thresh_plane_corn,
                                  height, width)
  local mask_connct = get_connections(xyz:clone(),  depth:clone(), nmp:clone(),
                                      nrsd:clone(), thresh_plane_conn,
                                      height,width)
                                  
  mask_corner[imask ] = 0
  mask_corner[phmask] = 0
  mask_corner[hmask ] = 0
  
  mask_connct[imask ] = 0
  mask_connct[phmask] = 0
  mask_connct[hmask ] = 0
  
  return mask_connct, mask_corner

end

function pointcloud:get_flattened_image(scale,numCorners,H)
  
  if not scale then
    scale = self.meter/100
  elseif scale < self.meter/1000 then
    scale = self.meter/1000
  end
  
  if not numCorners then
    numCorners = 25
  elseif numCorners > 100 or numCorners < 1 then
    numCorners = 25
  end
  
  -- get the points and depth
  local pts  = self:get_xyz_map()
  local centroid = self.centroid
  if H then
    pts = self:get_xyz_map_transformed(H)
    centroid = self:get_center_transformed(H)
  end
  pts        = pts:sub(1,2):clone():contiguous()
  local d2d  = (pts[1]-centroid[1]):pow(2):add((pts[2]-centroid[2]):pow(2)):sqrt():clone()
  local height = 2*d2d:max()/scale
  local width  = height
  minv = centroid-d2d:max()
  maxv = centroid+d2d:max()
  
  local hght   = self.height
  local wdth   = self.width
  
  -- make a list for corners
  local corners  
  if numCorners and numCorners > 0 then
      corners = torch.zeros(numCorners,2)
  end
  local corners_map_filled = torch.zeros(hght,wdth)
  
  -- get the wall connections and the wall corners
  local connections, corners_map = self:get_connections_and_corners()
  
  -- make the flattened images
  local imagez        = torch.zeros(height,width)
  local image_corners = imagez:clone()
  local imaget    = torch.zeros(height,width):clone():contiguous()
  local theta_cnt = torch.zeros(height,width):clone():contiguous()
  
  -- get the coordinates and points
  local crds = pts:clone()
  local mnv  = minv:sub(1,2):repeatTensor(hght,wdth,1):transpose(2,3):transpose(1,2)
  
  crds:add(mnv:mul(-1)):div(scale):floor()
  pts = self:get_xyz_map()
  
  -- get the normal map
  local nmp, nphi, ntht, ndd, nvmsk, nimsk = self:get_normal_map()
  if H then
    nmp, nphi, ntht = self:get_normal_map_transformed(H)
  end
  
  -- clear some space
  nmp   = nil
  nphi  = nil
  ndd   = nil
  nvmsk = nil
  nimsk = nil
  mnb   = nil
  collectgarbage()
  
  local weights = torch.zeros(self.height,self.width)
  weights:add(image.combine(self:get_depth_map():pow(2)))
  weights:add(image.combine(self:get_area_map()))
  weights:add(image.combine(self:get_reverse_density_map()))
  
  -- call the cfunction
  libpc.flatten_image_with_theta(torch.data(imagez), torch.data(image_corners), torch.data(corners_map_filled),
                                          torch.data(imaget), torch.data(theta_cnt),
                                          torch.data(crds:clone():contiguous()), torch.data(pts:clone():contiguous()),
                                          torch.data(connections:clone():contiguous()), 
                                          torch.data(corners_map:clone():contiguous()),
                                          torch.data(ntht:clone():contiguous()),
                                          torch.data(weights:clone():contiguous()),
                                          hght, wdth, height, width)
  
  -- mask out imaget and nmflat
  local imaget_mask = theta_cnt:gt(0)
  imaget[imaget_mask] = imaget[imaget_mask]:cdiv(theta_cnt[imaget_mask])
  imaget[imaget_mask:eq(0)] = -math.pi*2
  
  nm_flat = torch.zeros(3,height,width)
  nm_flat[1][imaget_mask] = imaget[imaget_mask]:cos()
  nm_flat[2][imaget_mask] = imaget[imaget_mask]:sin()

  -- normalize some
  local mean_height = (imagez:clone():sum())/(imagez:clone():cdiv(imagez:clone()+very_small_number):sum())
  local stdv_height = math.sqrt((imagez:clone():add(-mean_height):pow(2):sum())/(imagez:clone():cdiv(imagez:clone()+very_small_number):sum()))
  local max_height  = mean_height+stdv_height
  local show_thresh = 0.1*max_height
  imagez            = imagez:clone():gt(max_height):type('torch.DoubleTensor'):mul(max_height):add(
  imagez:clone():le(max_height):type('torch.DoubleTensor'):cmul(imagez)):cmul(imagez:clone():gt(show_thresh):type('torch.DoubleTensor'))
  imagez:div(imagez:max()+very_small_number)
  imagez:cmul(imagez:gt(0.05):type('torch.DoubleTensor'))
  image_corners:div(image_corners:max()+very_small_number):cmul(image_corners:gt(0.05):type('torch.DoubleTensor'))
  
  if numCorners and numCorners > 0 then
    local ps = math.ceil(0.10*self.meter/scale)
    local bs = math.ceil(0.05*self.meter/scale)
    local ds =  3
    local cc = 0
    local image_corners_orig = image_corners:clone()
        
    while cc < numCorners and image_corners:max() > 0 do
      local maxmap,maxord = image_corners:max(1)
      local mmmap,mmord = maxmap:max(2)
      local x = mmord[1][1]
      local y = maxord[1][x]
      local patch = image_corners_orig:sub(math.max(1,y-ps),
                                           math.min(height,y+ps),
                                           math.max(1,x-ps),
                                           math.min(width,x+ps))
      if patch:max() == image_corners[y][x] then
        cc = cc+1
        corners[cc] = torch.Tensor({y,x})
        image_corners:sub(math.max(1,y-bs),
        math.min(height,y+bs),
        math.max(1,x-bs),
        math.min(width,x+bs)):mul(0)
      else
        image_corners[y][x] = 0
      end
    end
    if cc < numCorners then
      if cc == 0 then
        corners = nil
      else
        corners = corners:sub(1,cc):clone()
      end
    end
    image_corners = image_corners_orig
  end
  
  imagez = imagez:clone():repeatTensor(3,1,1)
  
  mean_height = nil
  stdv_height = nil
  max_height  = nil
  shor_thresh = nil
  ranges      = nil
  minv        = nil
  maxv        = nil
  pix         = nil
  connections = nil
  corners_map = nil
  mask        = nil
  pts         = nil
  crds        = nil
  
  collectgarbage()
  
  return imagez, corners, image_corners, scale, imaget, imaget_mask, nm_flat
end


-- pose functions

function pointcloud:set_transformation_matrix(mat)
  if type(mat) == "userdata" and mat:size(1)==4 then
    self.transformation_matrix = mat
  end
end

function pointcloud:get_transformation_matrix()
  return self.transformation_matrix
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
  
  fout:write('element face  0\n')
  fout:write('property list uchar int vertex_indices\n')
  fout:write('end_header\n')
  
  write_to_ascii(fout, count, xyz_list, rgb_list, intensity_list, normal_list, hw_list)
  
  fout:close()
  collectgarbage()

end

function pointcloud:save_setup(use_global_or_matrix, use_rgb, use_intensity, use_normal, use_hw, mask)

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
  
  vmsk = mask or vmsk
  local count          = vmsk:double():sum()
  
  local xyz_list = self:get_xyz_list(vmsk)
  if(use_global_or_matrix) then
    if type(use_global_or_matrix) == "userdata" and use_global_or_matrix:size(1)==4 then
      xyz_list = self:get_xyz_list_transformed(vmsk, use_global_or_matrix)  
    else
      local mat = self:get_transformation_matrix()
      if mat then
        xyz_list = self:get_xyz_list_transformed(vmsk, mat)  
      end
      mat = nil
      collectgarbage()
    end
  end


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
    if use_global_or_matrix then
      if type(use_global_or_matrix) == "userdata" and use_global_or_matrix:size(1)==4 and use_global_or_matrix:size(2) == 4 then
        normal_list = self:get_normal_list_transformed(vmsk, use_global_or_matrix)
      else
        local mat = self:get_transformation_matrix()
        if mat then
          normal_list = self:get_normal_list_transformed(vmsk, mat)
        end
        mat = nil
        collectgarbage()
      end
    end
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

function pointcloud:save_to_xyz_file(filename, use_global_or_matrix, use_rgb, use_intensity, use_normal, use_hw, mask)

  local count, xyz_list, rgb_list, intensity_list, normal_list, hw_list = self:save_setup(use_global_or_matrix, use_rgb, use_intensity, use_normal, use_hw, mask)
  
  write_to_xyz(filename, count, xyz_list, rgb_list, intensity_list, normal_list, hw_list)
  
end

function pointcloud:save_to_ply_file(filename, use_global_or_matrix, use_rgb, use_intensity, use_normal, use_hw, mask)

  local count, xyz_list, rgb_list, intensity_list, normal_list, hw_list = self:save_setup(use_global_or_matrix, use_rgb, use_intensity, use_normal, use_hw, mask)
  
  write_to_ply(filename, count, xyz_list, rgb_list, intensity_list, normal_list, hw_list)
  
end

function pointcloud:save_to_data_file(filename)
  
  torch.save(filename,self)
  
end

--accessor, so we follow pattern of not directly accessing fields especially when outside of class
function pointcloud:get_meter()
  return self.meter
end

function pointcloud:save_for_julia(filename)
  local pc_object = {}
  pc_object.height = self.height 
  pc_object.width = self.width
  pc_object.count = self.count
  pc_object.meter= self.meter
  pc_object.centroid = self.centroid:clone()
  pc_object.meta = self.meta
  pc_object.def_mind = self.def_mind
  pc_object.def_maxd = self.def_maxd
  pc_object.depth_map = self.depth_map:clone()
  pc_object.rgb_map = self.rgb_map:clone()
  pc_object.intensity_map = self.intensity_map:clone()
  pc_object.xyz_phi_map = self.xyz_phi_map:clone()
  pc_object.xyz_theta_map = self.xyz_theta_map:clone()
  pc_object.depth_valid_mask = self.depth_valid_mask:clone()
  pc_object.rgb_valid_mask = self.rgb_valid_mask:clone()
  pc_object.intensity_valid_mask = self.intensity_valid_mask:clone()
  pc_object.depth_inverse_mask = self.depth_inverse_mask:clone()
  pc_object.rgb_inverse_mask = self.rgb_inverse_mask:clone()
  pc_object.intensity_inverse_mask = self.intensity_inverse_mask:clone()
  pc_object.xyz_map = self.xyz_map:clone()
  pc_object.hw_indices_map = self.hw_indices_map:clone()
  pc_object.index_map = self.index_map:clone()
  pc_object.xyz_minval = self.xyz_minval:clone()
  pc_object.xyz_maxval = self.xyz_maxval:clone()
  pc_object.xyz_radius = self.xyz_radius:clone()
  pc_object.centroid = self.centroid:clone()
  pc_object.normal_phi_map = self.normal_phi_map:clone()
  pc_object.normal_theta_map = self.normal_theta_map:clone()
  pc_object.normal_map = self.normal_map:clone()
  pc_object.normal_inverse_mask = self.normal_inverse_mask:clone()
  pc_object.normal_valid_mask = self.normal_valid_mask:clone()
  pc_object.normal_residual_map = self.normal_residual_map:clone()
  pc_object.transformation_matrix = self.transformation_matrix or torch.eye(4)
  pc_object.map_u = self.map_u:clone()
  pc_object.map_d = self.map_d:clone()
  pc_object.map_l = self.map_l:clone()
  pc_object.map_r = self.map_r:clone()

  torch.save(filename,pc_object, "ascii")

  
--[[
  
  if input.global_pose then
    self:set_global_pose(input.global_pose)
  end
  
  if input.global_rotation then
    self:set_global_rotation(input.global_rotation)
  end
  
]]--

end