local io = require 'io'
local path = require 'path'
local kdtree = kdtree.kdtree
local ffi = require 'ffi'
local ctorch = util.ctorch -- ctorch needs to be loaded before we reference THTensor stuff in a cdef
local log = require '../util/log'
local rotate_translate = geom.quaternion.rotate_translate
local fix_newline = PointCloud.fix_newline
local colors = require '../opencv/types/Colors.lua'

ffi.cdef
[[
    int get_index_and_mask(long* index_map, short* mask_map, double* points, short* hwindices,
                       long length, int height, int width);
                                                 
    int downsample(double* downsampled_points, double* downsampled_rgb, 
                             int* downsampled_count, double* coord_map, 
                             double* points_map, double* rgb_map, 
                             int height, int width);
    
    int flatten_image(double* imagez, double* image_corner,
                                          double* coord_map, double *points,
                                          char* connection_map, char* corners_map,
                                          double* corners_map_filled,
                                          int pan_hght, int pan_wdth,
                                          int img_hght, int img_wdth);
    
   int theta_map(double* theta_map, double* centered_point_map, int height, int width);
   int phi_map(double* phi_map, double* centered_point_map, int height, int width);
   int theta_map_smooth(double* smoothed_theta_map, double* theta_map, char * extant_map, 
                     int height, int width, int max_win, double max_theta_diff);
   int phi_map_smooth(double* smoothed_phi_map, double* phi_map, char * extant_map, 
                     int height, int width, int max_win, double max_phi_diff); 
   int get_step_maps(int* step_up, int* step_down, int* step_left, int* step_right, 
                  double* xyz, double* theta,
                  double min_step_size, double max_step_size, int height, int width);
   int theta_map_var(double* theta_map, double* centered_point_map, 
                  int* step_left, int* step_right,
                  int height, int width);
   int phi_map_var(double* phi_map, double* centered_point_map, 
                int* step_up, int* step_down,
                int height, int width);
   
   int remap_points(double* depth_sum, double* depth_num, double* phi_new, double* theta_new,
                 double* depth_old, char* rmask_old, double* phi_old, double* theta_old,
                 int hght_old, int wdth_old, int hght_new, int wdth_new,
                 double phi_max, double phi_stp, double theta_max, double theta_stp);
   
]]

local libpc   = util.ffi.load('libpointcloud')

-- angular to radians and back
local pi = math.pi
local r2d = 180/pi
local d2r = pi/180

local PointCloud = Class()

PointCloud.PC_OD_EXTENSION = '.od'
PointCloud.PC_ASCII_EXTENSION = '.xyz'
PointCloud.faro_degree_above = 90
PointCloud.faro_degree_below = 60
PointCloud.very_small_number = 0.00000000000000001
PointCloud.fudge_number = 1000

local function round(dbl)
  local du = math.ceil(dbl)
  local dl = math.floor(dbl)
  if math.abs(du-dbl) <= math.abs(dl-dbl) then
    return du
  else
    return dl
  end
end

-- rgb_map is a 2D equirectangular grid of rgb values which corresponds to xyz_map
function PointCloud:load_rgb_map(imagefile, type)
   type = type or "byte"
   local rgb_map = image.load(imagefile, type)
   if rgb_map then 
      self.rgb_map = rgb_map
      self.rgb = self:get_rgb()
   else 
      print("WARNING: no rgb map loaded")
   end
end

function PointCloud:get_index_and_mask(force)
   if (not self.index_map) or force then
      
      if not self.hwindices then
        error("this pointcloud has no hwindices. can't make maps")
      end
      
      local height = self.height
      local width = self.width
      local index_map = torch.ones(height,width):type('torch.LongTensor'):clone()
      local mask_map  = torch.ones(height,width):type('torch.ShortTensor'):clone()
      
      libpc.get_index_and_mask(torch.data(index_map), torch.data(mask_map), torch.data(self.points:clone():contiguous()), 
                               torch.data(self.hwindices:clone():contiguous()), self.count, height, width)
      
      self.index_map = index_map:clone()
      self.mask_map  = mask_map:clone():type('torch.ByteTensor')

      collectgarbage()
    
   end
   return self.index_map, self.mask_map
end

function PointCloud:get_max_radius()
   return torch.max(self.radius)
end

function PointCloud:get_points(force)
   if (not self.pointsT) or force then
      -- we should do index like this and store data the opposite way around
      self.pointsT = self.points:transpose(1,2):contiguous()
   end
   return self.pointsT
end

function PointCloud:get_xyz_map(force)
   if not self.xyz_map or force then
      local points      = self:get_points()
      local index, mask = self:get_index_and_mask()

      self.xyz_map = util.addr.remap(points,index,mask)
   end
   return self.xyz_map
end

function PointCloud:get_xyz_map_no_mask(force)
    if not self.xyz_map_no_mask or force then
        local points      = self:get_points()
        local index,mask = self:get_index_and_mask()
        self.xyz_map_no_mask = util.addr.remap(points,index,mask,torch.Tensor(3,self.height,self.width))
    end
    return self.xyz_map_no_mask
end

function PointCloud:get_depth_map_no_mask()
  local xyz_map = self:get_xyz_map_no_mask()
  local depth_map = xyz_map:norm(2,1):squeeze()
  depth_map[self.mask_map] = 0
  return depth_map
end

function PointCloud:get_depth_map()
   local xyz_map = self:get_xyz_map():clone()
   local depth_map = xyz_map:norm(2,1):squeeze()
   depth_map[self.mask_map] = 0
   return depth_map
end

-- put rgb values such as those read in from an xyzrgb format into a "map"
function PointCloud:get_rgb_map(force) 
  if not(self.rgb_map) or force then
   if self.rgb then
      rgbT = self:get_rgb():transpose(1,2):contiguous()
      index, mask = self:get_index_and_mask()
      self.rgb_map = util.addr.remap(rgbT,index,mask)
   else
      print("no rgb_map and no rgb values")
      return
   end
  end
  return self.rgb_map
end

-- put rgb values such as those read in from an xyzrgb format into a "map"
function PointCloud:get_rgb_map_no_mask()
  if self.rgb then
    rgbT = self:get_rgb():transpose(1,2):contiguous()
    index, mask = self:get_index_and_mask()
    local rgb_map_no_mask = util.addr.remap(rgbT,index,mask,torch.Tensor(3,self.height,self.width))
    return rgb_map_no_mask
  else
    print("no rgb_map and no rgb values")
    return nil
   end
   
end

-- self.rgb is a list of rgb values which corresponds to self.points,
-- can create these from an rgb_map image if they don't exist
function PointCloud:get_rgb()
   if ((not self.rgb) or (self.rgb:size(1) ~= self.points:size(1))) then 
      if self.rgb_map then
         img = self.rgb_map
         index,mask = self:get_index_and_mask()
         
         n_chan = img:size(1)
         n_pts  = self.points:size(1)
         
         -- make rgb same type as rgb_map
         mask = mask:clone():eq(0) -- invert the mask
         rgb = img:clone():resize(n_chan,n_pts)
         for chan = 1,n_chan do 
            rgb[chan] = img[chan][mask]
         end
         -- make the matrix Nx3 like self.points
         self.rgb = rgb:transpose(1,2):contiguous()
      else
         print("don't know where to get the data perhaps you need to load the rgb image load_rgb_map()")
         return nil
      end
   end
   return self.rgb
end

-- returns list of depths corresponding to points
function PointCloud:get_depth(force)
   if (not self.depth) or force then 
      points     = self.points
      self.depth = points:clone():norm(2,2):squeeze()
   end
   return self.depth
end

-- returns list of depths corresponding to points
function PointCloud:get_intensity_cost(scale, mid_value)
   scale = scale or 1
   mid_value = mid_value or 127
   rgb_cost = self:get_rgb():double()
   rgb_cost:add(-mid_value):mul(scale/(3*255)):abs()
   rgb_cost = rgb_cost:sum(2):squeeze()
   return rgb_cost
end

function PointCloud:get_normal_phi_theta(force)
  if not(self.phi_map and self.theta_map and self.normal_mask) or force then
    local tic = log.toc()
    local point_map = self:get_xyz_map_no_mask()
    local height = self.height
    local width = self.width
    self.phi_map = torch.zeros(height,width):clone():contiguous()
    self.theta_map = torch.zeros(height,width):clone():contiguous()
  
    --[[]]
    libpc.phi_map(torch.data(self.phi_map),torch.data(point_map:clone():contiguous()),height,width)
    libpc.theta_map(torch.data(self.theta_map),torch.data(point_map:clone():contiguous()),height,width)
    --[[]]
  
    self.normal_mask = self.phi_map:lt(-10):add(self.theta_map:lt(-10)):gt(0)
    self.phi_map[self.normal_mask] = 0
    self.theta_map[self.normal_mask] = 0

    print('get_normal_phi_theta: '..(log.toc()-tic))
  end
  return self.phi_map, self.theta_map, self.normal_mask
end

function PointCloud:get_normal_map()
    local tic = log.toc()
    local phi,theta,mask = self:get_normal_phi_theta()
    local height = self.height
    local width = self.width
    local nmp = torch.zeros(3,height,width)
  
    nmp[3] = phi:clone():sin()
    nmp[2] = phi:clone():cos():cmul(theta:clone():sin())
    nmp[1] = phi:clone():cos():cmul(theta:clone():cos())
    
    local dd = nmp:clone():cmul(self:get_xyz_map_no_mask()):sum(1):squeeze()
    local neg = dd:lt(0)
    
    for i = 1,3 do
      nmp[i][neg] = -nmp[i][neg]
      nmp[i][mask] = 0
    end
    
    dd = nmp:clone():cmul(self:get_xyz_map_no_mask()):sum(1):squeeze()
    
    print('get_normal_map: '..(log.toc()-tic))
    
    return nmp,dd,phi,theta,mask
end

function PointCloud:get_normal_map_varsize(force)

  if not self.var_normal_map or force then
    local tic = log.toc()
    local point_map = self:get_xyz_map_no_mask()
    local height = self.height
    local width = self.width
    local phi = torch.zeros(height,width):clone():contiguous()
    local theta = torch.zeros(height,width):clone():contiguous()
    
    --[[]]
    local mu,md,ml,mr = self:get_lookup_map()
    libpc.phi_map_var(torch.data(phi),torch.data(point_map:clone():contiguous()),
                      torch.data(mu:contiguous()), torch.data(md:contiguous()),
                      height,width)
    libpc.theta_map_var(torch.data(theta),torch.data(point_map:clone():contiguous()),
                      torch.data(ml:contiguous()), torch.data(mr:contiguous()),
                      height,width)
    --[[]]
  
    local mask = phi:lt(-10):add(theta:lt(-10)):gt(0)
    phi[mask] = 0
    theta[mask] = 0
    
    local nmp = torch.zeros(3,height,width)
  
    nmp[3] = phi:clone():sin()
    nmp[2] = phi:clone():cos():cmul(theta:clone():sin())
    nmp[1] = phi:clone():cos():cmul(theta:clone():cos())
    
    local dd = nmp:clone():cmul(self:get_xyz_map_no_mask()):sum(1):squeeze()
    local neg = dd:lt(0)
    
    for i = 1,3 do
      nmp[i][neg] = -nmp[i][neg]
      nmp[i][mask] = 0
    end
    
    dd = nmp:clone():cmul(self:get_xyz_map_no_mask()):sum(1):squeeze()
    
    self.var_normal_map = nmp
    self.var_normal_dd = dd
    self.var_phi = phi
    self.var_theta = theta
    self.var_normal_mask = mask
  end
  return self.var_normal_map,self.var_normal_dd,self.var_phi,self.var_theta,self.var_normal_mask
    --return nmp,dd,phi,theta,mask
end

function PointCloud:get_smooth_normal(max_win, phi_diff, theta_diff, phi, theta, mask)

  if not self.smooth_normal_map then
    local tic = log.toc()
    if not win then
      max_win = 10
    end
    if not phi_diff then
      phi_diff = math.pi/6
    end
    if not theta_diff then
      theta_diff = math.pi/6
    end
  
    local height = self.height
    local width = self.width
  
    if not (phi and theta and mask) then
      phi,theta,mask = self:get_normal_phi_theta()
    end
    local extant_map = mask:clone():eq(0)
    local smooth_phi = phi:clone():contiguous():fill(0)
    local smooth_theta = theta:clone():contiguous():fill(0)
  
    libpc.phi_map_smooth(torch.data(smooth_phi),torch.data(phi:clone():contiguous()),torch.data(extant_map:clone():contiguous()),height,width,max_win,phi_diff)
    libpc.theta_map_smooth(torch.data(smooth_theta),torch.data(theta:clone():contiguous()),torch.data(extant_map:clone():contiguous()),height,width,max_win,theta_diff)
  
    smooth_phi[mask] = 0
    smooth_theta[mask] = 0
    
    local smooth_nmp = torch.zeros(3,height,width)
  
    smooth_nmp[3] = smooth_phi:clone():sin()
    smooth_nmp[2] = smooth_phi:clone():cos():cmul(smooth_theta:clone():sin())
    smooth_nmp[1] = smooth_phi:clone():cos():cmul(smooth_theta:clone():cos())
    
    local dd = smooth_nmp:clone():cmul(self:get_xyz_map_no_mask()):sum(1):squeeze()
    local neg = dd:lt(0)
    
    for i = 1,3 do
      smooth_nmp[i][neg] = -smooth_nmp[i][neg]
      smooth_nmp[i][mask] = 0
    end
    
    dd = smooth_nmp:clone():cmul(self:get_xyz_map_no_mask()):sum(1):squeeze()
  
    print('get_smooth_normal_map: '..(log.toc()-tic))
    
    self.smooth_normal_map = smooth_nmp
    self.smooth_normal_dd = dd
    self.smooth_phi = smooth_phi
    self.smooth_theta = smooth_theta
    self.smooth_normal_mask = mask
    
  end
    
  --return smooth_nmp, dd, smooth_phi, smooth_theta, mask
  return self.smooth_normal_map, self.smooth_normal_dd, self.smooth_phi, self.smooth_theta, self.smooth_normal_mask

end

function PointCloud:get_xyz_unit_vectors()
  local xyz = self:get_xyz_map_no_mask()
  local depth = self:get_depth_map_no_mask()
  local index,mask = self:get_index_and_mask()
  local unitv = xyz:clone():cdiv(depth:repeatTensor(3,1,1))
  unitv[mask:repeatTensor(3,1,1)]=0
  return unitv
end

function PointCloud:get_xyz_phi_theta(force)
  if not (self.xyz_phi_map and self.xyz_theta_map) or force then
    local unitv = self:get_xyz_unit_vectors()
    local index,mask = self:get_index_and_mask()
    local phi = unitv[3]:clone():asin()
    local xy = phi:clone():cos()
    local xdxy = unitv[1]:clone():cdiv(xy)
    local theta = xdxy:clone():acos()
    theta[xy:eq(0)]=0
    theta[xdxy:gt(1)]=0
    theta[xdxy:lt(-1)]=math.pi
    theta[unitv[2]:lt(0)] = theta[unitv[2]:lt(0)]:clone():mul(-1)
    phi[mask]=0
    theta[mask]=0
    self.xyz_phi_map = phi
    self.xyz_theta_map = theta
  end
  return self.xyz_phi_map,self.xyz_theta_map
end

function PointCloud:get_noise()

  local height = self.height
  local width = self.width
  
  local index,mask=self:get_index_and_mask()
  local phi,theta = self:get_xyz_phi_theta()
  local xyz = self:get_xyz_map_no_mask()
  local z = xyz[3]:clone()
  
  local phi_max = phi:max()
  local phi_min = phi:min()
  local phi_rng = phi_max-phi_min
  local phi_stp = phi_rng/(height-1)
  local phi_thr = phi_stp/2
  
  local top_row_mask = phi:le(phi_max):cmul(phi:ge(phi_max-phi_thr))
  
  top_row_mask[mask] = 0
  
  topnoise = z[top_row_mask]:std()
  
  return topnoise
  
end

function PointCloud:get_lookup_map()
  local height = self.height
  local width = self.width
  local xyz = self:get_xyz_map_no_mask():contiguous()
  local phi,theta = self:get_xyz_phi_theta()
  local depth = self.points:sub(1,self.count,1,2):clone():norm(2,2)
  local min_depth = depth:min()
  local max_depth = depth:max()
  local noise = self:get_noise()
  local mindist = 25--math.max(5*min_depth*2*math.pi/width,10)+5*noise
  local maxdist = 100--2.5*max_depth*2*math.pi/width+5*noise
  theta = theta:contiguous()
  
  print(mindist,maxdist,noise,min_depth,max_depth)
  
  local map_u = torch.zeros(2,height,width):int():contiguous()
  local map_d = torch.zeros(2,height,width):int():contiguous()
  local map_l = torch.zeros(2,height,width):int():contiguous()
  local map_r = torch.zeros(2,height,width):int():contiguous() 
  
  libpc.get_step_maps(torch.data(map_u), torch.data(map_d), torch.data(map_l), torch.data(map_r), 
                  torch.data(xyz:contiguous()), torch.data(theta:contiguous()), mindist, maxdist, height, width)
  return map_u,map_d,map_l,map_r
end





--[[ LOADING FUNCTIONS ]]--

function PointCloud:__init(pcfilename, radius, numstd, option)

   log.tic()
   
   self.height = 0;
   self.width = 0;
   self.count = 0;
   
   self.hwindices = nil;
   self.points = nil;
   self.rgb = nil;
   
   self.meter = 1000;
   self.radius = nil
   
   self.index_map = nil;
   self.mask_map = nil;
   
   self.pointT = nil
   
   self.xyz_map = nil
   self.xyz_map_no_mask = nil
   
   self.phi_map = nil;
   self.theta_map = nil;
   self.normal_mask = nil;
   
   self.centroid   = torch.Tensor({{0,0,0}});
   
   local radius_near_far
   
   local default_near = 0.01*self.meter
   local default_far = 25*self.meter

   if radius then 
      if type(radius) == "number" then 
         radius_near_far = torch.zeros(2)
         radius_near_far[1] = default_near
         radius_near_far[2] = radius
      else
         radius_near_far = radius
      end
   else
      radius_near_far = torch.zeros(2)
      radius_near_far[1] = default_near
      radius_near_far[2] = default_far
   end

   if (not numstd) then
      --default number of standard dev prune
      numstd = 3
   end

   if pcfilename then
      if util.fs.is_file(pcfilename) then
        print('loading '..path.basename(pcfilename))
         if util.fs.extname(pcfilename)==PointCloud.PC_ASCII_EXTENSION then
            self:set_pc_ascii_file(pcfilename, radius_near_far, numstd, option)
         elseif util.fs.extname(pcfilename)==PointCloud.PC_OD_EXTENSION then
            self:set_pc_od_file(pcfilename)
         else
            error('arg #1 must either be empty or a valid file: '..pcfilename)
         end
      else
         error('arg #1 must either be empty or a valid file: '..pcfilename)
      end
   end
end

function PointCloud:reset_point_stats()
   self.centroid=torch.Tensor({{0,0,0}})
   
   minval,minind = self.points:min(1)
   maxval,maxind = self.points:max(1)
   
   self.minval   = minval:squeeze()
   self.maxval   = maxval:squeeze()

   self.minind   = minind:squeeze()
   self.maxind   = maxind:squeeze()

   local d = torch.Tensor(2,3)
   d[1] = self.maxval:clone():add(self.centroid:clone():mul(-1)):squeeze()
   d[2] = self.centroid:clone():add(self.minval:clone():mul(-1)):squeeze()
   self.radius = d:max(1):squeeze()
   d = nil
   
   self.count = self.points:size(1)
end

function PointCloud:make_hw_indices()
   local height  = self.height
   local width   = self.width
   local indices = torch.ShortTensor(2,height,width)
   
   local x = torch.range(1,width):resize(1,width):expand(height,width)
   local y = torch.range(1,height):resize(1,height):expand(width,height)
   
   indices[1]:copy(y:t())
   indices[2]:copy(x)
   return indices:resize(2,height*width):transpose(1,2):clone()
end

function PointCloud:set_pc_ascii_file(pcfilename, radius, numstd)

   --first pass to see what type of file it is, whether it has 3, 6 columns, or 8 (h/w first)
   
   fix_newline.fix_newline(pcfilename)
   
   local file = io.open(pcfilename, 'r');
   local line = file:read();
   
   if line == nil or line:len() < 5 then
      error("file did not have enough stuff in it")
   end
   
   local po_cloud_file = false
   -- find first line which starts with a number
   local countHeader = 0
   local key,val = line:match("^([^0-9-]): ([^%s]*)")
   while (key) do 
      -- this is inverted on purpose. Gets switched later [See: switcheroo]
      if (key == "h") then 
         self.width = tonumber(val)
      elseif (key == "w") then
         self.height = tonumber(val)
      end
      line = file:read()
      key,val = line:match("^([^0-9-]): ([^%s]*)")
      countHeader = countHeader + 1
   end
   
   -- this is only for the po_scans
   if ((self.width > 0) and (self.height > 0)) then 
      po_cloud_file = true
   end
   file:close()
   
   -- on first pass determine format
   local countColumns = 0
   for token in string.gmatch(line, "[^%s]+") do
      countColumns = countColumns + 1
   end

   if  (countColumns == 3) then
      self.format = 0 -- po_scan
   elseif  (countColumns == 8) then
      self.format = 1 -- faro_scan
   else
      print(line)
      error("unknown format, input should have either 6 or 8 columns")
   end

   local totalLines = util.fs.exec("wc -l " .. pcfilename)
   
   for token in string.gmatch(totalLines, "[^%s]+") do
      count = tonumber(token) - countHeader
      break
   end
   
   local file = torch.DiskFile(pcfilename, 'r', false)
   
   -- remove header
   
   while (countHeader>0) do 
      file:readString("*l")
      countHeader = countHeader - 1
   end
   
   local xyzrgbTensor =torch.Tensor(torch.File.readDouble(file,countColumns*count)):reshape(count, countColumns)
   file:close()
   
   local offset = 0
   local h,w
   local r, g, b
   
   if self.format==1 then
      offset = 2
      h = xyzrgbTensor:select(2,1)+1
      w = xyzrgbTensor:select(2,2)+1
      r = xyzrgbTensor:select(2,4+offset)
      g = xyzrgbTensor:select(2,5+offset)
      b = xyzrgbTensor:select(2,6+offset)
   end
   
   local x = xyzrgbTensor:select(2,1+offset)
   local y = xyzrgbTensor:select(2,2+offset)
   local z = xyzrgbTensor:select(2,3+offset)
   
   if self.format == 1 then
      -- scale x,y,z
      x = (x*self.meter):floor()
      y = (y*self.meter):floor()
      z = (z*self.meter):floor()
   end
   
   if (po_cloud_file) then 
      -- do points row col switcheroo
      local pts    = xyzrgbTensor[{{},{1+offset,3+offset}}] -- Nx3
      pts    = pts:transpose(1,2):contiguous():resize(3,self.height,self.width)
      
      local pointsT = util.addr.switch_column_to_row_major(pts)
      local new_height   = self.width
      self.width   = self.height
      self.height  = new_height
      pointsT:resize(3,self.width*self.height)
      x            = pointsT[1]
      y            = pointsT[2]
      z            = pointsT[3]
      xyzrgbTensor[{{},1+offset}] = x
      xyzrgbTensor[{{},2+offset}] = y
      xyzrgbTensor[{{},3+offset}] = z
   end

   local radius2d  = torch.cmul(x:clone(),x:clone()):add(torch.cmul(y:clone(),y:clone())):sqrt()
   local indexGood = torch.lt(radius2d, radius[2]):cmul(torch.gt(radius2d, radius[1]))
   
   if (self.format==1) then
      self.height = h:max()
      self.width  = w:max()
   end
   
   local meanz = 0
   if (self.format == 1) then
     -- adjust points so they are 0 centered
     -- get middle z value
     local rowid   = math.floor((PointCloud.faro_degree_above / (PointCloud.faro_degree_above + PointCloud.faro_degree_below)) * self.height)
     local indexCenter = h:eq(rowid)
     local midrow  = z[indexCenter]
     meanz = midrow[midrow:gt(0)]:mean()
     z[indexGood] = z[indexGood]-meanz
   end
   
   self.points   = torch.cat(torch.cat(x[indexGood],y[indexGood],2),z[indexGood],2)
   self.count    = self.points:size(1)
   
   self.centroid = torch.Tensor({{0, 0, 0}})

   local stdrd = math.sqrt((self.points-self.centroid:repeatTensor(self.count,1)):pow(2):sum(2):mean())
   local perc = 0.75

   print("pass 1: count: "..self.count..", height: "..self.height..", width: "..self.width);
   print("radius: ("..radius[1]..","..radius[2]..") stdrd: "..(stdrd*numstd))

   -- TODO add check if original radius is too small ?
   if (perc*radius[2]) > (stdrd * numstd) then
      --only run this if stdrd significantly smaller
      radius[2] = stdrd * numstd
      print("make second pass with new radius: "..radius[2])
      
      local radius3d  = torch.cmul(x:clone(),x:clone()):add(torch.cmul(y:clone(),y:clone())):add(torch.cmul(z:clone(),z:clone())):sqrt()
      indexGood = torch.lt(radius3d, radius[2]):cmul(torch.gt(radius3d, radius[1]))
      
      if (self.format==1) then
         self.height = h[indexGood]:max()
         self.width  = w[indexGood]:max()
      end
      self.points   = torch.cat(torch.cat(x[indexGood],y[indexGood],2),z[indexGood],2)
      self.count    = self.points:size(1)
   end

   if r and g and b then 
      self.rgb = torch.cat(torch.cat(r[indexGood],g[indexGood],2),b[indexGood],2):byte()
   end

   if(self.format==1) then
      self.hwindices = torch.cat(h[indexGood],w[indexGood],2):short()
   end
   if (po_cloud_file) then 
      fullgrid = self:make_hw_indices() -- full grid
      self.hwindices = torch.cat(fullgrid[{{},1}][indexGood],fullgrid[{{},2}][indexGood],2)
      self.have_hw_data = true
   end
   collectgarbage()
   
   -- drop insignificant digits.
   self.points = self.points:mul(PointCloud.fudge_number):floor():div(PointCloud.fudge_number)

   self:reset_point_stats()

   print("pass 2: count: "..self.count..", height: "..self.height..", width: "..self.width);
end   

function PointCloud:set_pc_od_file(pcfilename)
  local loaded = torch.load(pcfilename)
  self.format = loaded[1]
  self.hwindices = loaded[2]
  self.height = self.hwindices:max(1)[1][1]
  self.width  = self.hwindices:max(1)[1][2]
  self.points = loaded[3]:type('torch.DoubleTensor')
  self.rgb = loaded[4]
  self.count = self.points:size(1)
  self.phi_map = loaded[5]
  if self.phi_map then 
    self.phi_map = self.phi_map:type('torch.DoubleTensor'):div(PointCloud.fudge_number)
  end
  self.theta_map = loaded[6]
  if self.theta_map then 
    self.theta_map = self.theta_map:type('torch.DoubleTensor'):div(PointCloud.fudge_number)
  end
  if(self.normal_mask) then
    self.normal_mask = loaded[7]:type('torch.ByteTensor')
  end
  self.local_to_global_pose = loaded[8]
  self.local_to_global_rotation = loaded[9]
  self:reset_point_stats()
end

function PointCloud:remap(hh,ww)

  local hght_o = self.height
  local wdth_o = self.width
  local hght_n = hh or self.height
  local wdth_n = ww or self.width
  
  local phi_o,theta_o = self:get_xyz_phi_theta()
  local depth_o = self:get_depth_map_no_mask():contiguous()
  local mask_o = self.mask_map:contiguous()
  
  local phi_max = phi_o:max()
  local phi_min = phi_o:min()
  local phi_rng = phi_max-phi_min
  local phi_stp = phi_rng/(hght_n-1)
  local the_max = math.pi
  local the_min = -math.pi
  local the_rng = 2*math.pi
  local the_stp = the_rng/(wdth_n)
  
  local depth_n = torch.zeros(hght_n,wdth_n):contiguous()
  local mask_n = torch.zeros(hght_n,wdth_n):contiguous()
  local phi_n = torch.range(1,hght_n):add(-1):mul(-phi_stp):add(phi_max):repeatTensor(wdth_n,1):t():contiguous()
  local theta_n = torch.range(1,wdth_n):add(-1):mul(-the_stp):add(the_max):repeatTensor(hght_n,1):contiguous()
  
  --[[]]
  libpc.remap_points(torch.data(depth_n), torch.data(mask_n), torch.data(phi_n), torch.data(theta_n),
                     torch.data(depth_o), torch.data(mask_o), torch.data(phi_o), torch.data(theta_o),
                     hght_o, wdth_o, hght_n, wdth_n, phi_max, phi_stp, the_max, the_stp);
  --[[]]
  
  local emask = mask_n:gt(0)
  depth_n[emask] = depth_n[emask]:cdiv(mask_n[emask])
  mask_n = mask_n:eq(0)
  
  local count = emask:double():sum()
  local hwindices = torch.zeros(count,2)
  local indices_h = torch.range(1,hght_n)
  local indices_w = torch.range(1,wdth_n)
  indices_h = indices_h:repeatTensor(wdth_n,1):t():contiguous()
  indices_w = indices_w:repeatTensor(hght_n,1):contiguous()
  hwindices:select(2,1):copy(indices_h[emask]:contiguous())
  hwindices:select(2,2):copy(indices_w[emask]:contiguous())
  hwindices = hwindices:short()
  
  local xyz_map = torch.zeros(3,hght_n,wdth_n)
  xyz_map[3] = phi_n:clone():sin()
  xyz_map[2] = phi_n:clone():cos():cmul(theta_n:clone():sin())
  xyz_map[1] = phi_n:clone():cos():cmul(theta_n:clone():cos())
  xyz_map:cmul(depth_n:repeatTensor(3,1,1))
  xyz_map[mask_n:repeatTensor(3,1,1)] = 0
         
  local points = xyz_map:clone():resize(3,count)
  for chan = 1,3 do 
    points[chan] = xyz_map[chan][emask]
  end
  
  points = points:t():contiguous()
  
  self.count = count
  self.height = hght_n
  self.width = wdth_n
  self.hwindices = hwindices
  self.points = points
  self.rgb = torch.zeros(count,3):byte()
  self.xyz_phi_map = phi_n
  self.xyz_theta_map = theta_n
  
  self:reset_point_stats()
  
  force = true
  
  self:get_index_and_mask(force)
  self:get_points(force)
  self:get_depth(force)
  self:get_xyz_map(force)
  self:get_xyz_map_no_mask(force)
  self:get_rgb_map(force)
  self:get_normal_phi_theta(force)
  self:get_normal_map_varsize(force)
  self:get_xyz_phi_theta(force)
   
end



--[[ FLATTEN FUNCTIONS ]]--

local function adjust_dtheta(dtheta)
  local mid = dtheta:gt(-math.pi):cmul(dtheta:le(math.pi)):type('torch.DoubleTensor')
  local top = dtheta:gt(math.pi):type('torch.DoubleTensor')
  local bot = dtheta:le(-math.pi):type('torch.DoubleTensor')
  mid:cmul(dtheta:clone())
  top:cmul(dtheta:clone():add(-math.pi*2))
  bot:cmul(dtheta:clone():add(math.pi*2))
  dtheta = mid+top+bot
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
  for i = 1,3 do
    conv[i] = torch.conv2(xyz[i]:clone(), kernel, 'F')
  end
  dplane_ll = conv:sub(1,3,3,height+2,3,width+2):clone()
  dplane_ll = dplane_ll:clone():cmul(snormal):sum(1):squeeze():add(sdd:clone():mul(-1))
  mask3 = dplane_ll:gt(thresh):type('torch.DoubleTensor')
  
  kernel = torch.zeros(5,5)
  kernel[3][5] = 1
  
  conv = torch.Tensor(3,height+4,width+4)
  for i = 1,3 do
    conv[i] = torch.conv2(xyz[i]:clone(), kernel, 'F')
  end
  local dplane_rr = conv:sub(1,3,3,height+2,3,width+2):clone()
  dplane_rr = dplane_rr:clone():cmul(snormal):sum(1):squeeze():add(sdd:clone():mul(-1))
  mask4 = dplane_rr:gt(thresh):type('torch.DoubleTensor')
  
  local mask7 = (mask3+mask4):gt(0)
  mask7[mask2]=1
  mask7[mask0]=0
  mask7:select(2,1):fill(0)
  mask7:select(2,width):fill(0)

  local mask8 = mask7:clone():mul(-1):add(1)
  
  mask8[mask1]=0
  
  return mask7,mask8
  
end

local function get_corners(xyz,depth,snormal,stheta,sdd,mask0,theta_thresh,plane_thresh,height,width)
  local tic = log.toc()
  
  local mask_theta = get_theta_corners(theta_thresh, xyz:clone(), depth:clone(), stheta:clone(), height, width)
  local mask_plane,mask_same_plane = get_plane_corners(plane_thresh, xyz:clone(), depth:clone(), snormal:clone(), sdd:clone(), mask0, height, width)
  mask_theta:cmul(mask_same_plane)
  local mask_corner = (mask_theta+mask_plane):gt(0)
  
  print("get_corners: "..log.toc()-tic)
  
  return mask_corner
end

local function get_connections(xyz,depth,snormal,sdd,plane_thresh,height,width)
  local tic = log.toc()
  
  local dpt_thresh_1 = 3*2*math.pi*20*1000/width
  local dpt_thresh_2 = 2*math.pi*15/width
  local kernel
  local conv
  local dplane_l
  local dpoint_l
  
  kernel = torch.zeros(5,5)
  kernel[3][1] = 1
  
  conv = torch.Tensor(3,height+4,width+4)
  for i = 1,3 do
    conv[i] = torch.conv2(xyz[i]:clone(), kernel, 'F')
  end
  dplane_l = conv:sub(1,3,3,height+2,3,width+2):clone()
  dplane_l = dplane_l:clone():cmul(snormal):sum(1):squeeze():add(sdd:clone():mul(-1))
  local mask_con = dplane_l:clone():abs():lt(plane_thresh)
  
  dpoint_l = conv:sub(1,3,3,height+2,3,width+2):clone()
  dpoint_l = dpoint_l:add(xyz:clone():mul(-1)):norm(2,1):squeeze()
  local mask_point_1 = dpoint_l:gt(dpt_thresh_1)
  dpoint_l:cdiv(depth)
  local mask_point_2 = dpoint_l:gt(dpt_thresh_2)
  
  mask_con[mask_point_1] = 0
  mask_con[mask_point_2] = 0
  
  print("get_planes: "..log.toc()-tic)
  
  return mask_con
  
end

function PointCloud:get_connections_and_corners()
  
  local tic = log.toc()
  
  local height = self.height
  local width = self.width
  local index,mask_extant = self:get_index_and_mask()
  
  local xyz = self:get_xyz_map_no_mask()  
  local depth = self:get_depth_map_no_mask()
  
  local noise = self:get_noise()
  local winsize = 10+math.min(10,noise)
  local angdiff = math.min(math.pi/4,math.pi/6 + math.pi*noise/20)
  local thresh_theta = math.pi/4
  local thresh_plane_conn = math.min(0.25*self.meter,0.01*self.meter+3*noise)
  local thresh_plane_corn = math.min(0.50*self.meter,0.10*self.meter + 3*noise)
  local thresh_phi = math.max(3*math.pi/8, math.pi/2-math.pi/8 - noise*math.pi/20)
  local thresh_height = 0.1*height
  
  local snormal,sdd,sphi,stheta,smask = self:get_smooth_normal(winsize,angdiff,angdiff)
  
  local mask_phi = sphi:clone():abs():ge(thresh_phi)
  local mask_height = {{1,thresh_height},{}}
  
  local mask_corner = get_corners(xyz:clone(),depth:clone(),snormal:clone(),stheta:clone(),sdd:clone(),mask_extant,thresh_theta,thresh_plane_corn,height,width)
  local mask_connct = get_connections(xyz:clone(),depth:clone(),snormal:clone(),sdd:clone(),thresh_plane_conn,height,width)
  mask_corner[mask_extant]=0
  mask_corner[mask_phi]=0
  mask_corner[mask_height]=0
  
  mask_connct[mask_extant]=0
  mask_connct[mask_phi]=0
  mask_connct[mask_height]=0
  
  print("get_connections_and_corners: "..log.toc()-tic)
  
  return mask_connct,mask_corner

end

function PointCloud:get_flattened_images(scale,numCorners)

  local tic = log.toc()
  local toc = 0
  
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
  
  local ranges = self.radius:clone():mul(2)
  local minv = self.radius:clone():mul(-1)
  local maxv = self.radius:clone()
  local pix = ranges:clone():div(scale):floor()
  local height = pix[1]
  local width = pix[2]
  local hght = self.height
  local wdth = self.width
  
  local elim_dist = 0.25      
  
  local corners  
  if numCorners and numCorners > 0 then
      corners = torch.zeros(numCorners,2)
  end
  
  local connections,corners_map = self:get_connections_and_corners()
  
  local imagez = torch.zeros(height,width)
  local image_corners = imagez:clone()
  
  local pts = self:get_xyz_map_no_mask():sub(1,2):clone()
  local crds = torch.Tensor(pts:size()):copy(pts)
  local mnv = minv:sub(1,2):repeatTensor(hght,wdth,1):transpose(2,3):transpose(1,2)
  crds:add(mnv:mul(-1)):div(scale):floor()
  pts = self:get_xyz_map_no_mask():clone()
  local corners_map_filled = torch.zeros(hght,wdth)
  
  --cfunc
  
  libpc.flatten_image(torch.data(imagez), torch.data(image_corners),
                                              torch.data(crds:clone():contiguous()), torch.data(pts:clone():contiguous()),
                                              torch.data(connections:clone():contiguous()), 
                                              torch.data(corners_map:clone():contiguous()),
                                              torch.data(corners_map_filled),
                                              hght, wdth, height, width)
  
  local mean_height = (imagez:clone():sum())/(imagez:clone():cdiv(imagez:clone()+PointCloud.very_small_number):sum())
  local stdv_height = math.sqrt((imagez:clone():add(-mean_height):pow(2):sum())/(imagez:clone():cdiv(imagez:clone()+PointCloud.very_small_number):sum()))
  local max_height = mean_height+stdv_height
  local show_thresh = 0.1*max_height
        
  imagez = imagez:clone():gt(max_height):type('torch.DoubleTensor'):mul(max_height):add(
  imagez:clone():le(max_height):type('torch.DoubleTensor'):cmul(imagez)):cmul(
  imagez:clone():gt(show_thresh):type('torch.DoubleTensor'))
  
  mean_height = nil
  stdv_height = nil
  max_height = nil
  shor_thresh = nil
                 
  imagez:div(imagez:max()+PointCloud.very_small_number)
  imagez:cmul(imagez:gt(0.05):type('torch.DoubleTensor'))

  image_corners:div(image_corners:max()+PointCloud.very_small_number):cmul(image_corners:gt(0.05):type('torch.DoubleTensor'))
  
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
  
  --image.displayPoints(imagez:clone():gt(0):type('torch.ByteTensor'):mul(255), corners, colors.MAGENTA, 2)
  --image.display(corners_map_filled)
  imagez = imagez:clone():repeatTensor(3,1,1)
  
  ranges = nil
  minv = nil
  maxv = nil
  pix = nil
  connections = nil
  corners_map = nil
  mask = nil
  pts = nil
  crds = nil
  mnv = nil
  
  toc = log.toc()-tic
  tic = tic + toc
  print('flatten_image_1: finish: '..toc)
  
  collectgarbage()
  
  return imagez,corners,image_corners, scale
end



--[[

local function get_height_slice(zzz, lobound, upbound ,emask,nmask,pmask)

  local lev = zzz:ge(lobound):cmul(zzz:le(upbound)):cmul(emask:eq(0)):cmul(nmask:eq(0)):cmul(pmask)
  --local lev = zzz:ge(lobound):cmul(zzz:le(upbound)):cmul(emask:eq(0)):cmul(nmask:eq(0))
  
  return lev
end

local function squeeze_height_slice(xyz,nmp,dd,smask)
  
  local rmask1 = smask:eq(0)
  
  local xxyy = xyz:sub(1,2):clone()
  local dpth = xyz:sub(1,2):clone():norm(2,1):squeeze()
  local nmxy = nmp:clone():cdiv(nmp:sub(1,2):clone():norm(2,1):repeatTensor(3,1,1))
  nmxy:select(1,3):fill(0)
  local nmdd = dd:clone()
  
  --image.display(smask:double():cmul(dd):add(dd))
  local smsk = torch.zeros(smask:size())
  for w = 1,dd:size(2) do
    local submask = smask:select(2,w)
    if submask:double():sum() > 0 then
      local subdpth = dpth:select(2,w):clone()
      local dmin = subdpth[submask]:min()
      local dmax = subdpth[submask]:max()
      local dmask = subdpth:ge(dmin):cmul(subdpth:le(dmin+10)):cmul(submask)
      submask[dmask:eq(0)] = 0
      smsk:select(2,w):copy(dmask:clone())
    end
  end
  smask = smsk:byte()
  --image.display(smask:double():cmul(dd):add(dd))

  rmask1 = smask:eq(0)
  local rmask2 = rmask1:repeatTensor(2,1,1)
  local rmask3 = rmask1:repeatTensor(3,1,1)
  
  xxyy[rmask2] = 0
  nmxy[rmask3] = 0
  nmdd[rmask1] = 0
  
  local ssum1 = smask:double():sum(1):squeeze()
  local ssum2 = ssum1:repeatTensor(2,1)
  local ssum3 = ssum1:repeatTensor(3,1)
  local smsk1 = ssum1:eq(0)
  local smsk2 = ssum2:eq(0)
  local smsk3 = ssum3:eq(0)
    
  local xysqz = xxyy:sum(2):squeeze():cdiv(ssum2)
  local nmsqz = nmxy:sum(2):squeeze():cdiv(ssum3)
  local ddsqz = nmdd:sum(1):squeeze():cdiv(ssum1)
  
  local thsqz = nmsqz[1]:clone():acos()
  thsqz[nmsqz[2]:lt(0)] = thsqz[nmsqz[2]:lt(0)]:mul(-1)
  
  xysqz[smsk2]=0
  nmsqz[smsk3]=0
  ddsqz[smsk1]=0
  thsqz[smsk1]=0
  
  local smsk = smsk1:eq(0)
  
  xxyy = nil
  nmxy = nil
  bndd = nil
  ssum1 = nil
  ssum2 = nil
  ssum3 = nil
  smsk1 = nil
  smsk2 = nil
  smsk3 = nil
  
  return xysqz,nmsqz,ddsqz,thsqz,smsk,smask
  
end

local function bin_height_slice(xysqz,nmsqz,ddsqz,qmask,width,scale,fht,fwt,cph,cpw)

  local flat_map = torch.zeros(fht,fwt)
  
  local xyu = torch.zeros(2,width)
  local ndu = torch.zeros(width)
  local nmu = torch.zeros(3,width)
  local nbu = torch.zeros(width)
  
  local count = 0
  
  for i=1,width do
    if qmask[i] > 0 then
      local xy = xysqz:select(2,i)
      
      local w = round(xy[1]/scale)+cpw
      local h = round(-xy[2]/scale)+cph
      
      local c = flat_map[h][w]
      
      if c == 0 then
        count = count + 1
        c = count
        --xyu:select(2,c):copy(torch.Tensor({(w-cpw)*scale,-(h-cph)*scale}))
        xyu:select(2,c):copy(xysqz:select(2,i):clone())
        flat_map[h][w] = c
      end
      nbu:sub(c,c):add(1)
      ndu:sub(c,c):add(ddsqz[i])
      nmu:select(2,c):add(nmsqz:select(2,i):clone())      
    elseif count == 0 or nbu[count] ~= 0 then
        count = count + 1
    end
  end
  
  xyu=xyu:sub(1,2,1,count):clone()
  nbu=nbu:sub(1,count):clone()
  ndu=ndu:sub(1,count):clone()
  nmu=nmu:sub(1,3,1,count):clone()
  
  local umask1 = nbu:gt(0)
  local umask3 = umask1:repeatTensor(3,1)
  
  ndu[umask1]=ndu[umask1]:cdiv(nbu[umask1])
  nmu[umask3]=nmu[umask3]:cdiv(nbu[umask1]:repeatTensor(3,1))
  
  local ntu = nmu[1]:clone():acos()
  ntu[nmu[2]:lt(0)] = ntu[nmu[2]:lt(0)]:mul(-1)
  ntu[nbu:le(0)]=0
  
  local umask = umask1
  local fmask = flat_map:gt(0)
  local vmask = flat_map[fmask]:long()
  
  return xyu,nmu,ndu,ntu,umask,fmask,vmask,flat_map
  
end


local function get_theta_slice(col,emask,nmask)
  smask=torch.zeros(emask:size())
  smask:select(2,col):fill(1)
  smask[emask] = 0
  smask[nmask] = 0
  smask:byte()
  return smask
end

local function squeeze_theta_slice(xyz,nphi,nmp,ndd,smask,col)
  
  local xyzz = xyz:sub(1,2):clone()
  xyzz[1] = xyz:sub(1,2):clone():norm(2,1):squeeze()
  xyzz[2] = xyz:sub(3,3):clone():squeeze()
  local xysqz = xyzz:select(3,col):clone()
  local nmsqz = nmp:select(3,col):clone()
  local ddsqz = ndd:select(2,col):clone()
  local phsqz = nphi:select(2,col):clone()
  local qmask = smask:select(2,col):clone()
    
  xyzz = nil
  
  return xysqz,nmsqz,ddsqz,phsqz,qmask
end

local function bin_theta_slice(xysqz,nmsqz,ddsqz,phsqz,qmask,height,scale,fht,fwt,cph,cpw)

  local flat_map = torch.zeros(fht,fwt)
  
  local xyu = torch.zeros(2,height)
  local ndu = torch.zeros(height)
  local nmu = torch.zeros(3,height)
  local npu = torch.zeros(height)
  local nbu = torch.zeros(height)
  
  local count = 0

  for i=1,height do

    if qmask[i] > 0 then
      local xy = xysqz:select(2,i)
      
      local w = round(xy[1]/scale)+cpw
      local h = round(-xy[2]/scale)+cph
      
      local c = flat_map[h][w]
      
      if c == 0 then
        count = count + 1
        c = count
        xyu:select(2,c):copy(xysqz:select(2,i):clone())
        flat_map[h][w] = c
      end
      nbu:sub(c,c):add(1)
      ndu:sub(c,c):add(ddsqz[i])
      nmu:select(2,c):add(nmsqz:select(2,i):clone())  
      npu:sub(c,c):add(phsqz[i])    
    elseif count == 0 or nbu[count] ~= 0 then
      count = count + 1
    end
  end
  
  xyu=xyu:sub(1,2,1,count):clone()
  nbu=nbu:sub(1,count):clone()
  ndu=ndu:sub(1,count):clone()
  nmu=nmu:sub(1,3,1,count):clone()
  npu=npu:sub(1,count):clone()
  
  local umask1 = nbu:gt(0)
  local umask3 = umask1:repeatTensor(3,1)
  
  ndu[umask1]=ndu[umask1]:cdiv(nbu[umask1])
  nmu[umask3]=nmu[umask3]:cdiv(nbu[umask1]:repeatTensor(3,1))
  npu[umask1]=npu[umask1]:cdiv(nbu[umask1])
  
  local umask = umask1
  local fmask = flat_map:gt(0)
  local vmask = flat_map[fmask]:long()
  
  return xyu,nmu,ndu,npu,umask,fmask,vmask,flat_map
end
--[[]]

--[[
local function flatten_slice(nmu,ndu,ntu,fmask,vmask,fht,fwt)
  
  local flat_nmp = torch.zeros(3,fht,fwt)
  local flat_nmd = torch.zeros(fht,fwt)
  local flat_nth = torch.zeros(fht,fwt)
  
  for k = 1,3 do
    flat_nmp[k][fmask] = nmu[k][vmask]+1
  end
  flat_nmd[fmask] = ndu[vmask]
  flat_nth[fmask] = ntu[vmask]+2*math.pi
  
  return flat_nmp, flat_nmd, flat_nth
end

local function isColinear(pts,i,radius,noisethresh)

  local dim = pts:size(1)
  local len = pts:size(2)
  local pts_t = pts:contiguous()
  local cent = pts_t:sub(1,dim,i,i):expand(pts_t:size(1),1)
  local pts_c = (pts_t-cent:contiguous():expand(pts_t:size()):contiguous()):contiguous()
  local nrm_c = pts_c:clone():norm(2,1):squeeze()
  local ind_z = pts_t:clone():norm(2,1):eq(0)
  if ind_z:double():sum() > 0 then
    return false
  end
  local ind_s = nrm_c:le(radius)
  --ind_s[math.max(1,i-1)] = 1
  --ind_s[math.min(len,i+1)] = 1
  local totind = ind_s:double():sum()
  local totlft = ind_s:sub(1,i):sum()
  local totrgt = ind_s:sub(i,len):sum()
  
  if i == 1 or i == len then
    if totind < dim*2 then
      return false
    end
  elseif totind < dim*2 or totlft < dim+1 or totrgt < dim+1 then
    return false
  end
  
  -- look only at local points
  local pts_u = torch.Tensor(dim,ind_s:double():sum())
  for k=1,dim do
    pts_u[k] = pts_c[k][ind_s]
  end

  pts_u = pts_u:contiguous()
  local s,v,d = torch.svd(pts_u)
  local abc = s:sub(1,dim,dim,dim):contiguous()
  local maxnoise = (abc:expand(pts_u:size()):contiguous():cmul(pts_u):sum(1)):abs():max()
  
  if maxnoise > noisethresh then
    --print(maxnoise)
    return false
  else
    return true
  end
end

local function isCorner(xyu,nmu,umask,i,radius,thresh,step)
  
  local len = umask:size(1)
  if i == 1 or i == len then
    return false
  else
    local pts 
    local lft
    local rgt
    local cnt
    
    lft = math.max(1,i-step)
    rgt = math.min(len,i+step)
    cnt = i-lft+1
    pts = xyu:contiguous():sub(1,2,lft,rgt):clone()
    local colinear_c = isColinear(pts,cnt,radius,thresh)
    
    lft = math.max(1,i-step)
    rgt = math.min(len,i)
    cnt = i-lft+1
    pts = xyu:contiguous():sub(1,2,lft,rgt):clone()
    local colinear_l = isColinear(pts,cnt,radius,thresh)
    
    lft = math.max(1,i)
    rgt = math.min(len,i+step)
    cnt = i-lft+1
    pts = xyu:contiguous():sub(1,2,lft,rgt):clone()
    local colinear_r = isColinear(pts,cnt,radius,thresh)
    
    if colinear_c then
      return false
    else
      local dptc = xyu:sub(1,2,i,i):clone():norm()
      local dptl = xyu:sub(1,2,i-1,i-1):clone():norm()
      local dptr = xyu:sub(1,2,i+1,i+1):clone():norm()
        
      if colinear_l and colinear_r then
        return true
      elseif colinear_l then
        if dptc > 0 and (dptc <= dptr or dptr == 0) then
          return true
        else
          return false
        end
      elseif colinear_r then
        if dptc > 0 and (dptc <= dptl or dptl == 0) then
          return true
        else
          return false
        end
      else
        return false
      end
    end
  end
end

local function validateCorner(xysq,nmsq,thsq,qmask,smask,radius,thresh,step,maxdepth,height,width,scale,fht,fwt,cph,cpw,dim)

  local ndpt = xysq:clone():norm(2,1):squeeze()
  ndpt[qmask:eq(0)] = math.huge
        
  local xy1 = xysq[1]
  
  -- check validity of candidate corner
  local corners = torch.zeros(height,width)
  local flat_corners = torch.zeros(fht,fwt)
  local dthe = torch.zeros(qmask:size())
  local www = qmask:size(1)
  local onC = false
  local beg = 0
  local enn = 0
  local xyc
  local ww
  local hh
  local cnt = 0
  for i = 1+1,www-1 do
    local u = qmask[i]
    dthe[i] = math.abs(thsq[((i) % www) + 1]-thsq[((i-2+www) % www)+1])
    local rad = math.max(2*step * ndpt[i]/www,radius)
    
    local abv = 1
    local blw = 1
    if dim == 1 then
      abv = qmask:sub(1,i-1):double():sum()
      blw = qmask:sub(i+1,www):double():sum()
    end
      
    if u > 0 and isCorner(xysq,nmsq,qmask,i,rad,thresh,step) and 
       (dim == 2 or xy1[i] > 10) and abv > 0 and blw > 0 then
    
      local p = i-1
      local n = i+1
      if onC then
        enn = i
      else
        cnt=cnt+1
        onC = true
        beg = i
        enn = i
      end
              
    elseif onC then
      onC = false
              
      if beg == enn then

        xyc = xysq:select(2,beg):clone()
        ww = round(xyc[1]/scale)+cpw
        hh = round(-xyc[2]/scale)+cph
                  
        flat_corners:sub(math.max(1,hh-1),math.min(fht,hh+1),math.max(1,ww-1),math.min(fwt,ww+1)):fill(1)
        corners:select(dim,beg):fill(1)
                
      else
                
        local maxd,maxi = (xysq:sub(1,2,beg,enn-1):clone()-xysq:sub(1,2,beg+1,enn):clone()):norm(2,1):max(2)
        local mind,mini = (xysq:sub(1,2,beg,enn-1):clone()-xysq:sub(1,2,beg+1,enn):clone()):norm(2,1):min(2)
                
        maxd = maxd:squeeze()
        mind = mind:squeeze()
        maxi = maxi:squeeze()+beg
        mini = mini:squeeze()+beg
                
        if maxd - mind <= 2.5*thresh then
                    
          local maxd,maxi = dthe:sub(beg,enn):clone():max(1)
          maxd = maxd:squeeze()
          maxi = maxi:squeeze()+beg-1
                  
          xyc = xysq:select(2,maxi):clone()
          ww = round(xyc[1]/scale)+cpw
          hh = round(-xyc[2]/scale)+cph
          flat_corners:sub(math.max(1,hh-1),math.min(fht,hh+1),math.max(1,ww-1),math.min(fwt,ww+1)):fill(1)
          corners:select(dim,maxi):fill(1)
                
        else
                
          for j=beg,enn do
                  
            local dpc = ndpt[j]
            local dpl = dpc
            local dpr = dpc
            if j > beg then
              dpl = ndpt[j-1]
            end
            if j < enn then
              dpr = ndpt[j+1]
            end
                    
            if dpc <= dpl and dpc <= dpr and dpc <= maxdepth then
                    
              xyc = xysq:select(2,j):clone()
              ww = round(xyc[1]/scale)+cpw
              hh = round(-xyc[2]/scale)+cph
              
              flat_corners:sub(math.max(1,hh-1),math.min(fht,hh+1),math.max(1,ww-1),math.min(fwt,ww+1)):fill(1)
              corners:select(dim,j):fill(1)
                  
            end
          
          end
          
        end   
      end
    end
  end
     
  corners[smask:eq(0)]=0
  flat_corners = flat_corners:byte()
  corners = corners:byte()
  ndpt = nil
  collectgarbage()
  
  return flat_corners,corners
end


function PointCloud:get_slice_corners_th(h,scaleh,scalew)
  
  local scale = scalew or 10
  scaleh = scaleh or 25
  
  local height = self.height
  local width = self.width
  local lobound = h-scaleh/2
  local upbound = h+scaleh/2
  local phthrsh = math.pi/6
  local rx = math.ceil(self.radius[1]/scale)
  local ry = math.ceil(self.radius[2]/scale)
  local fht = ry*2+1
  local fwt = rx*2+1
  local cph = ry+1
  local cpw = rx+1
  
  local index,emask = self:get_index_and_mask()
  local xyz = self:get_xyz_map_no_mask()
  local nmp,dd,phi,theta,nmask = self:get_normal_map_varsize()
  --nmp,dd,phi,theta,nmask = self:get_smooth_normal(nil, nil, nil, phi, theta, nmask)
  
  local pmask = phi:clone():abs():lt(phthrsh)
  
  local zzz = xyz[3]:clone()
  local smask = get_height_slice(zzz, lobound, upbound ,emask,nmask,pmask)
  
  if smask:double():sum() > 2 then
  
    local xysqz,nmsqz,ddsqz,thsqz,qmask,smask = squeeze_height_slice(xyz,nmp,dd,smask)
    
    if qmask:double():sum() > 2 then
      local xyu,nmu,ndu,ntu,umask,fmask,vmask,flat_map = bin_height_slice(xysqz,nmsqz,ddsqz,qmask,width,scale,fht,fwt,cph,cpw)
      
      if fmask:double():sum() > 2 then
        
        local flat_nmp, flat_nmd, flat_nth = flatten_slice(nmu,ndu,ntu,fmask,vmask,fht,fwt)

        local rad = 100
        local stp = 10        
        local pd = 15
        local md = 15*self.meter

        
        local flat_corners,corners = validateCorner(xysqz,nmsqz,thsqz,qmask,smask,rad,pd,stp,md,height,width,scale,fht,fwt,cph,cpw,2)
        flat_nmp[flat_corners:repeatTensor(3,1,1)]=2
        --image.display(flat_nmp:clone():div(2))
        --image.display(corners)
        return flat_nmp,corners
        
      end
      
    end

  else
    xyz = nil
    nmp = nil
    dd = nil
    phi = nil
    theta = nil
    nmask = nil
  end
  
end

function PointCloud:get_slice_corners_ph(col,scale )
  
  if col < 1 or col > self.width then
    return nil,nil
  end
  
  local scale = scale or 10
  
  local height = self.height
  local width = self.width

  local rxy = math.ceil(math.sqrt(math.pow(self.radius[1],2)+math.pow(self.radius[2],2))/scale)
  local rzz = math.ceil(self.radius[3]/scale)
  
  local fht = rzz*2+1
  local fwt = rxy*1+1
  
  local cph = rzz+1
  local cpw = 1
  
  local index,emask = self:get_index_and_mask()
  local xyz = self:get_xyz_map_no_mask()
  local nmp,ndd,nphi,ntheta,nmask = self:get_normal_map_varsize()
  --nmp,ndd,nphi,ntheta,nmask = self:get_smooth_normal(nil, nil, nil, nphi, ntheta, nmask)
  
  local phi,theta = self:get_xyz_phi_theta()
  
  local smask = get_theta_slice(col,emask,nmask)
  
  
  if smask:double():sum() > 2 then
    local xysqz,nmsqz,ddsqz,phsqz,qmask = squeeze_theta_slice(xyz,nphi,nmp,ndd,smask,col)
    
    if qmask:double():sum() > 2 then
      local xyu,nmu,ndu,npu,umask,fmask,vmask,flat_map = bin_theta_slice(xysqz,nmsqz,ddsqz,phsqz,qmask,height,scale,fht,fwt,cph,cpw)
      
      if fmask:double():sum() > 2 then
        
        local flat_nmp, flat_nmd, flat_nph = flatten_slice(nmu,ndu,npu,fmask,vmask,fht,fwt)
        
        local rad = 100
        local stp = 15        
        local pd = 15
        local md = 15*self.meter

        local flat_corners,corners = validateCorner(xysqz,nmsqz,phsqz,qmask,smask,rad,pd,stp,md,height,width,scale,fht,fwt,cph,cpw,1)
        flat_nmp[flat_corners:repeatTensor(3,1,1)]=2
        return flat_nmp,corners
        
      end
      
    end
  end
end



function PointCloud:get_corners_th(scaleh,scalew)
  scaleh = scaleh or 10
  scalew = scalew or 10
  local stp = scaleh/2
  local maxz = self.maxval[3]
  local minz = self.minval[3]
  local rngz = math.ceil((maxz-minz)/stp+1)
  local i = 0
  
  print(rngz,scaleh,scalew)
  
  local corners_rmp = torch.zeros(self.height,self.width)
  
  for i=1,rngz do
    local h = (i-1)*stp+minz
    local j = rngz-i+1
    if i % 50 == 0 then
      print(i,h)
    end
    local flat_nmp,corners = self:get_slice_corners_th(h,scaleh,scalew)
    if corners then
      corners_rmp[corners] = 1
    end
    collectgarbage()
  end
  
  return corners_rmp
end


function PointCloud:get_corners_ph(scale)

  local toc = log.toc()
  scale = scale or 10
  
  local corners_rmp = torch.zeros(self.height,self.width)
  
  for i=1,self.width do
    if i % 50 == 0 then
      print(i)
    end
    local flat_nmp,corners = self:get_slice_corners_ph(i,scale)
    local corners = torch.ones(self.height,self.width):byte()
    if corners then
      corners_rmp[corners] = 1
    end
    collectgarbage()
  end
  print(log.toc()-toc)
  
  return corners_rmp
end

function PointCloud:test2(scaleh,scalew, scaleth)
  
  local c1 = self:get_corners_th(scaleh,scale)
  local c2 = self:get_corners_ph(scaleth,scale)
  
  for j = 1,self.width do
    local cc = c1:select(2,j)
    for i = 1,self.height do
      if cc[i] > 0 and cc:sub(math.max(1,i-2),i):sum() < 3 and cc:sub(i,math.min(self.height,i+2)):sum() < 3 then
        cc:sub(i,i):fill(0)
      end
    end
  end
  
  for i = 1,self.height do
    for j = 1,self.width do
      local cc = c2[i][j]
      if cc > 0 then
        local ccl = c2:sub(math.max(1,i-2),math.min(self.height,i+2),math.max(1,j-2),j):clone():sum(1):gt(0):sum()
        local ccr = c2:sub(math.max(1,i-2),math.min(self.height,i+2),j,math.min(self.width,j+2)):clone():sum(1):gt(0):sum()
        
        if ccl <3 and ccr <3 then
          c2:sub(i,i,j,j):fill(0)
        end
        
      end
    end
  end
  
  corners_rmp = (c1+c2):gt(0)
  
  return corners_rmp,c1,c2
end
--[[]]

--[[
function PointCloud:project_at_theta(theta)

  local points = self.points:t():contiguous()
  local rot = torch.Tensor({{ math.cos(theta), math.sin(theta), 0},
                            {-math.sin(theta), math.cos(theta), 0},
                            {               0,               0, 1}})
                            
  points = rot * points
  local ptsyz = points:sub(2,3):clone()
  local depth = points:select(1,1):clone():abs()
  local side = points:select(1,1):ge(0)
  
  local miny = ptsyz[1]:min()
  local maxy = ptsyz[1]:max()
  local minz = ptsyz[2]:min()
  local maxz = ptsyz[2]:max()
  
  local scale = 10
  local thrsh = 15
  local h = math.ceil((maxz-minz)/scale)+1
  local w = math.ceil((maxy-miny)/scale)+1
  
  local dptpos = torch.zeros(h,w)
  local dptneg = torch.zeros(h,w)
  local binpos = torch.zeros(h,w)
  local binneg = torch.zeros(h,w)
  
  local crdh = ptsyz[2]:clone():mul(-1):add(maxz):div(scale):ceil():add(1)
  local crdw = ptsyz[1]:clone():add(-miny):div(scale):ceil():add(1)
  crdw[side] = ptsyz[1][side]:clone():mul(-1):add(maxy):div(scale):ceil():add(1)
  
  for i = 1,self.count do
    local chh = crdh[i]
    local cww = crdw[i]
    local dp = depth[i]
    local img = dptpos
    local bin = binpos
    
    if side[i] == 0 then
      img = dptneg
      bin = binneg
    end
    
    for ch = math.max(1,chh-1),chh do
      for cw = math.max(1,cww-1),cww do
        local b = bin[ch][cw]
        if b > 0 then
          local dc = img[ch][cw]
          if dp <= dc + thrsh then
            if dp < dc - thrsh then
              bin[ch][cw] = 1
              img[ch][cw] = dp
            else
              img[ch][cw] = (dc*b+dp)/(b+1)
              bin[ch][cw] = b+1
            end
          end
        else
          bin[ch][cw] = 1
          img[ch][cw] = dp
        end
      end
    end
    
  end
  
  return dptpos,dptneg
end

function PointCloud:find_main_thetas(binnum)
  local nmp,dd,phi,theta,mask = self:get_normal_map_varsize()
  local theta_list = theta[mask:eq(0)]:clone()
  theta_list[theta_list:lt(0)] = theta_list[theta_list:lt(0)]+math.pi
  local theta_val = torch.range(1,binnum)*math.pi/binnum - math.pi/(2*binnum)
  local theta_hst = torch.histc(theta_list,binnum)
  local hst_val,hst_ind = theta_hst:sort(1,true)
  theta_val=theta_val[hst_ind]
  return hst_val,theta_val
end

function PointCloud:test()
  local binnum = 8
  local hst,the = self:find_main_thetas(binnum)
  local the1 = the[1]
  local the2 = the[1]-math.pi/2
  local img1,img2 = self:project_at_theta(the1)
  local img3,img4 = self:project_at_theta(the2)
  return img1,img2,img3,img4,the,the1,the2
end

--[[]]



--[[ POSE AND TRANSFORMATION FUNCTIONS ]]

function PointCloud:get_global_scan_center()
   local pose = self:get_local_to_global_pose()
   return pose
end

-- add this vector to all points to get points in global coordinates
function PointCloud:set_local_to_global_pose(pose)
   self.local_to_global_pose = pose
end

-- add this vector to all points to get points in global coordinates
function PointCloud:get_local_to_global_pose()
   if not self.local_to_global_pose then 
      self.local_to_global_pose = torch.zeros(3)
   end
   return self.local_to_global_pose
end

function PointCloud:set_local_to_global_rotation(quaternion)
   self.local_to_global_rotation = quaternion
end

-- rotate all points by this quaternion to place them in global coordinates
function PointCloud:get_local_to_global_rotation()
   if not self.local_to_global_rotation then
      self.local_to_global_rotation = torch.Tensor({0,0,0,1})
   end
   return self.local_to_global_rotation 
end

function PointCloud:set_pose_from_rotation_matrix(mat)
   self:set_local_to_global_rotation(geom.quaternion.from_rotation_matrix(mat))
   self:set_local_to_global_pose(mat[{{1,3},4}]:clone())
end

function PointCloud:get_global_points()
   local pose = self:get_local_to_global_pose()
   local rot  = self:get_local_to_global_rotation()
   return rotate_translate(rot,pose,self.points)
end

function PointCloud:get_global_normal_map(recompute)
  local norm = self:get_normal_map(recompute)
  local pose = self:get_local_to_global_pose()
  local rot  = self:get_local_to_global_rotation()
  return rotate_translate(rot, pose,norm:reshape(3,norm:size(2)*norm:size(3)):t():clone()):t():reshape(3,norm:size(2),norm:size(3))
end

function PointCloud:estimate_global_faro_pose(degree_above, degree_below)
  local poseFaro = self:estimate_faro_pose(degree_above, degree_below)
  local pose = self:get_local_to_global_pose()
  local rot  = self:get_local_to_global_rotation()
  return rotate_translate(rot,pose,poseFaro:reshape(1,3))
end

function PointCloud:estimate_faro_pose(degree_above, degree_below)
   degree_above = degree_above or 90
   degree_below = degree_below or 60
   -- get middle z value
   local rowid   = math.floor((degree_above / (degree_above + degree_below)) * self.height)
   local xyz_map = self:get_xyz_map()
   local midrow  = xyz_map[{3,rowid,{}}]
   return torch.Tensor({0,0,midrow[midrow:gt(0)]:mean()})
end



--[[ SAVE FUNCTIONS ]]--

function PointCloud:downsample_points(leafsize)

   local tic = log.toc()
   local toc = 0
   
   if not leafsize then
     leafsize = self.meter/100
   elseif leafsize < self.meter/1000 then
     leafsize = self.meter/1000
   end
   
   local scale = leafsize
   local ranges = self.maxval:clone():squeeze():add(self.minval:clone():squeeze():mul(-1))
   local pts = self:get_xyz_map_no_mask():clone()
   local crd = pts:clone():add(self.minval:clone():mul(-1):squeeze():repeatTensor(self.width,self.height,1):transpose(1,3)):div(scale):floor():add(1)
   local rgb = self:get_rgb_map_no_mask():type('torch.DoubleTensor')
   pts = crd:clone():add(-1):mul(scale):add(self.minval:squeeze():repeatTensor(self.height,self.width,1):transpose(1,3))
   
   local points = torch.zeros(self.count,3)
   local dsrgb = torch.zeros(self.count,3)
   local count = torch.zeros(1):type('torch.IntTensor')
   
   local downsampled_points = torch.data(points)
   local downsampled_rgb = torch.data(dsrgb)
   local downsampled_count = torch.data(count)
   local coord_map = torch.data(crd:clone():contiguous())
   local points_map = torch.data(pts:clone():contiguous())
   local rgb_map = torch.data(rgb:clone():contiguous())

   toc = log.toc()-tic
   tic = tic + toc 
   print('downsample: setup: '..toc)
   
   libpc.downsample(downsampled_points, downsampled_rgb, downsampled_count, 
                                  coord_map, points_map, rgb_map, self.height, self.width)
   
   toc = log.toc()-tic
   tic = tic + toc                                
   print('downsampled: loop '..toc)
   
   count = count:clone()[1]
   if count == 0 then
     count = 1
   end
   points = points:clone():sub(1,count)
   dsrgb = dsrgb:clone():sub(1,count):type('torch.ByteTensor')
   
   count = nil
   scale = nil
   ranges = nil
   nmp = nil
   pts = nil
   crd = nil
   rgb = nil
   downsampled_points = nil
   downsampled_rgb = nil
   downsampled_count = nil
   coord_map = nil
   points_map = nil
   rgb_map = nil
   
   toc = log.toc()-tic
   tic = tic + toc 
   print('downsampled_0: finish: '..toc)
   
   collectgarbage()
   return points, dsrgb;
   
end

function PointCloud:save_to_od(filename)
  if util.fs.extname(filename)==PointCloud.PC_OD_EXTENSION then
    local pts = self.points:clone():type('torch.IntTensor')
    local phi_map
    local theta_map
    if self.phi_map then 
      phi_map = self.phi_map:clone():mul(PointCloud.fudge_number):floor():clone():type('torch.IntTensor')
    end
    if self.theta_map then 
      theta_map = self.theta_map:clone():mul(PointCloud.fudge_number):clone():type('torch.IntTensor')
    end
    torch.save(filename, {self.format, self.hwindices, pts, self:get_rgb(), phi_map, theta_map, self.normal_mask, self.local_to_global_pose, self.local_to_global_rotation})
  end
end

local function saveHelper_xyz(points, rgb, fname)
   local file = io.open(fname, 'w')
   local tmpt = torch.range(1,points:size(1))                                                                                                                                
   tmpt:apply(function(i) 
      pt = points[i]
      if(rgb) then 
        rgbx = rgb[i]
        file:write(''..pt[1]..' '..pt[2]..' '..pt[3]..' '..rgbx[1]..' '..rgbx[2]..' '..rgbx[3]..'\n')
      else
        file:write(''..pt[1]..' '..pt[2]..' '..pt[3]..'\n')        
      end
   end)
   file:close()
end

function PointCloud:save_points_to_xyz(fname)
   local points = self.points:clone()
   if self.format == 1 then
     points = points/self.meter
   end  
   saveHelper_xyz(points, self:get_rgb(), fname)
end

function PointCloud:save_global_points_to_xyz(fname)
   local points = self:get_global_points()
   if self.format == 1 then
     points = points/self.meter
   end  
   saveHelper_xyz(points, self:get_rgb(), fname)
end

function PointCloud:save_any_points_to_xyz(fname, points,rgb)
   if self.format == 1 then
     points = points/self.meter
   end
   saveHelper_xyz(points, rgb, fname)
end

function PointCloud:save_downsampled_to_xyz(leafsize, fname)
   local downsampled_points, downsampled_rgb = self:downsample_points(leafsize)
   if self.format == 1 then
     downsampled_points = downsampled_points/self.meter
   end  
   saveHelper_xyz(downsampled_points, downsampled_rgb, fname)
end

function PointCloud:save_downsampled_global_to_xyz(leafsize, fname)
   local downsampled_points, downsampled_rgb = self:downsample_points(leafsize)
   local pose = self:get_local_to_global_pose()
   local rot  = self:get_local_to_global_rotation()
   downsampled_points = rotate_translate(rot,pose,downsampled_points)
   if self.format == 1 then
     downsampled_points = downsampled_points/self.meter
   end 
   saveHelper_xyz(downsampled_points, downsampled_rgb, fname)
end
