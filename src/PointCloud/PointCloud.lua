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
                                                 
    int downsample_with_widthheightinfo(double* downsampled_points, double* downsampled_rgb, 
                             int* downsampled_count, double* coord_map, 
                             double* points_map, double* rgb_map, 
                             int height, int width);
    
    int make_normal_list(double* normal_list, double* normal_map,
                     int* hwindices, int length, int height, int width);
    
    int connect_lines_in_image(double* img, double y1, double x1, double y2, double x2, 
                           int height, int width, double incr);
    
    int flatten_image_with_widthheightinfo(double* imagez, double* image_corner,
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

function PointCloud:get_xyz_map()
   if not self.xyz_map then
      local points      = self:get_points()
      local index, mask = self:get_index_and_mask()

      self.xyz_map = util.addr.remap(points,index,mask)
   end
   return self.xyz_map
end

function PointCloud:get_xyz_map_no_mask()
    if not self.xyz_map_no_mask then
        local points      = self:get_points()
        local index,mask = self:get_index_and_mask()
        self.xyz_map_no_mask = util.addr.remap(points,index,mask,torch.Tensor(3,self.height,self.width))
        local dpth = self.xyz_map_no_mask:clone():pow(2):sum(1):squeeze()
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
function PointCloud:get_depth()
   if not self.depth then 
      points     = self.points
      depth = torch.sqrt(points:clone():norm(2,2)):squeeze()
   end
   return depth
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
  
    libpc.phi_map(torch.data(self.phi_map),torch.data(point_map:clone():contiguous()),height,width)
    libpc.theta_map(torch.data(self.theta_map),torch.data(point_map:clone():contiguous()),height,width)
  
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
    
    return nmp,dd,mask
end

function PointCloud:get_smooth_normal(max_win, phi_diff, theta_diff)

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
  
    local phi,theta,mask = self:get_normal_phi_theta()
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
  
    return smooth_nmp, smooth_phi, smooth_theta, dd, mask

end

function PointCloud:get_normal_list()

  local tic = log.toc()
  local toc = 0
  
  local nmp = self:get_normal_map()
  local nmlist = torch.zeros(self.count,3)
  
  local normal_list = torch.data(nmlist)
  local normal_map = torch.data(nmp:clone():contiguous())
  local hwindices = torch.data(self.hwindices:clone():type('torch.IntTensor'):contiguous())
 
  libpc.make_normal_list(normal_list, normal_map, 
                     hwindices, self.count, self.height, self.width)

  normal_list = nil
  normal_map = nil
  hwindices = nil
  
  toc = log.toc()-tic
  tic = tic + toc 
  print('get_normal_list: all: '..toc)
  
  return nmlist

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
  self.normal_mask = loaded[7]:type('torch.ByteTensor')
  self.local_to_global_pose = loaded[8]
  self.local_to_global_rotation = loaded[9]
  self:reset_point_stats()
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
    
function PointCloud:get_connections_and_corners()

  local noise = self:get_xyz_map_no_mask():select(1,3):select(1,1):std()
  local winsize = 10+noise
  local angdiff = math.pi/6 + math.pi*noise/20
  local thresh_theta_changed = math.pi/4
  local thresh_plane_conn = 0.01*self.meter+3*noise
  local thresh_plane_corn = 0.10*self.meter + 3*noise
  local thresh_phi = math.pi/2-math.pi/8 - noise*math.pi/20
  
  local height = self.height
  local width = self.width
  --local phi,theta,mask = self:get_normal_phi_theta()
  local snormal,sphi,stheta,sdd = self:get_smooth_normal(winsize,angdiff,angdiff)
  local mask
  local depth_shift
  local xyz = self:get_xyz_map_no_mask()
  local kernel_left3 = torch.Tensor({{0,0,0},{-1,1,0},{0,0,0}})
  local kernel_comp = torch.Tensor({{0,0,0},{-1,2,-1},{0,0,0}})
  local kernel_left5 = torch.zeros(5,5)
  kernel_left5[3][1] = 1
  
  local conv
  
  local dtheta
  local dplane
  
  -- find the change of theta_normal from curr pixel to previous pixel
  conv = torch.conv2(stheta:clone(), kernel_left3, 'F')
  dtheta = conv:sub(2,height+1,2,width+1):clone()
  dtheta = adjust_dtheta(dtheta):abs()
  conv = torch.conv2(dtheta:clone(), kernel_comp, 'F')
  dtheta = conv:sub(2,height+1,2,width+1):clone()
  
  -- find the points that do not line on the plane 2-adjacent-left
  
  conv = torch.Tensor(3,height+4,width+4)
  for i = 1,3 do
    conv[i] = torch.conv2(xyz[i]:clone(), kernel_left5, 'F')
  end
  dplane = conv:sub(1,3,3,height+2,3,width+2):clone()
  dplane = dplane:clone():cmul(snormal):sum(1):squeeze():add(sdd:clone():mul(-1))
  
  conv = torch.Tensor(3,height+2,width+2)
  for i = 1,3 do
    conv[i] = torch.conv2(xyz[i]:clone(), kernel_left3, 'F')
  end
  depth_shift = conv:sub(1,3,2,height+1,2,width+1):clone():norm(2,1):squeeze()
  mask = (depth_shift:eq(0)-self.mask_map):gt(0)
  
  dplane[mask] = 15*self.meter
  
  local connection = dplane:clone():abs():lt(thresh_plane_conn)
  local corners = dplane:clone():abs():gt(thresh_plane_corn):add(dtheta:clone():gt(thresh_theta_changed)):gt(0)
  
  local mask_phi = sphi:clone():abs():ge(thresh_phi)

  
  connection[self.mask_map] = 0
  connection[mask_phi] = 0
  connection[{{1,0.1*height},{}}] = 0
  corners[self.mask_map] = 0
  corners[mask_phi] = 0
  corners[{{1,0.1*height},{}}] = 0
  
  return connection,corners
  
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
  
  libpc.flatten_image_with_widthheightinfo(torch.data(imagez), torch.data(image_corners),
                                              torch.data(crds:clone():contiguous()), torch.data(pts:clone():contiguous()),
                                              torch.data(connections:clone():contiguous()), 
                                              torch.data(corners_map:clone():contiguous()),
                                              torch.data(corners_map_filled),
                                              hght, wdth, height, width)
  
  local mean_height = (imagez:clone():sum())/(imagez:clone():cdiv(imagez:clone()+PointCloud.very_small_number):sum())
  local stdv_height = math.sqrt((imagez:clone():add(-mean_height):pow(2):sum())/(imagez:clone():cdiv(imagez:clone()+PointCloud.very_small_number):sum()))
  local max_height = mean_height+stdv_height
  local show_thresh = 0.01*max_height
        
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
  
  image.displayPoints(imagez:clone():gt(0):type('torch.ByteTensor'):mul(255), corners, colors.MAGENTA, 2)
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
  
  return imagez,corners,image_corners
end



--[[
function PointCloud:get_connections_and_corners()
    local height = self.height
    local width = self.width
    local normal = self:get_normal_map()
    local phi,theta = self:get_normal_phi_theta()
    local snormal,sphi,stheta = self:get_smooth_normal()
    local xyz = self:get_xyz_map_no_mask()
    
    local theta_thresh = math.pi/2
    local plane_thresh_conn = 0.01 * self.meter
    local plane_thresh_corn = 0.25 * self.meter
    local z_thresh = 0.75
    
    local kl = torch.Tensor({{0,0,0},{-1,1,0},{0,0,0}})
    local kr = torch.Tensor({{0,0,0},{0,1,-1},{0,0,0}})
    
    local C
    
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
    
    C = torch.conv2(stheta:clone(),kl,'F')
    local dtheta_l = C:sub(2,height+1,2,width+1):clone()
    dtheta_l = adjust_dtheta(dtheta_l)
    
    C = torch.conv2(stheta:clone(),kr,'F')
    local dtheta_r = C:sub(2,height+1,2,width+1):clone()
    dtheta_r = adjust_dtheta(dtheta_r)
    
    local d2theta = (dtheta_l+dtheta_r):clone()
    
    local smooth_d = snormal:clone():cmul(xyz):norm(2,1):squeeze():mul(-1)
    
    kl = torch.Tensor({{0,0,0},{1,0,0},{0,0,0}})
    kr = torch.Tensor({{0,0,0},{0,0,1},{0,0,0}})
    
    C = torch.Tensor(3,height+2,width+2)
    
    for i = 1,3 do
      C[i] = torch.conv2(xyz[i],kl,'F')
    end
    local xyz_l = C:sub(1,3,2,height+1,2,width+1):clone()
    local dd_l = xyz_l:clone():cmul(snormal):sum(1):squeeze():add(smooth_d)
    
    for i = 1,3 do
      C[i] = torch.conv2(xyz[i],kr,'F')
    end
    local xyz_r = C:sub(1,3,2,height+1,2,width+1):clone()
    local dd_r = xyz_r:clone():cmul(snormal):sum(1):squeeze():add(smooth_d)
    
    local d2d = (dd_l:clone():abs() + dd_r:clone():abs())
    
    local kk = torch.Tensor({{-1,-1,-1},{-1,8,-1},{-1,-1,-1}})
    C = torch.conv2(d2d:clone(),kk,'F')
    local d2d = C:sub(2,height+1,2,width+1):clone():abs()
    
    local conn = d2d:lt(plane_thresh_conn)
    local corn = d2d:gt(plane_thresh_corn):add(d2theta:gt(theta_thresh)):gt(0)
    
    conn[self.mask_map] = 0
    corn[self.mask_map] = 0
    
    z_filt = snormal[3]:clone():abs():gt(z_thresh)
    
    conn[z_filt] = 0
    corn[z_filt] = 0
    
    --image.display(conn)
    --image.display(corn)
    return conn,corn
end
--]]


--[[ POSE AND TRANSOFORMATION FUNCTIONS ]]

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

--[[ SAVIE FUNCTIONS ]]--

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
   
   libpc.downsample_with_widthheightinfo(downsampled_points, downsampled_rgb, downsampled_count, 
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
