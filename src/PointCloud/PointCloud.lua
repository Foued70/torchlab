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
    
    int downsample_without_widthheightinfo(double* downsampled_points, double* downsampled_rgb, 
                                int* downsampled_count, int* coord_list, 
                                double* points_list, double* rgb_list, 
                                int length, int binx, int biny, int binz);
    
    int make_normal_list(double* normal_list, double* normal_map,
                     int* hwindices, int length, int height, int width);
    
    int flatten_image_without_widthheightinfo(double* imagez, int* coords, double* dists,
                                              int length, int height, int width);
    
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
PointCloud.fudge_number = 10000
PointCloud.very_small_number = 0.00000000000000001

function PointCloud:__init(pcfilename, radius, numstd, option)

   log.tic()
   self.hwindices = nil;
   self.height = 0;
   self.width = 0;
   self.have_hw_data = nil
   self.points = nil;
   self.rgb = nil;
   self.count = 0;
   self.centroid   = torch.Tensor({{0,0,0}});
   local radius_near_far

   if radius then 
      if type(radius) == "number" then 
         radius_near_far = torch.zeros(2)
         radius_near_far[2] = radius
      else
         radius_near_far = radius
      end
   else
      radius_near_far = torch.zeros(2)
      radius_near_far[2] = 25
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
            local loaded = torch.load(pcfilename)
            self.format = loaded[1]
            if self.format == 1 then
               self.hwindices = loaded[2]
               self.height = self.hwindices:max(1)[1][1]
               self.width  = self.hwindices:max(1)[1][2]
               self.have_hw_data = true
            else
               self.height = 0
               self.width = 0
            end
            local pts = loaded[3]
            self.points = pts:type('torch.DoubleTensor'):div(PointCloud.fudge_number)
            self.rgb = loaded[4]
            self.count = self.points:size(1)
            self.normal_map = loaded[5]
            if self.normal_map then 
               self.normal_map = self.normal_map:type('torch.DoubleTensor'):div(PointCloud.fudge_number)
            end
            self.local_scan_center = loaded[6] 
            self.local_to_global_pose = loaded[7]
            self.local_to_global_rotation = loaded[8]

            self:reset_point_stats()
         else
            error('arg #1 must either be empty or a valid file: '..pcfilename)
         end
      else
         error('arg #1 must either be empty or a valid file: '..pcfilename)
      end
   end
end

function PointCloud:reset_point_stats()
   self.centroid=torch.Tensor({{0,0,self.points:mean(1)[1][3]}})
   if self.format == 2 then
     self.centroid[1][3] = 0
   end
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
   collectgarbage()
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

   if (countColumns == 6) then
      -- x y z r g b 
      self.format = 0
   elseif  (countColumns == 8) then
      -- h w x y z r g b 
      self.format = 1
   elseif  (countColumns == 3) then
      -- x y z 
      -- TODO make this format 0 as that is more consistent, but would
      -- be incompatible with already processed and saved pointclouds
      self.format = 2 
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
   if self.format==1 then
      offset = 2
      h = xyzrgbTensor:select(2,1)+1
      w = xyzrgbTensor:select(2,2)+1
   end
   local x = xyzrgbTensor:select(2,1+offset)
   local y = xyzrgbTensor:select(2,2+offset)
   local z = xyzrgbTensor:select(2,3+offset)
   local r, g, b
   if self.format<2 then
      r = xyzrgbTensor:select(2,4+offset)
      g = xyzrgbTensor:select(2,5+offset)
      b = xyzrgbTensor:select(2,6+offset)
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
      self.height = h[indexGood]:max()
      self.width  = w[indexGood]:max()
   end
   
   self.points   = torch.cat(torch.cat(x[indexGood],y[indexGood],2),z[indexGood],2)
   self.count    = self.points:size(1)
   local meanz   = z[indexGood]:sum()/(self.count+.000001)
   self.centroid = torch.Tensor({{0, 0, meanz}})

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
      local meanz   = z[indexGood]:sum()/(self.count+.000001)
      
      self.centroid = torch.Tensor({{0, 0, meanz}})
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

function PointCloud:get_normal_image(recompute)
   if self.have_hw_data then
     local pmap = self:get_normal_map(recompute):clone()
     local pmap = pmap:add(1):div(2)
     return pmap
   else
      print("can't make panoramic image, no w/h info given")
   end
end

function PointCloud:get_flattened_images_without_widthheightinfo(scale)

  local tic = log.toc()
  local toc = 0
  
  scale = scale+PointCloud.very_small_number
  local ranges = self.radius:clone():mul(2)
  local minv = self.radius:clone():mul(-1)
  local maxv = self.radius:clone()
  local pix = ranges:clone():div(scale):floor()
  local height = pix[1]
  local width = pix[2]
  local elim_dist = 0.25
  
  local imagez = torch.zeros(height,width)
  
  local points = self.points[{{},{1,2}}]:clone()
  
  local coords = torch.Tensor(points:size()):copy(points)
  coords:add(minv:sub(1,2):repeatTensor(self.count,1):mul(-1)):div(scale):floor()
  coords = coords:type('torch.IntTensor')
  
  local z_val = self.points[{{},{3}}]:clone()
  local z_big = z_val:lt(self.maxval[3]-2*elim_dist)
  local z_sma = z_val:gt(self.minval[3]+1*elim_dist)
  local filtr = z_big:clone():cmul(z_sma):type('torch.DoubleTensor')
  
  local dists = torch.Tensor(points:size()):copy(points):pow(2):sum(2):squeeze():clone()
  dists:cmul(filtr)
  
  points = nil
  collectgarbage()
  
  
  libpc.flatten_image_without_widthheightinfo(torch.data(imagez), 
                                              torch.data(coords:clone():contiguous()), 
                                              torch.data(dists:clone():contiguous()), 
                                              self.count, height, width);                                     

  imagez:div(imagez:max()+PointCloud.very_small_number)
  imagez = imagez:clone():resize(height,width):repeatTensor(3,1,1)
  
  toc = log.toc()-tic
  tic = tic + toc
  print('flatten_image_0: finish: '..toc)
  
  collectgarbage()

  return imagez
  
end

function PointCloud:get_flattened_images_with_widthheightinfo(scale,mask,numCorners)

  local tic = log.toc()
  local toc = 0
  
  scale = scale+PointCloud.very_small_number
  if self.format == 2 then
    scale = scale*1000
  end
  print(scale)
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
  if mask then
    connections:cmul(mask)
  end
  
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
    local ps = math.ceil(0.10/scale)
    local bs = math.ceil(0.05/scale)
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
  
  return imagez,corners,image_corners
end

function PointCloud:get_flattened_images(scale,mask,numCorners)
   if not(self.have_hw_data) then
    return self:get_flattened_images_without_widthheightinfo(scale)
  else
    return self:get_flattened_images_with_widthheightinfo(scale,mask,numCorners)
  end
end

function PointCloud:get_connections_and_corners_2()
  if self.have_hw_data then
    local height = self.height
    local width = self.width
    local normal = self:get_normal_map()
    local phi,theta = self:get_normal_phi_theta()
    local snormal,sphi,stheta = self:get_smooth_normal()
    local xyz = self:get_centered_xyz_map_no_mask()
    
    local theta_thresh = math.pi/2
    local plane_thresh_conn = 0.01
    local plane_thresh_corn = 0.25
    local z_thresh = 0.75
    if self.format == 2 then
      plane_thresh_conn = 5000
      plane_thresh_corn = 6000
    end
    
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
end
--[[]]
function PointCloud:get_connections_and_corners()
    if self.have_hw_data then
    
        if self.format == 2 then
          return self:get_connections_and_corners_2()
        else
        local tic = log.toc()
        local toc = 0
        
        self:get_normal_map()
        
        local pts = self:get_xyz_map_no_mask():clone()
        
        local height = self.height
        local width = self.width
        local normal_map = self.normal_map:clone()
        local smooth_normal_map = self:get_smooth_normal():clone()--normal_map:clone()--
        
        local plane_const = torch.zeros(height,width)
        plane_const:add(pts:clone():cmul(smooth_normal_map):sum(1):squeeze())
        
        image.display(plane_const)

        local shift_normal_map = torch.zeros(3,height,width)
        local shift_plane_const = torch.zeros(height,width)
        
        shift_normal_map:sub(1,3,1,height,2,width):add(smooth_normal_map:clone():sub(1,3,1,height,1,width-1))
        shift_normal_map:sub(1,3,1,height,1,1):add(smooth_normal_map:clone():sub(1,3,1,height,width,width))
        shift_plane_const:sub(1,height,2,width):add(plane_const:clone():sub(1,height,1,width-1))
        shift_plane_const:sub(1,height,1,1):add(plane_const:clone():sub(1,height,width,width))
        
        local dist_from_plane_l1 = shift_normal_map:clone():cmul(pts):sum(1):squeeze():add(shift_plane_const:clone():mul(-1)):abs():clone()
        
        shift_normal_map = torch.zeros(3,height,width)
        shift_plane_const = torch.zeros(height,width)
        
        shift_normal_map:sub(1,3,1,height,3,width):add(smooth_normal_map:clone():sub(1,3,1,height,1,width-2))
        shift_normal_map:sub(1,3,1,height,1,2):add(smooth_normal_map:clone():sub(1,3,1,height,width-1,width))
        shift_plane_const:sub(1,height,3,width):add(plane_const:clone():sub(1,height,1,width-2))
        shift_plane_const:sub(1,height,1,2):add(plane_const:clone():sub(1,height,width-1,width))
        
        local dist_from_plane_l2 = shift_normal_map:clone():cmul(pts):sum(1):squeeze():add(shift_plane_const:clone():mul(-1)):abs():clone()
        
        shift_plane_const = nil
        
        smnmp = smooth_normal_map:sub(1,2):clone()
        
        shift_normal_map = torch.zeros(2,height,width)
        
        shift_normal_map:sub(1,2,1,height,2,width):add(smnmp:clone():sub(1,2,1,height,1,width-1))
        shift_normal_map:sub(1,2,1,height,1,1):add(smnmp:clone():sub(1,2,1,height,width,width))
        
        local diff_from_normal_l = shift_normal_map:clone():cmul(smnmp:clone()):sum(1):squeeze():cdiv(
                                   shift_normal_map:clone():pow(2):sum(1):sqrt():add(
                                   smnmp:clone():pow(2):sum(1):sqrt()):squeeze()):acos()

        shift_normal_map = torch.zeros(2,height,width)
        
        shift_normal_map:sub(1,2,1,height,1,width-1):add(smnmp:clone():sub(1,2,1,height,2,width))
        shift_normal_map:sub(1,2,1,height,width,width):add(smnmp:clone():sub(1,2,1,height,1,1))
        
        local diff_from_normal_r = shift_normal_map:clone():cmul(smnmp:clone()):sum(1):squeeze():cdiv(
                                   shift_normal_map:clone():pow(2):sum(1):sqrt():add(
                                   smnmp:clone():pow(2):sum(1):sqrt()):squeeze()):acos()
        
        shift_normal_map = nil
         
        local pc_tol_conn = 0.01
        local nm_tol_corn = .1 --0.25
        local pc_tol_corn = 0.25
        local z_tol = 0.25
        if format == 2 then
          pc_tol_conn = 1000
          pc_tol_corn = pc_tol_corn * 1000
        end
        
        local zconstraint = smooth_normal_map:select(1,3):clone():abs():lt(z_tol)
        local extantpts = pts:clone():pow(2):sum(1):squeeze():gt(0):cmul(pts:clone():pow(2):sum(1):squeeze():lt(math.huge))
        
        local ptdiff = pts:clone():add(self.centroid:repeatTensor(1,1,1):transpose(1,3):repeatTensor(1,height,width):mul(-1))
        local ptnm = ptdiff:clone():pow(2):sum(1):sqrt():squeeze()
        local ptz = ptdiff:select(1,3):clone():squeeze():cdiv(ptnm:clone():add(PointCloud.very_small_number)):abs():lt(0.9)
        local ptd = ptnm:gt(0.05)
        zconstraint:cmul(ptz):cmul(ptd)
        
        ptnm = nil
        ptdiff = nil
        ptz = nil
        
        --image.display(dist_from_plane_l1)
        --image.display(dist_from_plane_l2)
        --image.display(diff_from_normal_l)
        --image.display(diff_from_normal_r)
        
        local connections = dist_from_plane_l1:clone():gt(pc_tol_conn):add(dist_from_plane_l1:clone():gt(pc_tol_conn)):eq(0)
        local corners = (diff_from_normal_l - diff_from_normal_r):abs():gt(nm_tol_corn):add(
                         dist_from_plane_l1:clone():gt(pc_tol_corn):add(dist_from_plane_l1:clone():gt(pc_tol_corn))):gt(0)

        corners:cmul(zconstraint):cmul(extantpts)
        connections:cmul(zconstraint):cmul(extantpts)
        
        normal_map = nil
        smooth_normal_map = nil
        pts = nil
        plane_const = nil
        plane_const_norm = nil
        zconstraint = nil
        extantpts = nil
        compare_plane_lr = nil
        compare_plane_ud = nil
        compare_normal_lr = nil
        compare_normal_ud = nil
        dist_from_plane_l1 = nil
        dist_from_plane_l2 = nil
        diff_normal_l = nil
        diff_normal_r = nil
        collectgarbage()
        
        toc = log.toc()-tic
        tic = tic + toc
        print('get_connections_and_corners: '..toc)
        
        return connections,corners
      end
    end
end
--[[]]

function PointCloud:downsample_with_widthheightinfo(leafsize)

   local tic = log.toc()
   local toc = 0
   
   local scale = leafsize + PointCloud.very_small_number
   local ranges = self.maxval:clone():squeeze():add(self.minval:clone():squeeze():mul(-1))
   local pts = self:get_xyz_map_no_mask():clone()
   local crd = pts:clone():add(self.minval:clone():mul(-1):squeeze():repeatTensor(self.width,self.height,1):transpose(1,3)):div(scale):floor():add(1)
   local rgb = self:get_rgb_map_no_mask():type('torch.DoubleTensor')
   pts = crd:clone():add(-1):mul(scale):add(self.minval:squeeze():repeatTensor(self.height,self.width,1):transpose(1,3))
   
   local downsampled = PointCloud.new()
   downsampled.height = 0;
   downsampled.width = 0;
   downsampled.format = 0;
   
   local count = torch.zeros(1):type('torch.IntTensor')
   local points = torch.zeros(self.count,3)
   local dsrgb = torch.zeros(self.count,3)
   
   local downsampled_points = torch.data(points)
   local downsampled_rgb = torch.data(dsrgb)
   local downsampled_count = torch.data(count)
   local coord_map = torch.data(crd:clone():contiguous())
   local points_map = torch.data(pts:clone():contiguous())
   local rgb_map = torch.data(rgb:clone():contiguous())

   toc = log.toc()-tic
   tic = tic + toc 
   print('downsample_0: setup: '..toc)
   
   libpc.downsample_with_widthheightinfo(downsampled_points, downsampled_rgb, downsampled_count, 
                                  coord_map, points_map, rgb_map, self.height, self.width)
   
   toc = log.toc()-tic
   tic = tic + toc                                
   print('downsampled_0: loop '..toc)
   
   count = count:clone()[1]
   if count == 0 then
     count = 1
   end
   points = points:clone():sub(1,count)
   dsrgb = dsrgb:clone():sub(1,count):type('torch.ByteTensor')
   
   downsampled.points = points:clone()
   downsampled.rgb = dsrgb:clone()
   downsampled.count = count
   downsampled:set_local_to_global_pose(self:get_local_to_global_pose())
   downsampled:set_local_to_global_rotation(self:get_local_to_global_rotation())
   downsampled:reset_point_stats()
   
   count = nil
   points = nil
   dsrgb = nil
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
   return downsampled;
   
end

function PointCloud:downsample_without_widthheightinfo(leafsize)

   local tic = log.toc()
   local toc = 0
   
   local scale = leafsize + PointCloud.very_small_number
   local ranges = self.maxval:clone():squeeze():add(self.minval:clone():squeeze():mul(-1))
   local pts = self.points:clone()
   local crd = pts:clone():add(self.minval:clone():mul(-1):squeeze():repeatTensor(self.count,1)):div(scale):floor()
   local rgb = self.rgb:clone():type('torch.DoubleTensor')
   pts = crd:clone():mul(scale):add(self.minval:squeeze():repeatTensor(self.count,1))
   
   local downsampled = PointCloud.new()
   downsampled.height = 0;
   downsampled.width = 0;
   downsampled.format = 0;
   
   local count = torch.zeros(1):type('torch.IntTensor')
   local points = torch.zeros(self.count,3)
   local dsrgb = torch.zeros(self.count,3)
   
   local downsampled_points = torch.data(points)
   local downsampled_rgb = torch.data(dsrgb)
   local downsampled_count = torch.data(count)
   local coord_list = torch.data(crd:type('torch.IntTensor'):clone():contiguous())
   local points_list = torch.data(pts:clone():contiguous())
   local rgb_list = torch.data(rgb:clone():contiguous())
   local binsizes = crd:max(1):add(1):squeeze()
   local binx = binsizes[1]
   local biny = binsizes[2]
   local binz = binsizes[3]

   toc = log.toc()-tic
   tic = tic + toc 
   print('downsample_1: setup: '..toc)
   
   libpc.downsample_without_widthheightinfo(downsampled_points, downsampled_rgb, downsampled_count, 
                                     coord_list, points_list, rgb_list, self.count,
                                     binx, biny, binz)
   
   toc = log.toc()-tic
   tic = tic + toc                                
   print('downsample_1: loop: '..toc)
   
   count = count[1]
   if count == 0 then
     count = 1
   end
   points = points:sub(1,count)
   dsrgb = dsrgb:sub(1,count):type('torch.ByteTensor')
   
   downsampled.points = points:clone()
   downsampled.rgb = dsrgb:clone()
   downsampled.count = count
   downsampled:set_local_to_global_pose(self:get_local_to_global_pose())
   downsampled:set_local_to_global_rotation(self:get_local_to_global_rotation())
   downsampled:reset_point_stats()
   
   count = nil
   points = nil
   dsrgb = nil
   scale = nil
   ranges = nil
   nmp = nil
   pts = nil
   crd = nil
   rgb = nil
   downsampled_points = nil
   downsampled_rgb = nil
   downsampled_count = nil
   coord_list = nil
   points_list = nil
   rgb_list = nil
   binsizes = nil
   binx = nil
   biny = nil
   binz = nil
   
   toc = log.toc()-tic
   tic = tic + toc 
   print('downsampled_1: finish: '..toc)
   
   collectgarbage()
   return downsampled;
   
end

function PointCloud:downsample(leafsize)
  if self.have_hw_data then
    return self:downsample_with_widthheightinfo(leafsize)
  else
    return self:downsample_without_widthheightinfo(leafsize)
  end
end

function PointCloud:get_normal_phi_theta()
  if self.have_hw_data then
    local tic = log.toc()
    local point_map = self:get_centered_xyz_map_no_mask()
    local height = self.height
    local width = self.width
    local phi = torch.zeros(height,width):clone():contiguous()
    local theta = torch.zeros(height,width):clone():contiguous()
  
    libpc.phi_map(torch.data(phi),torch.data(point_map:clone():contiguous()),height,width)
    libpc.theta_map(torch.data(theta),torch.data(point_map:clone():contiguous()),height,width)
  
    local filter = phi:lt(-10):add(theta:lt(-10)):gt(0)
    phi[filter] = 0
    theta[filter] = 0

    print('get_normal_phi_theta: '..(log.toc()-tic))
    return phi, theta, filter
  end
end

function PointCloud:get_normal_map(force)
  if self.have_hw_data and ((not self.normal_map) or force) then
    local tic = log.toc()
    local phi,theta,filter = self:get_normal_phi_theta()
    local height = self.height
    local width = self.width
    local nmp = torch.zeros(3,height,width)
  
    nmp[3] = phi:clone():sin()
    nmp[2] = phi:clone():cos():cmul(theta:clone():sin())
    nmp[1] = phi:clone():cos():cmul(theta:clone():cos())
  
    nmp[1][filter] = 0
    nmp[2][filter] = 0
    nmp[3][filter] = 0
    
    print('get_normal_map: '..(log.toc()-tic))
    self.normal_map = nmp
  end
  return self.normal_map
end

function PointCloud:get_smooth_normal(max_win, phi_diff, theta_diff)

  if self.have_hw_data then
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
  
    local phi,theta,filter = self:get_normal_phi_theta()
    local extant_map = filter:clone():eq(0)
    local smooth_phi = phi:clone():contiguous():fill(0)
    local smooth_theta = theta:clone():contiguous():fill(0)
  
    libpc.phi_map_smooth(torch.data(smooth_phi),torch.data(phi:clone():contiguous()),torch.data(extant_map:clone():contiguous()),height,width,max_win,phi_diff)
    libpc.theta_map_smooth(torch.data(smooth_theta),torch.data(theta:clone():contiguous()),torch.data(extant_map:clone():contiguous()),height,width,max_win,theta_diff)
  
    smooth_phi[filter] = 0
    smooth_theta[filter] = 0
    
    local smooth_nmp = torch.zeros(3,height,width)
  
    smooth_nmp[3] = smooth_phi:clone():sin()
    smooth_nmp[2] = smooth_phi:clone():cos():cmul(smooth_theta:clone():sin())
    smooth_nmp[1] = smooth_phi:clone():cos():cmul(smooth_theta:clone():cos())
  
    smooth_nmp[1][filter] = 0
    smooth_nmp[2][filter] = 0
    smooth_nmp[3][filter] = 0
  
    print('get_smooth_normal_map: '..(log.toc()-tic))
  
    return smooth_nmp, smooth_phi, smooth_theta
  end
end

function PointCloud:get_normal_list()

  if self.have_hw_data then
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
  end
  return nmlist
end

function PointCloud:get_3dtree()
   self.k3dtree = kdtree.new(self.points)
   collectgarbage()
end

-- I made these [MS]

-- ugly to get z position of faro scans
function PointCloud:estimate_faro_pose(degree_above, degree_below)
   degree_above = degree_above or 90
   degree_below = degree_below or 60
   -- get middle z value
   local rowid   = math.floor((degree_above / (degree_above + degree_below)) * self.height)
   local xyz_map = self:get_xyz_map()
   local midrow  = xyz_map[{3,rowid,{}}]
   return torch.Tensor({0,0,midrow[midrow:gt(0)]:mean()})
end

-- TODO make pose and rot getter and setter methods inherited from a View

-- store the position of the scan center in local coordiantes
function PointCloud:set_local_scan_center(pose)
   self.local_scan_center = pose
end

function PointCloud:get_local_scan_center()
   if not self.local_scan_center then
      self.local_scan_center = self.centroid
   end
   return self.local_scan_center
end

function PointCloud:get_global_scan_center()
   local pose = self:get_local_to_global_pose()
   local local_center = self:get_local_scan_center()
   return local_center + pose
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

function PointCloud:get_global_points_as_pc()
   local pose = self:get_local_to_global_pose()
   local rot  = self:get_local_to_global_rotation()
   pc = PointCloud.new()
   pc.points=rotate_translate(rot,pose,self.points)
   pc.rgb = self.rgb
   pc.count = self.count
   pc:reset_point_stats()
   pc.format = 0
  return pc
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

function PointCloud:get_centered_xyz_map_no_mask()

  local pts = self:get_xyz_map_no_mask():clone()
  local center = self.centroid:squeeze()
  
  for i=1,3 do
    pts:select(1,i):add(-center[i])
  end
  return pts
end

function PointCloud:get_depth_image()

    local xyz_map = self:get_centered_xyz_map_no_mask()
    
    local depth_map = xyz_map:norm(2,1):squeeze()
    depth_map[self.mask_map] = 0
    
    return depth_map
end

function PointCloud:get_depth_map()

   if not self.depth_map then
      local xyz_map = self:get_xyz_map():clone()
      local center = self.centroid:squeeze()
      for i = 1,3 do
         xyz_map[i]:add(-center[i])
      end

      self.depth_map = xyz_map:norm(2,1):squeeze()
      self.depth_map[self.mask_map] = 0
   end

   return self.depth_map

end

-- rgb_map is a 2D equirectangular grid of rgb values which corresponds to xyz_map
function PointCloud:load_rgb_map(imagefile, type)
   type = type or "byte"
   local rgb_map = image.load(imagefile, type)
   if rgb_map then 
      self.rgb_map = rgb_map
      self.rgb = nil -- needs to be replaced
   else 
      print("WARNING: no rgb map loaded")
   end
end

-- put rgb values such as those read in from an xyzrgb format into a "map"
function PointCloud:get_rgb_map(force)
   if (not self.rgb_map) or force then  
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
function PointCloud:get_rgb_map_no_mask(force)
   if (not self.rgb_map_no_mask) or force then  
      if self.rgb then
         rgbT = self:get_rgb():transpose(1,2):contiguous()
         index, mask = self:get_index_and_mask()
         self.rgb_map_no_mask = util.addr.remap(rgbT,index,mask,torch.Tensor(3,self.height,self.width))
      else
         print("no rgb_map and no rgb values")
         return nil
      end
   end
   return self.rgb_map_no_mask
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
      pose       = self:get_local_scan_center():clone():mul(-1)
      self.depth = torch.sqrt(torch.norm(torch.add(points, pose:reshape(1,3):expandAs(points)),2,2)):squeeze()
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

function PointCloud:write(filename)

   if util.fs.extname(filename)==PointCloud.PC_OD_EXTENSION then
      local pts = self.points:clone():mul(PointCloud.fudge_number):type('torch.IntTensor')
      local nmp
      if self.normal_map then 
         nmp = self.normal_map:clone():mul(PointCloud.fudge_number):type('torch.IntTensor')
      end
      torch.save(filename, {self.format, self.hwindices, pts, self:get_rgb(), nmp,self.local_scan_center, self.local_to_global_pose, self.local_to_global_rotation})
   elseif util.fs.extname(filename)==PC_ASCII_EXTENSION then
      local file = io.open(filename, 'w');
      local tmpt = torch.range(1,self.count)
      local rgb = self:get_rgb()
      local points = self:get_global_points()
      
      tmpt:apply(function(i)
                    local pt = points[i]
                    local rgbx = rgb[i]
                    if self.have_hw_data then
                       local ind = self.hwindices[i]:clone()
                       local ih = ind[1]
                       local iw = ind[2]
                       file:write(''..(ih-1)..' '..(iw-1)..' '..pt[1]..' '..pt[2]..' '..pt[3]..' '..rgbx[1]..' '..rgbx[2]..' '..rgbx[3]..'\n')
                    else
                        file:write(''..pt[1]..' '..pt[2]..' '..pt[3]..' '..rgbx[1]..' '..rgbx[2]..' '..rgbx[3]..'\n')
                    end
                 end)
      file:close()
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

function PointCloud:save_global_points_to_xyz(fname)
   saveHelper_xyz(self:get_global_points(), self:get_rgb(), fname)
end

function PointCloud:save_downsampled_to_xyz(leafsize, fname)
   local downsampled = self:downsample(leafsize)
   saveHelper_xyz(downsampled.points, downsampled.rgb, fname)
end

function PointCloud:save_downsampled_global_to_xyz(leafsize, fname)
   local downsampled = self:downsample(leafsize)
   local pose = self:get_local_to_global_pose()
   local rot  = self:get_local_to_global_rotation()
   downsampled.points = rotate_translate(rot,pose,downsampled.points)
   saveHelper_xyz(downsampled.points, downsampled.rgb, fname)
end

-- old functions
--[[
function PointCloud:get_normal_map(force)
  if self.have_hw_data and ((not self.normal_map) or force) then
  
    local snum = 1
    
    local tic = log.toc()
    local toc = 0

    local xyz = self:get_xyz_map_no_mask():clone()
    local height = self.height
    local width = self.width
    local shift = torch.zeros(xyz:size())
    
    local pts = self:get_centered_xyz_map_no_mask():clone()
    
    shift:sub(1,3,1,height,1+snum,width):add(xyz:sub(1,3,1,height,1,width-snum):clone())
    shift:sub(1,3,1,height,1,snum):add(xyz:sub(1,3,1,height,width-snum+1,width):clone())
    
    local minus_l = torch.Tensor(xyz:size()):copy(xyz:clone())
	minus_l:add(shift:clone():mul(-1))  
	
	shift:fill(0)
	shift:sub(1,3,1,height,1,width-snum):add(xyz:sub(1,3,1,height,1+snum,width):clone())
    shift:sub(1,3,1,height,width-snum+1,width):add(xyz:sub(1,3,1,height,1,snum):clone())
    
    local minus_r = torch.zeros(xyz:size()):copy(xyz:clone())
	minus_r:add(shift:clone():mul(-1))
    
    shift:fill(0)
    shift:sub(1,3,1+snum,height):add(xyz:sub(1,3,1,height-snum):clone())
    shift:sub(1,3,1,snum):add(xyz:sub(1,3,1,snum):clone())
    
    local minus_u = torch.zeros(xyz:size()):copy(xyz:clone())
	minus_u:add(shift:clone():mul(-1))
	
	shift:fill(0)
	shift:sub(1,3,1,height-snum):add(xyz:sub(1,3,1+snum,height):clone())
    shift:sub(1,3,height-snum+1,height):add(xyz:sub(1,3,height-snum+1,height):clone())
    
    local minus_d = torch.zeros(xyz:size()):copy(xyz:clone())
    minus_d:add(shift:clone():mul(-1))
	
    local minus_1_x = minus_l:select(1,1):clone()
    local minus_1_y = minus_l:select(1,2):clone()
    local minus_1_z = minus_l:select(1,3):clone()
            
    local minus_2_x = minus_u:select(1,1):clone()
    local minus_2_y = minus_u:select(1,2):clone()
    local minus_2_z = minus_u:select(1,3):clone()
        
    local crossprod_1 = torch.Tensor(3,height,width)
        
    crossprod_1[1] = minus_1_y:clone():cmul(minus_2_z):add(
                     minus_1_z:clone():cmul(minus_2_y):mul(-1))
    crossprod_1[2] = minus_1_z:clone():cmul(minus_2_x):add(
                     minus_1_x:clone():cmul(minus_2_z):mul(-1))
    crossprod_1[3] = minus_1_x:clone():cmul(minus_2_y):add(
                     minus_1_y:clone():cmul(minus_2_x):mul(-1))
    
    local crossprod_norm = crossprod_1:clone():pow(2):sum(1):sqrt():repeatTensor(3,1,1)
    crossprod_1:cdiv(crossprod_norm:add(PointCloud.very_small_number))
    
    local d = pts:clone():cmul(crossprod_1):sum(1):repeatTensor(3,1,1)
    crossprod_1 = crossprod_1:clone():cmul(d:clone():ge(0):type('torch.DoubleTensor')):add(
                crossprod_1:clone():mul(-1):cmul(d:clone():lt(0):type('torch.DoubleTensor')))
                     
    minus_1_x = minus_u:select(1,1):clone()
    minus_1_y = minus_u:select(1,2):clone()
    minus_1_z = minus_u:select(1,3):clone()
            
    minus_2_x = minus_r:select(1,1):clone()
    minus_2_y = minus_r:select(1,2):clone()
    minus_2_z = minus_r:select(1,3):clone()
	
	local crossprod_2 = torch.Tensor(3,height,width)
        
    crossprod_2[1] = minus_1_y:clone():cmul(minus_2_z):add(
                     minus_1_z:clone():cmul(minus_2_y):mul(-1))
    crossprod_2[2] = minus_1_z:clone():cmul(minus_2_x):add(
                     minus_1_x:clone():cmul(minus_2_z):mul(-1))
    crossprod_2[3] = minus_1_x:clone():cmul(minus_2_y):add(
                     minus_1_y:clone():cmul(minus_2_x):mul(-1))
    
    crossprod_norm = crossprod_2:clone():pow(2):sum(1):sqrt():repeatTensor(3,1,1)
    crossprod_2:cdiv(crossprod_norm:add(PointCloud.very_small_number))
    
    d = pts:clone():cmul(crossprod_2):sum(1):repeatTensor(3,1,1)
    crossprod_2 = crossprod_2:clone():cmul(d:clone():ge(0):type('torch.DoubleTensor')):add(
                crossprod_2:clone():mul(-1):cmul(d:clone():lt(0):type('torch.DoubleTensor')))
    
    minus_1_x = minus_r:select(1,1):clone()
    minus_1_y = minus_r:select(1,2):clone()
    minus_1_z = minus_r:select(1,3):clone()
            
    minus_2_x = minus_d:select(1,1):clone()
    minus_2_y = minus_d:select(1,2):clone()
    minus_2_z = minus_d:select(1,3):clone()
	
	local crossprod_3 = torch.Tensor(3,height,width)
        
    crossprod_3[1] = minus_1_y:clone():cmul(minus_2_z):add(
                     minus_1_z:clone():cmul(minus_2_y):mul(-1))
    crossprod_3[2] = minus_1_z:clone():cmul(minus_2_x):add(
                     minus_1_x:clone():cmul(minus_2_z):mul(-1))
    crossprod_3[3] = minus_1_x:clone():cmul(minus_2_y):add(
                     minus_1_y:clone():cmul(minus_2_x):mul(-1))
                     
    crossprod_norm = crossprod_3:clone():pow(2):sum(1):sqrt():repeatTensor(3,1,1)
    crossprod_3:cdiv(crossprod_norm:add(PointCloud.very_small_number))
    
    d = pts:clone():cmul(crossprod_3):sum(1):repeatTensor(3,1,1)
    crossprod_3 = crossprod_3:clone():cmul(d:clone():ge(0):type('torch.DoubleTensor')):add(
                crossprod_3:clone():mul(-1):cmul(d:clone():lt(0):type('torch.DoubleTensor')))
                     
    minus_1_x = minus_d:select(1,1):clone()
    minus_1_y = minus_d:select(1,2):clone()
    minus_1_z = minus_d:select(1,3):clone()
            
    minus_2_x = minus_l:select(1,1):clone()
    minus_2_y = minus_l:select(1,2):clone()
    minus_2_z = minus_l:select(1,3):clone()
	
    local crossprod_4 = torch.Tensor(3,height,width)
        
    crossprod_4[1] = minus_1_y:clone():cmul(minus_2_z):add(
                     minus_1_z:clone():cmul(minus_2_y):mul(-1))
    crossprod_4[2] = minus_1_z:clone():cmul(minus_2_x):add(
                     minus_1_x:clone():cmul(minus_2_z):mul(-1))
    crossprod_4[3] = minus_1_x:clone():cmul(minus_2_y):add(
                     minus_1_y:clone():cmul(minus_2_x):mul(-1))
    
    crossprod_norm = crossprod_4:clone():pow(2):sum(1):sqrt():repeatTensor(3,1,1)
    crossprod_4:cdiv(crossprod_norm:add(PointCloud.very_small_number))
    
    d = pts:clone():cmul(crossprod_4):sum(1):repeatTensor(3,1,1)
    crossprod_14= crossprod_4:clone():cmul(d:clone():ge(0):type('torch.DoubleTensor')):add(
                crossprod_4:clone():mul(-1):cmul(d:clone():lt(0):type('torch.DoubleTensor')))
    
    minus_l = nil
    minus_r = nil
    minus_u = nil
    minus_d = nil
    
    minus_1_x = nil
    minus_1_y = nil
    minus_1_z = nil
            
    minus_2_x = nil
    minus_2_y = nil
    minus_2_z = nil
    shift = nil
    
    collectgarbage()
    
    local crossprod = torch.zeros(3,height,width)
    
    local c1 = torch.data(crossprod_1:clone():contiguous())
    local c2 = torch.data(crossprod_2:clone():contiguous())
    local c3 = torch.data(crossprod_3:clone():contiguous())
    local c4 = torch.data(crossprod_4:clone():contiguous())
    local cc = torch.data(crossprod)
    
    local thresh = 0.1
    
    libpc.compute_normal_helper_pick(cc,c1,c2,c3,c4,height,width,thresh)
    crossprod_norm = crossprod:clone():pow(2):sum(1):sqrt():repeatTensor(3,1,1)
    crossprod = crossprod:cdiv(crossprod_norm:add(PointCloud.very_small_number))
    
    d = pts:clone():cmul(crossprod):sum(1):repeatTensor(3,1,1)
    crossprod = crossprod:clone():cmul(d:clone():ge(0):type('torch.DoubleTensor')):add(
                crossprod:clone():mul(-1):cmul(d:clone():lt(0):type('torch.DoubleTensor')))
    
    cc = nil
    c1 = nil
    c2 = nil
    c3 = nil
    c4 = nil
    
    crossprod_1 = nil
    crossprod_2 = nil
    crossprod_3 = nil
    crossprod_4 = nil
    
    pts = nil
    d = nil
    
    collectgarbage()
    
    self.normal_map =  crossprod:clone()
    
    toc = log.toc()-tic
    tic = tic + toc 
    print('normal_map: '..toc)
    
    collectgarbage()
    
  end
  return self.normal_map
end

function PointCloud:get_phi_and_theta_map()
  local nmp = self:get_normal_map():clone()
  
  local phi = nmp[3]:clone():asin()
  local theta = nmp[1]:clone():cdiv(phi:clone():cos()):acos()
  local theta = theta:clone():cmul(nmp[2]:clone():ge(0):type('torch.DoubleTensor')):add(
                theta:clone():mul(-1):cmul(nmp[2]:clone():lt(0):type('torch.DoubleTensor')))
                
  phi:apply(function(ph)
    if not(math.abs(ph) >= 0) then
      return 0
    elseif math.abs(ph) == math.huge then
      return 0
    end
  end)
  
  theta:apply(function(th)
    if not(math.abs(th) >= 0) then
      return 0
    elseif math.abs(th) == math.huge then
      return 0
    end
  end)
  
  local d = nmp:clone():cmul(self:get_xyz_map_no_mask()):norm(2,1):squeeze()
  collectgarbage()
  
  local img = torch.Tensor(3,self.height,self.width)
  img[1] = phi
  img[2] = theta
  img[3] = d
  
  return phi,theta,img
end

function PointCloud:get_normal_map_nine_neighbors()
  
  local tic = log.toc()
  local toc = 0
  
  local nmp = torch.zeros(3,self.height,self.width)
  local pts = self:get_centered_xyz_map_no_mask()
  local xyznorm = pts:norm(2,1):squeeze()
  
  local abnorm = pts:sub(1,2):clone():norm(2,1):squeeze()
  local factormax = abnorm:max()
    
  torch.range(1,self.height):apply(function(ch)
    
    local xyznorm_ch = xyznorm[ch]
    
    local abnorm_ch = abnorm[ch]
    
    local xyznorm_hh
    
    torch.range(1,self.width):apply(function(cw)
      
      local dist = xyznorm_ch[cw]
      local factor = factormax/abnorm_ch[cw]
      local win = math.min(3,math.ceil(factor))
      local win = 1
      local numv = math.pow(2*win+1,2)
      
      if dist > PointCloud.very_small_number then
      
        local xyz_c = pts:sub(1,3,ch,ch,cw,cw):squeeze()
        local tab = torch.zeros(numv,3)
        local count = 0
      
        for hh = ch-win,ch+win do
          if hh > 0 and hh < self.height then
          
            if hh == ch then
              xyznorm_hh = xyznorm_ch
            else
              xyznorm_hh = xyznorm[hh]
            end
      
            for w = cw-win,cw+win do
              
              local ww = (w - 1) % self.width + 1
              
              if xyznorm_hh[ww] > 0 then
                local xyz = pts:sub(1,3,hh,hh,ww,ww):squeeze()
                local dist = xyz_c:dist(xyz)
                count = count + 1
                tab[count]=(xyz-xyz_c)
              end
              
              ww = nil
              
            end
      
          end
      
        end
        
        if count > 2 then
          --tab = tab:sub(1,count):clone():t():clone()
          tab = tab:t():clone()
          local u,s,v = torch.svd(tab)
          local mins, minsi = s:min(1)
          local nm = u:select(2,minsi[1]):clone()
          nm = nm:squeeze():clone()
          
          nmp:sub(1,3,ch,ch,cw,cw):copy(nm:clone())
          if cw == 1 and ch % 100 == 0 then
            print(ch,cw,count,log.toc()-tic)
          end
          xyzn = nil
          nm = nil
          mins = nil
          minsi = nil
          u = nil
          s = nil
          v = nil
        end
        
        xyz_c = nil
        tab = nil
        count = nil
      end
      
    end)
    
  end)
  
  local d = nmp:clone():cmul(pts):sum(1):repeatTensor(3,1,1)
  nmp = nmp:clone():cmul(d:ge(0):type('torch.DoubleTensor')):add(
        nmp:clone():mul(-1):cmul(d:lt(0):type('torch.DoubleTensor')))
  
  pts = nil
  xyznorm = nil
  d = nil
  
  collectgarbage()
  toc = log.toc()-tic
  tic = tic + toc
  print('nine_neighbor_normal: '..toc)
  return nmp
  
end

function PointCloud:get_smooth_normal(win, dist_thresh, norm_thresh, point_thresh)
    
    local tic = log.toc()
    local toc = 0
    
    if not win then
      win = 5
    end
    if not dist_thresh then
      dist_thresh = 0.005
    end
    if not norm_thresh then
      norm_thresh = 2.5
    end
    if not point_thresh then
      point_thresh = 0.1
    end
    
    local height = self.height
    local width = self.width
  
    local nmp = self:get_normal_map()
    local xyz = self:get_xyz_map()
    
    crossprod = torch.zeros(nmp:size())
    local d = torch.zeros(height,width)
    local xyznew = torch.zeros(xyz:size())
    
    local cc = torch.data(crossprod)
    local nn = torch.data(nmp:clone():contiguous())
    local pp = torch.data(xyz:clone():contiguous())
    
    libpc.compute_normal_help_blend(cc, nn, pp, height, width, win, dist_thresh, norm_thresh, point_thresh)
    crossprod_norm = crossprod:clone():pow(2):sum(1):sqrt():repeatTensor(3,1,1)
    crossprod = crossprod:cdiv(crossprod_norm:add(PointCloud.very_small_number))
    
    toc = log.toc()-tic
    tic = tic + toc 
    print('smooth_normal: '..toc)
    
    collectgarbage()
    
    return crossprod
    
end

--[[]]
