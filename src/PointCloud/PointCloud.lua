local io = require 'io'
local path = require 'path'
local kdtree = kdtree.kdtree
local ffi = require 'ffi'
local ctorch = util.ctorch -- ctorch needs to be loaded before we reference THTensor stuff in a cdef
local log = require '../util/log'
local rotate_translate = geom.quaternion.rotate_translate

-- angular to radians and back
local pi = math.pi
local r2d = 180/pi
local d2r = pi/180

local PointCloud = Class()

PointCloud.PC_OD_EXTENSION = '.od'
PointCloud.PC_ASCII_EXTENSION = '.xyz'
PointCloud.fudge_number = 10000
PointCloud.very_small_number = 0.00000001

function PointCloud:__init(pcfilename, radius, numstd, option)
   self.hwindices = nil;
   self.height = 0;
   self.width = 0;
   self.points = nil;
   self.rgb = nil;
   self.count = 0;
   self.centroid = torch.Tensor({{0,0,0}});
   if (not radius) then
      --default radius prune
      radius = 25
   end
   if (not numstd) then
      --default number of standard dev prune
      numstd = 3
   end

   if pcfilename then
      if util.fs.is_file(pcfilename) then
        print('loading '..path.basename(pcfilename))
         if util.fs.extname(pcfilename)==PointCloud.PC_ASCII_EXTENSION then
            self:set_pc_ascii_file(pcfilename, radius, numstd, option)
         elseif util.fs.extname(pcfilename)==PointCloud.PC_OD_EXTENSION then
            local loaded = torch.load(pcfilename)
            self.format = loaded[1]
            if self.format == 1 then
               self.hwindices = loaded[2]
               self.height = self.hwindices:max(1)[1][1]
               self.width = self.hwindices:max(1)[1][2]
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

function PointCloud:set_pc_ascii_file(pcfilename, radius, numstd)

   local rad2d = math.sqrt(math.pow(radius,2)*2/2.25)

   --first pass to see what type of file it is, whether it has 6 columns, or 8 (h/w first)
   local file = io.open(pcfilename, 'r');
   local line = file:read();
   if line == nil or line:len() < 5 then
      error("file did not have enough stuff in it")
   end
   file:close()
   -- on first pass determine format
   local countColumns = 0
   for token in string.gmatch(line, "[^%s]+") do
      countColumns = countColumns + 1
   end

   if countColumns == 6 then
   -- x y z r g b format
      self.format = 0
   elseif  (countColumns == 8) then
      self.format = 1
   else
      error("unknown format, input should have either 6 or 8 columns")
   end

   local totalLines = util.fs.exec("wc -l " .. pcfilename)
   for token in string.gmatch(totalLines, "[^%s]+") do
      count = tonumber(token)
      break
   end

   local file = torch.DiskFile(pcfilename, 'r', false)
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
   local r = xyzrgbTensor:select(2,4+offset)
   local g = xyzrgbTensor:select(2,5+offset)
   local b = xyzrgbTensor:select(2,6+offset)

   local indexGood = torch.lt((torch.cmul(x,x)+torch.cmul(y,y)):sqrt(), rad2d)
   if(self.format) then
      self.height = h[indexGood]:max()
      self.width = w[indexGood]:max()
   end
   self.points = torch.cat(torch.cat(x[indexGood],y[indexGood],2),z[indexGood],2)
   self.count = self.points:size(1)
   local meanz = z[indexGood]:sum()/(z[indexGood]:size(1)+.000001)
   self.centroid = torch.Tensor({{0, 0, meanz}})

   local stdrd = math.sqrt((self.points-self.centroid:repeatTensor(self.count,1)):pow(2):sum(2):mean())
   local perc = 0.75

   print("pass 1: count: "..self.count..", height: "..self.height..", width: "..self.width);
   print("radius: "..radius..", stdrd: "..(stdrd*numstd))

   if (perc*radius) > (stdrd * numstd) then
         --only run this if stdrd significantly smaller
         radius = stdrd * numstd
         print("make second pass with new radius: "..radius)

         local allXYZ = xyzrgbTensor[{{},{1+offset,3+offset}}]
         local distanceTensor = (allXYZ-self.centroid:repeatTensor(allXYZ:size(1),1)):pow(2):sum(2):sqrt()     
         indexGood = torch.lt(distanceTensor, radius)
         if(self.format) then
            self.height = h[indexGood]:max()
            self.width = w[indexGood]:max()
         end
         self.points = torch.cat(torch.cat(x[indexGood],y[indexGood],2),z[indexGood],2)
         self.count = self.points:size(1)
         local meanz = z[indexGood]:sum()/(z[indexGood]:size(1)+.000001)
         self.centroid = torch.Tensor({{0, 0, meanz}})
   end

   self.rgb = torch.cat(torch.cat(r[indexGood],g[indexGood],2),b[indexGood],2):byte()

   if self.format == 1 then
      self.hwindices = torch.cat(h[indexGood],w[indexGood],2):short()
   end
   collectgarbage()
   
   self.points = self.points:mul(PointCloud.fudge_number):floor():div(PointCloud.fudge_number)

   self:reset_point_stats()

   print("pass 2: count: "..self.count..", height: "..self.height..", width: "..self.width);
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
      --[[]]
      local pose = self:get_local_to_global_pose()
      local rot  = self:get_local_to_global_rotation()
      local points = rotate_translate(rot,pose,self.points)
      --[[]]
      --local points = self.points
      tmpt:apply(function(i)
                    local pt = points[i]
                    local rgbx = rgb[i]
                    if self.format == 1 then
                       local ind = self.hwindices[i]:clone()
                       local ih = ind[1]
                       local iw = ind[2]
                       --[[if self.normal_map then
                           local nmp = self.normal_map[ih][iw]
                           file:write(''..(ih-1)..' '..(iw-1)..' '..pt[1]..' '..pt[2]..' '..pt[3]..' '..rgbx[1]..' '..rgbx[2]..' '..rgbx[3]..' '..nmp[1]..' '..nmp[2]..' '..nmp[3]..'\n')
                       else]]
                           file:write(''..(ih-1)..' '..(iw-1)..' '..pt[1]..' '..pt[2]..' '..pt[3]..' '..rgbx[1]..' '..rgbx[2]..' '..rgbx[3]..'\n')
                       --end
                    else
                        file:write(''..pt[1]..' '..pt[2]..' '..pt[3]..' '..rgbx[1]..' '..rgbx[2]..' '..rgbx[3]..'\n')
                    end
                 end)
      file:close()
   end
end


function PointCloud:get_normal_image(recompute)
   if self.format == 1 then
     local pmap = self:get_normal_map(recompute):clone()
     local pmap = pmap:add(1):div(2)
     return pmap
   else
      print("can't make panoramic image, no w/h info given")
   end
end

local function connect_lines(img,x1,y1,x2,y2,height,width,dst)
    
    local minx = math.min(x1,x2)
    local maxx = math.max(x1,x2)
    local miny = math.min(y1,y2)
    local maxy = math.max(y1,y2)
    
    local dffx = x1-x2
    local dffy = y1-y2
    
    if math.abs(dffx) >= math.abs(dffy) then
        local slp = dffy / dffx
        if minx < maxx then
            local prev = 0
            torch.range(minx, maxx):apply(
                function(xx)
                    local yy = y2 + (xx-x2)*slp
                    if xx <= width and yy <=height and xx >=1 and yy >=1 then
                        img[yy][xx] = img[yy][xx] + dst
                        if prev > 0 and yy ~= prev then
                            img[prev][xx] = img[prev][xx] + dst/2
                            img[yy][xx-1] = img[yy][xx-1] + dst/2
                        end
                    end
                    prev = yy
                end)
        else
            img[y1][x1] = img[y1][x1] + dst/2
        end
    else
        local slp = dffx / dffy
        if miny < maxy then
            local prev = 0
            torch.range(miny, maxy):apply(
                function(yy)
                    local xx = x2 + (yy-y2)*slp
                    if xx <= width and yy <=height and xx >=1 and yy >=1 then
                        img[yy][xx] = img[yy][xx] + dst
                        if prev > 0 and xx ~= prev then
                            img[yy][prev] = img[yy][prev] + dst/2
                            img[yy-1][xx] = img[yy-1][xx] + dst/2
                        end
                    end
                    prev = xx
                end)
        else
            img[y1][x1] = img[y1][x1] + dst/2
        end 
    end
end

function PointCloud:get_flattened_images(scale,mask,numCorners)

    scale = scale+PointCloud.very_small_number
    local ranges = self.radius:clone():mul(2)
    local minv = self.radius:clone():mul(-1)
    local maxv = self.radius:clone()
    local pix = ranges:clone():div(scale):floor()
    local height = pix[1]+1
    local width = pix[2]+1
    local imagez = torch.zeros(height,width)
    local corners
    
    if numCorners and numCorners > 0 then
        corners = torch.zeros(numCorners,2)
    end
    
    if self.format == 0 then
        
        imagez = imagez:resize(height*width)
        --sort points
        local points = self.points:clone()[{{},{1,2}}]
    
        local coords = torch.Tensor(points:size()):copy(points)
        coords:add(minv:sub(1,2):repeatTensor(self.count,1):mul(-1)):div(scale):floor():add(1)
    
        local ys = torch.LongTensor(self.count):copy(coords[{{},1}])
        local xs = torch.LongTensor(self.count):copy(coords[{{},2}])
        local index = ys:add(-1):mul(width):add(xs)
        local dists = torch.Tensor(points:size()):copy(points)
        dists:add(self.centroid:squeeze():sub(1,2):repeatTensor(self.count,1):mul(-1)):pow(2)
        dists = dists:sum(2):squeeze()
    
        local i=1
        index:apply(function(x)
                        imagez[x]=imagez[x]+dists[i]
                        i=i+1
                        return x
                    end)
        
        imagez=imagez:pow(2)
        
        imagez=(imagez:div(imagez:max()+PointCloud.very_small_number):mul(256)):floor()
        imagez = imagez:clone():resize(height,width):repeatTensor(3,1,1)
    else
        local connections,corners_map = self:get_connections_and_corners()
        if mask then
            connections:cmul(mask)
        end
        local image_corners = imagez:clone()
        
        local imgpts = self:get_xyz_map_no_mask():transpose(1,2):transpose(2,3):clone()
        local hght = self.height
        local wdth = self.width
        local points = imgpts:clone():sub(1,hght,1,wdth,1,2)
        local coords = torch.Tensor(points:size()):copy(points)
        coords:add(minv:sub(1,2):repeatTensor(hght,wdth,1):mul(-1)):div(scale):floor():add(1)
        
        centerpt = self.centroid:squeeze():sub(1,2)
        
        local udind = torch.range(1,hght)
        local lrind = torch.range(1,wdth)
        
        local inline_tol = 0.25
        --local orig_tol = 0.1
        
        local points = imgpts:clone()
        
        local coords_f = coords:transpose(1,2)
        local points_f = points:transpose(1,2)
        local connec_f = connections:transpose(1,2)
        local corner_f = corners_map:transpose(1,2)
        
        local coords_cw = coords_f[wdth]
        local points_cw = points_f[wdth]
        local connec_cw = connec_f[wdth]
        local corner_cw = corner_f[wdth]
        
        local coords_pw
        local points_pw
        local connec_pw
        local corner_pw
        
        lrind:apply(function(w)
            coords_pw = coords_cw:clone()
            points_pw = points_cw:clone()
            connec_pw = connec_cw:clone()
            corner_pw = corner_cw:clone()
            
            coords_cw = coords_f[w]
            points_cw = points_f[w]
            connec_cw = connec_f[w]
            corner_cw = corner_f[w]
            
            local coords_cw_ch = coords_cw[1]
            local points_cw_ch = points_cw[1]
            local connec_cw_ch = connec_cw[1]
            local corner_cw_ch = corner_cw[1]
            
            local coords_cw_ph
            local points_cw_ph
            local connec_cw_ph
            local corner_cw_ph
            
            local dst_corn = 0
            local dst_conn = 0
            
            local do_connec_left = false
            local connec_left_coord = torch.ones(2)
            local connec_left_point = torch.zeros(2)
            
            udind:apply(function(h)
            
                coords_cw_ph = coords_cw_ch
                points_cw_ph = points_cw_ch
                connec_cw_ph = connec_cw_ch
                corner_cw_ph = corner_cw_ch
            
                coords_cw_ch = coords_cw[h]
                points_cw_ch = points_cw[h]
                connec_cw_ch = connec_cw[h]
                corner_cw_ch = corner_cw[h]
                
                local dst = math.abs(points_cw_ph[3]-points_cw_ch[3])
                
                if corner_cw_ph == 1 then
                    if corner_cw_ch == 1 then
                        dst_corn = dst_corn + dst
                        if h == hght or corner_cw[h+1] ~= 1 then
                            local y = coords_cw_ch[1]
                            local x = coords_cw_ch[2]
                            dst_corn = dst_corn
                            image_corners:sub(y,y,x,x):add(dst_corn)
                            dst_corn = 0
                        end
                    else
                        dst_corn = 0
                    end
                end
                
                if connec_cw_ph == 1 then
                    if connec_cw_ch == 1 then
                        dst_conn = dst_conn + dst
                        
                        if (not do_connec_left) and connec_pw[h] == 1 then
                            do_connec_left = true
                            connec_left_coord = coords_pw[h]
                            connec_left_point = points_pw[h]
                        end
                        if h == hght or connec_cw[h+1] ~= 1 then
                            local y1 = coords_cw_ch[1]
                            local x1 = coords_cw_ch[2]
                            dst_conn = dst_conn
                            if do_connec_left then
                                local y2 = connec_left_coord[1]
                                local x2 = connec_left_coord[2]
                                        
                                if geom.util.normalize(points_cw_ch:sub(1,2) - connec_left_point:sub(1,2)):dist(
                                   geom.util.normalize(points_cw_ch:sub(1,2) - self.centroid:squeeze():sub(1,2))) > inline_tol and
                                   geom.util.normalize(connec_left_point:sub(1,2) - points_cw_ch:sub(1,2)):dist(
                                   geom.util.normalize(points_cw_ch:sub(1,2) - self.centroid:squeeze():sub(1,2))) > inline_tol then                                
                                    connect_lines(imagez,x1,y1,x2,y2,height,width,dst_conn)
                                end
                            else
                                imagez:sub(y1,y1,x1,x1):add(dst_conn)
                            end

                            dst_conn = 0
                            do_connec_left = false
                            connec_left_coord = torch.ones(2)
                            connec_left_point = torch.zeros(2)
                        end
                    else
                        dst_conn = 0
                        if do_connec_left then
                            do_connec_left = false
                            connec_left_coord = torch.ones(2)
                            connec_left_point = torch.zeros(2)
                        end
                    end
                end
            end)
            collectgarbage()
        end)    
        
        local mean_height = (imagez:clone():sum())/(imagez:clone():cdiv(imagez:clone()+PointCloud.very_small_number):sum())
        local stdv_height = math.sqrt((imagez:clone():add(-mean_height):pow(2):sum())/(imagez:clone():cdiv(imagez:clone()+PointCloud.very_small_number):sum()))
        local max_height = mean_height+stdv_height
        local show_thresh = 0.01*max_height
        
        imagez = imagez:clone():gt(max_height):type('torch.DoubleTensor'):mul(max_height):add(
                 imagez:clone():le(max_height):type('torch.DoubleTensor'):cmul(imagez)):cmul(
                 imagez:clone():gt(show_thresh):type('torch.DoubleTensor'))
                 
        imagez:div(imagez:max()+PointCloud.very_small_number)
        
        image_corners:div(image_corners:max()+PointCloud.very_small_number)
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
         end
         imagez = imagez:clone():repeatTensor(3,1,1)
        
    end
    collectgarbage()
    return imagez,corners
    
end

function PointCloud:get_connections_and_corners()
    if self.format == 1 then
        self:get_normal_map(false)
        
        local pts = self:get_xyz_map_no_mask()
        local height = self.height
        local width = self.width
        local normal_map = self.normal_map:clone()
        
        local plane_const = torch.zeros(height,width)
        plane_const:add(pts:clone():cmul(normal_map):sum(1):squeeze())
        
        local plane_const_norm = normal_map:clone():pow(2):sum(1):squeeze():sqrt()
        
        local compare_plane_lr = torch.zeros(height,width)
        local compare_plane_ud = torch.zeros(height,width)
        
        local compare_normal_lr = torch.zeros(height,width)
        local compare_normal_ud = torch.zeros(height,width)
        
        compare_plane_lr:sub(1,height,2,width):add(
                    plane_const:clone():sub(1,height,1,width-1):mul(-1):add(
                    normal_map:clone():sub(1,3,1,height,1,width-1):cmul(
                    pts:sub(1,3,1,height,2,width)):sum(1):squeeze()):abs():cdiv(
                    plane_const_norm:sub(1,height,1,width-1)))
        compare_plane_lr:sub(1,height,1,1):add(
                    plane_const:clone():sub(1,height,width,width):mul(-1):add(
                    normal_map:clone():sub(1,3,1,height,width,width):cmul(
                    pts:sub(1,3,1,height,1,1)):sum(1):squeeze()):abs():cdiv(
                    plane_const_norm:sub(1,height,width,width)))
                    
        compare_plane_ud:sub(2,height,1,width):add(
                    plane_const:clone():sub(1,height-1,1,width):mul(-1):add(
                    normal_map:clone():sub(1,3,1,height-1,1,width):cmul(
                    pts:sub(1,3,2,height,1,width)):sum(1):squeeze()):abs():cdiv(
                    plane_const_norm:sub(1,height-1,1,width)))
        compare_plane_ud:sub(1,1):add(compare_plane_ud:clone():sub(1,1))
        
        compare_normal_lr:sub(1,height,2,width):add(
                    normal_map:clone():sub(1,3,1,height,2,width):mul(-1):add(
                    normal_map:sub(1,3,1,height,1,width-1)):pow(2):sum(1):squeeze())
        compare_normal_lr:sub(1,height,1,1):add(
                    normal_map:clone():sub(1,3,1,height,1,1):mul(-1):add(
                    normal_map:sub(1,3,1,height,width,width)):pow(2):sum(1):squeeze())
        
        compare_normal_ud:sub(2,height,1,width):add(
                    normal_map:clone():sub(1,3,2,height,1,width):mul(-1):add(
                    normal_map:sub(1,3,1,height-1,1,width)):pow(2):sum(1):squeeze())
        compare_normal_ud:sub(1,1):add(compare_normal_ud:clone():sub(1,1))
        
        local nm_tol1 = 2.5
        local pc_tol1 = 0.25
        local z_tol = 0.1
        
        local tmp3 = normal_map:select(1,3):clone():abs():lt(z_tol)
        local tmp4 = pts:clone():pow(2):sum(1):squeeze():gt(0)
        
        local connections = compare_plane_lr:lt(pc_tol1):cmul(compare_plane_ud:lt(pc_tol1)):cmul(
                            compare_normal_lr:lt(nm_tol1):cmul(compare_normal_ud:lt(nm_tol1))):cmul(tmp3):cmul(tmp4)
        
        local nm_tol2 = 0.1
        local pc_tol2 = 0.1
        
        local corners = compare_plane_lr:gt(pc_tol2):cmul(compare_plane_ud:lt(pc_tol1)):add(
                        compare_normal_lr:gt(nm_tol2):cmul(compare_normal_ud:lt(nm_tol1))):gt(0):cmul(tmp3):cmul(tmp4)
        
        return connections,corners

    end
end

function PointCloud:get_normal_map(recompute)
    if self.format == 1 then
        if (not self.normal_map) or recompute then
            local img = self:get_xyz_map_no_mask()
            
            local height = self.height
            local width = self.width
        
            local minus_lr = torch.zeros(3,height,width)
            local minus_ud = torch.zeros(3,height,width)
            minus_lr:sub(1,3,1,height,1,width-1):add(
                     img:sub(1,3,1,height,1,width-1):clone():mul(-1):add(
                     img:sub(1,3,1,height,2,width)))
            minus_lr:sub(1,3,1,height,width,width):add(
                     img:sub(1,3,1,height,width,width):clone():mul(-1):add(
                     img:sub(1,3,1,height,1,1)))
            
            minus_ud:sub(1,3,1,height-1,1,width):add(
                     img:sub(1,3,1,height-1,1,width):clone():mul(-1):add(
                     img:sub(1,3,2,height,1,width)))
            minus_ud:sub(1,3,height,height,1,width):add(
                     img:sub(1,3,height-1,height-1,1,width):clone())
        
            local minus_lr_tx = minus_lr:select(1,1)
            local minus_lr_ty = minus_lr:select(1,2)
            local minus_lr_tz = minus_lr:select(1,3)
            
            local minus_ud_tx = minus_ud:select(1,1)
            local minus_ud_ty = minus_ud:select(1,2)
            local minus_ud_tz = minus_ud:select(1,3)
        
            local crossprod = torch.Tensor(3,height,width)
        
            crossprod[1] = minus_lr_ty:clone():cmul(minus_ud_tz):add(
                           minus_lr_tz:clone():cmul(minus_ud_ty):mul(-1))
            crossprod[2] = minus_lr_tz:clone():cmul(minus_ud_tx):add(
                           minus_lr_tx:clone():cmul(minus_ud_tz):mul(-1))
            crossprod[3] = minus_lr_tx:clone():cmul(minus_ud_ty):add(
                           minus_lr_ty:clone():cmul(minus_ud_tx):mul(-1))
                       
            local crossprodnorm = crossprod:clone():pow(2):sum(1):sqrt():squeeze():repeatTensor(3,1,1):add(PointCloud.very_small_number)
                
            crossprod = crossprod:clone():cdiv(crossprodnorm:clone())
            
            crossprod:mul(PointCloud.fudge_number):floor():div(PointCloud.fudge_number)
            
            self.normal_map=crossprod:clone()
            
        else
            print('normal map exists')
            self:get_xyz_map_no_mask()
        end
        return self.normal_map
    end
end

function PointCloud:downsample_map(leafsize)
   --leafsize is edge length of voxel
   scale = leafsize + PointCloud.very_small_number
   local ranges = self.maxval:clone():squeeze():add(self.minval:clone():squeeze():mul(-1))
   local pix = ranges:clone():div(scale):floor():add(1)
   local tmp = torch.range(1,self.count)
   local downsampled = PointCloud.new()
   downsampled.height = 0;
   downsampled.width = 0;

   local bin = torch.zeros(pix[1], pix[2], pix[3])
   local coord = self.points:clone():add(self.minval:clone():squeeze():mul(-1):repeatTensor(self.count,1)):div(scale):floor():add(1)
   local coord2 = torch.zeros(coord:size())
   coord2:sub(2,self.count):copy(coord:sub(1,self.count-1))
   coord2 = coord2:mul(-1):add(coord):pow(2):sum(2)
   local neq = coord2:ne(0):squeeze()
   local uniquecount = neq:sum()
   
   if uniquecount <= 0 then
      downsampled.count = 0
      return downsampled
   end
   
   print('uniquecount: '..uniquecount..', total points: '..self.count)
   
   local ptss = coord:clone():add(-1):mul(scale):add(self.minval:squeeze():repeatTensor(self.count,1))

   local points = torch.zeros(uniquecount,3)
   local rgb = torch.zeros(uniquecount,3):type('torch.ByteTensor')
   
   coord = coord:type('torch.LongTensor')

   local count = 0
   
   uniquecount = nil
   --neq = nil
   coord2 = nil
   ranges = nil
   pix = nil
   
   tmp:apply(function(i)
   				   if neq[i] > 0 then
	                   local c = coord[i]:clone()
	                   if bin[c:storage()] < 1 then
    	                  count = count + 1
    	                  local c1 = c[1]
        	           	  local c2 = c[2]
            	          local c3 = c[3]
                	      bin:sub(c1,c1,c2,c2,c3,c3):add(1)
            	          points:sub(count,count):add(ptss[i])
                	      rgb:sub(count,count):add(self.rgb[i])
                	      if count % 25000 == 0 then
	                    	  print('curr count: '..count..' processed: '..i)
	                      end
	                   end
    	               --[[
        	           c = nil
            	       c1 = nil
                	   c2 = nil
	                   c3 = nil
    	               tmpbin = nil
        	           ]]
        	        end
                   collectgarbage()
             end)
    print('done downsample loop')
    collectgarbage()
   
   downsampled.points = points:sub(1,count)
   downsampled.rgb = rgb:sub(1,count)
   downsampled.count = count
   downsampled:set_local_to_global_pose(self:get_local_to_global_pose())
   downsampled:set_local_to_global_rotation(self:get_local_to_global_rotation())

   downsampled:reset_point_stats()
   collectgarbage()
   return downsampled
end

function PointCloud:downsample(leafsize,fnamexyz,fnameobj)
   --leafsize is edge length of voxel
   
   --[[]]
   local pose = self:get_local_to_global_pose()
   local rot  = self:get_local_to_global_rotation()
   self.points = rotate_translate(rot,pose,self.points)
   --[[]]
   
   print('downsample: points rotated')
    
   scale = leafsize + PointCloud.very_small_number
   local ranges = self.maxval:clone():squeeze():add(self.minval:clone():squeeze():mul(-1))
   local nmp = self:get_normal_map(true):clone()
   local pts = self:get_xyz_map_no_mask():clone()
   local crd = pts:clone():add(self.minval:clone():mul(-1):squeeze():repeatTensor(self.width,self.height,1):transpose(1,3)):div(scale):floor():add(1)
   local rgb = self:get_rgb_map_no_mask(true):type('torch.ByteTensor')
   
   print('normals,pts,crd,rgb found')
   
   pts = crd:clone():add(-1):mul(scale):add(self.minval:squeeze():repeatTensor(self.height,self.width,1):transpose(1,3))
   
   nmp = nmp:transpose(1,2):transpose(2,3)
   pts = pts:transpose(1,2):transpose(2,3)
   crd = crd:transpose(1,2):transpose(2,3)
   rgb = rgb:transpose(1,2):transpose(2,3)
   
   local downsampled = PointCloud.new()
   downsampled.height = 0;
   downsampled.width = 0;

   local points = torch.zeros(self.count,3)
   local dsrgb = torch.zeros(self.count,3):type('torch.ByteTensor')
   local normals = torch.zeros(self.count,3)

   local count = 0
   
   local c_h = torch.Tensor(self.width,3):fill(math.huge)
   local p_h = torch.Tensor(self.width,3):fill(math.huge)
   local n_h = torch.Tensor(self.width,3):fill(math.huge)
   
   print('start loop')
   
   torch.range(1,self.height):apply(function(h)

     local c_ph = c_h
     
     c_h = crd[h]
     p_h = pts[h]
     n_h = nmp[h]
     r_h = rgb[h]
     
     local c_h_w = torch.Tensor(3):fill(math.huge)
     local c_ph_w = torch.Tensor(3):fill(math.huge)
     
     torch.range(1,self.width):apply(function(w)
       
       local c_h_pw = c_h_w
       local c_ph_pw = c_ph_w
       c_ph_w = c_ph[w]:clone()
       c_h_w = c_h[w]:clone()
       
       if c_h_w:sum() ~= math.huge then
         if torch.dist(c_h_w,c_ph_pw) > 0 and torch.dist(c_h_w,c_ph_w) > 0 and torch.dist(c_h_w,c_h_pw) > 0 then

           count = count + 1
           
           points:sub(count,count):add(p_h[w])
           dsrgb:sub(count,count):add(r_h[w])
           normals:sub(count,count):add(n_h[w])
           
           if count % 100000 == 0 then
	         print('curr count: '..count..' processed: '..h..','..w)
	       end
	     end
	   else
	     if c_ph_pw:sum() ~= math.huge then
	       crd[h][w] = c_ph_pw
	     elseif c_ph_w:sum() ~= math.huge then
	       crd[h][w] = c_ph_w
	     elseif c_h_pw:sum() ~= math.huge then
	       crd[h][w] = c_h_pw
	     end
	   end
	   
	 end)
   end)
   collectgarbage()
         
   downsampled.points = points:sub(1,count)
   downsampled.rgb = dsrgb:sub(1,count)
   downsampled.normals = normals:sub(1,count)
   downsampled.count = count
   downsampled:set_local_to_global_pose(self:get_local_to_global_pose())
   downsampled:set_local_to_global_rotation(self:get_local_to_global_rotation())

   downsampled:reset_point_stats()
   collectgarbage()
   print('end loop')
   
   if fnamexyz then
     local filexyz = io.open(fnamexyz, 'w')
     local bnamexyz = path.basename(fnamexyz)
     print('create xyz file '..bnamexyz)
     for i = 1,count do
       local nmp = normals[i]
	   local pt = points[i]
	   local rgbx = dsrgb[i]
	     filexyz:write(pt[1]..' '..pt[2]..' '..pt[3]..' '..rgbx[1]..' '..rgbx[2]..' '..rgbx[3]..--[[' '..nmp[1]..' '..nmp[2]..' '..nmp[3]..]]'\n')
	   end
    end
    filexyz:close()
    
   end
   collectgarbage()
   
   if fnameobj then
     local fileobj = io.open(fnameobj, 'w')
     local bnameobj = path.basename(fnameobj)
     print('create obj file '..bnameobj)
     
     fileobj:write('####\n')
     fileobj:write('#\n')
     fileobj:write('# OBJ File Generated by Cloudlab\n')
     fileobj:write('####\n')
     fileobj:write('# Object '..bnameobj..'\n')
     fileobj:write('#\n')
     fileobj:write('# Vertices: '..count..'\n')
     fileobj:write('# Faces: 0\n')
     fileobj:write('#\n')
     fileobj:write('####\n')
     
     local rgb = dsrgb:type('torch.DoubleTensor'):div(255)
     
     for i = 1,count do
       local pt = points[i]
       local rgbx = rgb[i]
       local nmp = normals[i]
       fileobj:write('vn '..nmp[1]..' '..nmp[2]..' '..nmp[3]..'\n')
       fileobj:write('v '..pt[1]..' '..pt[2]..' '..pt[3]..' '..rgbx[1]..' '..rgbx[2]..' '..rgbx[3]..'\n')
    end
     
     fileobj:write('# '..count..' vertices, '..count..' vertices normals\n\n')
     fileobj:write('# 0 faces, 0 coords texture\n\n')
     fileobj:write('# End of File')
     fileobj:close()
   
   end
   collectgarbage()
   
   return downsampled
end

function PointCloud:get_3dtree()
   self.k3dtree = kdtree.new(self.points)
   collectgarbage()
end

-- I made these [MS]
function PointCloud:get_points(force)
   if (not self.pointsT) or force then
      -- we should do index like this and store data the opposite way around
      self.pointsT = self.points:transpose(1,2):contiguous()
   end
   return self.pointsT
end

function PointCloud:get_index_and_mask(force)
   local points = self:get_points()
   if (not self.index_map) or force then
      local index_map = torch.LongTensor(self.height,self.width):fill(1)
      local mask_map  = torch.ByteTensor(self.height,self.width):fill(1)
      -- make sure we get the reverse index
      for i = 1,self.count do
         if not self.hwindices then
            error("this pointcloud has no hwindices. can't make maps")
         end
         local hw = self.hwindices[i]
         index_map[{hw[1],hw[2]}] = i
         mask_map[{hw[1],hw[2]}] = 0
      end
      self.index_map = index_map
      self.mask_map  = mask_map
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
        
        torch.range(1,self.height):apply(function(h)
            local d = dpth[h]
            torch.range(1,self.width):apply(function(w)
                if d[w] == 0 then
                    self.xyz_map_no_mask:sub(1,3,h,h,w,w):fill(math.huge)
                end
            end)
        end)
    end
    return self.xyz_map_no_mask
end

function PointCloud:get_depth_image()

    local xyz_map = self:get_xyz_map_no_mask():clone()
    local center = self.centroid:squeeze()
    for i = 1,3 do
      xyz_map[i]:add(-center[i])
    end
    local depth_map = xyz_map:norm(2,1):squeeze()
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

function PointCloud:get_max_radius()
   return torch.max(self.radius)
end

local function saveHelper_xyz(points, rgb, fname)
   local file = io.open(fname, 'w')
   local tmpt = torch.range(1,points:size(1))                                                                                                                                  
   tmpt:apply(function(i) 
      pt = points[i] 
      rgbx = rgb[i]
      file:write(''..pt[1]..' '..pt[2]..' '..pt[3]..' '..rgbx[1]..' '..rgbx[2]..' '..rgbx[3]..'\n')
   end)
   file:close()
end

function PointCloud:save_global_points_to_xyz(fname)
   saveHelper_xyz(self:get_global_points(), self.rgb, fname)
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
         rgbT = self.rgb:transpose(1,2):contiguous()
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
   if (not self.rgb_map) or force then  
      if self.rgb then
         rgbT = self.rgb:transpose(1,2):contiguous()
         index, mask = self:get_index_and_mask()
         self.rgb_map = util.addr.remap(rgbT,index,mask,torch.Tensor(3,self.height,self.width))
      else
         print("no rgb_map and no rgb values")
         return
      end
   end
   return self.rgb_map
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
