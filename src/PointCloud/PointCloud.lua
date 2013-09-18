local io = require 'io'
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
local PC_ASCII_EXTENSION = '.xyz'

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
         if util.fs.extname(pcfilename)==PC_ASCII_EXTENSION then
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
            self.points = pts:type('torch.DoubleTensor'):div(10000.0)
            self.rgb = loaded[4]
            self.count = self.points:size(1)
            self.normal_map = loaded[5]
            if self.normal_map then 
               self.normal_map:type('torch.DoubleTensor'):div(10000.0)
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

   self:reset_point_stats()

   print("pass 2: count: "..self.count..", height: "..self.height..", width: "..self.width);
end

function PointCloud:write(filename)

   if util.fs.extname(filename)==PointCloud.PC_OD_EXTENSION then
      local pts = self.points:clone():mul(10000):type('torch.IntTensor')
      local nmp
      if self.normal_map then 
         nmp = self.normal_map:clone():mul(10000):type('torch.IntTensor')
      end
      torch.save(filename, {self.format, self.hwindices, pts, self.rgb, nmp,self.local_scan_center, self.local_to_global_pose, self.local_to_global_rotation})
   elseif util.fs.extname(filename)==PC_ASCII_EXTENSION then
      local file = io.open(filename, 'w');
      local tmpt = torch.range(1,self.count)
      tmpt:apply(function(i)
                    if self.format == 1 then
                       local ind = self.hwindices[i]:clone():add(-1)
                       file:write(''..(ind[1])..' '..(ind[2])..' ')
                    end
                    local pt = self.points[i]
                    local rgbx = self.rgb[i]
                    file:write(''..pt[1]..' '..pt[2]..' '..pt[3]..' '..rgbx[1]..' '..rgbx[2]..' '..rgbx[3]..'\n')
                 end)
      file:close()
   end
end

function PointCloud:flatten()
   self.flattenx = self.points:sub(1,self.count,2,3):clone()
   self.flattenz = self.points:sub(1,self.count,1,2):clone()

   self.flatteny = torch.zeros(2,self.count)
   self.flatteny[1] = self.points:transpose(1,2)[1]:clone()
   self.flatteny[2] = self.points:transpose(1,2)[3]:clone()
   self.flatteny = self.flatteny:transpose(1,2)

   collectgarbage()
end

function PointCloud:make_panoramic_image()
   if self.format == 1 then
      local img = torch.zeros(self.height,self.width,3);
      local rgb = self.rgb:clone()
      for i = 1,self.count do
         local ind = self.hwindices[i]
         img[ind[1]][ind[2]] = rgb[i]
      end
      collectgarbage()
      local pan_image = img:transpose(1,3):transpose(2,3);
      pan_image:div(pan_image:max());
      return pan_image
   else
      print("can't make panoramic image, no w/h info given")
   end
end

function PointCloud:make_panoramic_depth_map()
   if self.format == 1 then
      local img = torch.ones(self.height, self.width, 3)
      local norm_factor = self.radius:norm()
      local depthmp = self.centroid:clone():repeatTensor(self.count,1):add(self.points:clone():mul(-1)):pow(2):sum(2):sqrt():div(norm_factor):squeeze()
      for i = 1,self.count do
         local ind = self.hwindices[i]
         img[ind[1]][ind[2]] = torch.Tensor(3):fill(depthmp[i])
      end
      collectgarbage()
      local depth_map = torch.Tensor(img:size()):copy(img)
      depth_map:div(depth_map:max()):add(-1):abs():div(depth_map:max())
      depth_map = depth_map:transpose(1,3):transpose(2,3)
      return depth_map
   else
      print("can't make panoramic image, no w/h info given")
   end
end


local function connect_lines(img,x1,y1,x2,y2,height,width,dst)
	
	local minx = math.min(x1,x2)+1
	local maxx = math.max(x1,x2)
	local miny = math.min(y1,y2)+1
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
						img[yy][xx] = img[yy][xx] + dst*3
						if prev > 0 and yy ~= prev then
							img[prev][xx] = img[prev][xx] + dst*2
							img[yy][xx-1] = img[yy][xx-1] + dst*2
						end
					end
					prev = yy
				end)
		else
			img[y1][x1] = img[y1][x1] + dst
		end
	else
		local slp = dffx / dffy
		if miny < maxy then
			local prev = 0
			torch.range(miny, maxy):apply(
				function(yy)
					local xx = x2 + (yy-y2)*slp
					if xx <= width and yy <=height and xx >=1 and yy >=1 then
						img[yy][xx] = img[yy][xx] + dst*3
						if prev > 0 and xx ~= prev then
							img[yy][prev] = img[yy][prev] + dst*2
							img[yy-1][xx] = img[yy-1][xx] + dst*2
						end
					end
					prev = xx
				end)
		else
			img[y1][x1] = img[y1][x1] + dst
		end 
	end
end

function PointCloud:make_flattened_images(scale,mask,numCorners)

	scale = scale+0.000000001
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
		
		imagez=(imagez:div(imagez:max()+0.000001):mul(256)):floor()
		self.imagez = imagez:clone():resize(height,width):repeatTensor(3,1,1)
	else
		local connections,corners_map = self:find_connections_and_corners()
		if mask then
			connections:cmul(mask)
		end
		local image_corners = imagez:clone()
		
		local imgpts = self.xyz_map:clone()
		local hght = self.height
		local wdth = self.width
		local points = imgpts:clone():sub(1,hght,1,wdth,1,2)
		local coords = torch.Tensor(points:size()):copy(points)
		coords:add(minv:sub(1,2):repeatTensor(hght,wdth,1):mul(-1)):div(scale):floor():add(1)
		
		centerpt = self.centroid:squeeze():sub(1,2)
		
		local udind = torch.range(1,hght)
		local lrind = torch.range(1,wdth)
		
		local inline_tol = 0.25
		
		local points = imgpts:clone()
		
		local crdh = coords[1]
		local ptsh = points[1]
		local connh = connections[1]
		local cornh = corners_map[1]
		
		local crdph
		local ptsph
		local connph
		local cornph

		udind:apply(function(h)
						
						ptsph = ptsh
						connph = connh
						cornph = cornh
						
						crdh = coords[h]
						ptsh = points[h]
						connh = connections[h]
						cornh = corners_map[h]
						
						local ptszdiff = ptsph:clone():mul(-1):add(ptsh):select(2,3):abs()--:pow(2)
						
						local connhw = connh[wdth]
						local connphw
						local connhpw
						
						local cornhw
						local cornphw
						
						lrind:apply(function(w)
						
							cornhw = cornh[w]
							cornphw = cornph[w]
							
							if cornhw == 1 and cornphw == 1 then
								local dst = ptszdiff[w]	
								local crd = crdh[w]
								local y = crd[1]
								local x = crd[2]
								image_corners[y][x] = image_corners[y][x]+dst
							end
					
							connhpw = connhw
							connhw = connh[w]
							
							if connhpw == 1 then
							
								if connhw == 1 then
								
									connphw = connph[w]
									
									if connphw == 1 then
										
										local pw = w-1
										if pw ==0 then
											pw =wdth
										end
										
										local dst = ptszdiff[w]
									
										local crd = crdh[w]
										local y = crd[1]
										local x = crd[2]
									
										local xyzhw = ptsh[w]
										local xyzhpw = ptsh[pw]
									
										local cc = crdh[pw]
										local yc = cc[1]
										local xc = cc[2]
										
										if geom.util.normalize(xyzhw:sub(1,2) - xyzhpw:sub(1,2)):dist(
										   geom.util.normalize(xyzhw:sub(1,2) - self.centroid:squeeze():sub(1,2))) > inline_tol and
										   geom.util.normalize(xyzhpw:sub(1,2) - xyzhw:sub(1,2)):dist(
										   geom.util.normalize(xyzhw:sub(1,2) - self.centroid:squeeze():sub(1,2))) > inline_tol then								   
											connect_lines(imagez,x,y,xc,yc,height,width,dst)
										end
									end
								end
							end
						end)
					end)

		show_threshold = 0.025
		imagez:div(imagez:max()+0.000001):add(-show_threshold)
		imagez:add(imagez:clone():abs()):div(2)
		imagez:div(imagez:max()+0.000001)
		
		local tmp1 = imagez:clone():cdiv(imagez:clone():add(0.0000000001)):ceil()
		local kern = torch.Tensor({{ 1, 2, 3, 2, 1},
								   { 2, 9,16, 9, 2},
								   { 3,16,25,16, 3},
					     		   { 2, 9,16, 9, 2},
								   { 1, 2, 3, 2, 1}})
		
		local conv1 = torch.conv2(tmp1:clone(),kern,'F')
		local conv2 = torch.conv2(imagez:clone(),kern,'F')
		local conv3 = conv2:cdiv(conv1:add(0.0000000001)):clone():sub(3,height+2,3,width+2):cmul(tmp1)
		local conv4 = conv2:div(kern:sum()):clone():sub(3,height+2,3,width+2):cmul(tmp1)
		
		conv3:add(conv4)
		conv3:div(conv3:max()+0.000001)
		
		imagez:add(conv3:mul(3))
		
		imagez:div(imagez:max()+0.000001)
			
		self.imagez = imagez:clone():repeatTensor(3,1,1)
		
		image_corners:div(image_corners:max()+0.00000001)
		if numCorners and numCorners > 0 then
			local ps = math.ceil(0.10/scale)
			local bs = math.ceil(0.05/scale)
			local ds =  3
			local cc = 0
			local image_corners_orig = image_corners:clone()
		
			--local blank_corners = imagez:clone():repeatTensor(3,1,1)
		
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
					--[[blank_corners:sub(1,1,math.max(1,y-ds),
			                      math.min(height,y+ds),
			                      math.max(1,x-ds),
			                      math.min(width,x+ds)):add(1)]]
				else
					image_corners[y][x] = 0
				end
			end
		
			--blank_corners:cdiv(blank_corners:clone():add(0.0000001))
			--image.display(blank_corners)
		end
	end
	collectgarbage()
	return self.imagez,corners
	
end

function PointCloud:find_connections_and_corners()
	if self.format == 1 then
		self:make_normal_map()
		local pts = self.xyz_map:clone()
		
		local normal_map = self.normal_map:clone()
		local plane_const = torch.zeros(self.height,self.width)
		plane_const:add(pts:clone():cmul(normal_map):sum(3):squeeze())
		
		local plane_const_norm = normal_map:clone():pow(2):sum(3):squeeze():sqrt()
		
		local compare_plane_lr = torch.zeros(self.height,self.width)
		local compare_plane_ud = torch.zeros(self.height,self.width)
		
		local compare_normal_lr = torch.zeros(self.height,self.width)
		local compare_normal_ud = torch.zeros(self.height,self.width)
		
		compare_plane_lr:sub(1,self.height,2,self.width):add(
					plane_const:clone():sub(1,self.height,1,self.width-1):mul(-1):add(
					normal_map:clone():sub(1,self.height,1,self.width-1):cmul(
					pts:sub(1,self.height,2,self.width)):sum(3):squeeze()):abs():cdiv(
					plane_const_norm:sub(1,self.height,1,self.width-1)))
		compare_plane_lr:sub(1,self.height,1,1):add(
					plane_const:clone():sub(1,self.height,self.width,self.width):mul(-1):add(
					normal_map:clone():sub(1,self.height,self.width,self.width):cmul(
					pts:sub(1,self.height,1,1)):sum(3):squeeze()):abs():cdiv(
					plane_const_norm:sub(1,self.height,self.width,self.width)))
					
		compare_plane_ud:sub(2,self.height,1,self.width):add(
					plane_const:clone():sub(1,self.height-1,1,self.width):mul(-1):add(
					normal_map:clone():sub(1,self.height-1,1,self.width):cmul(
					pts:sub(2,self.height,1,self.width)):sum(3):squeeze()):abs():cdiv(
					plane_const_norm:sub(1,self.height-1,1,self.width)))
		compare_plane_ud:sub(1,1):add(compare_plane_ud:clone():sub(1,1))
		
		compare_normal_lr:sub(1,self.height,2,self.width):add(
					normal_map:clone():sub(1,self.height,2,self.width):mul(-1):add(
					normal_map:sub(1,self.height,1,self.width-1)):pow(2):sum(3):squeeze())
		compare_normal_lr:sub(1,self.height,1,1):add(
					normal_map:clone():sub(1,self.height,1,1):mul(-1):add(
					normal_map:sub(1,self.height,self.width,self.width)):pow(2):sum(3):squeeze())
		
		compare_normal_ud:sub(2,self.height,1,self.width):add(
					normal_map:clone():sub(2,self.height,1,self.width):mul(-1):add(
					normal_map:sub(1,self.height-1,1,self.width)):pow(2):sum(3):squeeze())
		compare_normal_ud:sub(1,1):add(compare_normal_ud:clone():sub(1,1))
		
		local nm_tol = 2.5
		local pc_tol = 0.05
		local z_tol = 0.25
		
		local tmp11 = compare_plane_lr:clone():add(-pc_tol):mul(-1)
		tmp11:add(tmp11:clone():abs())
		tmp11:cdiv(tmp11:clone():clone():add(0.000001))
		local tmp12 = compare_plane_ud:add(-pc_tol):mul(-1)
		tmp12:add(tmp12:clone():abs())
		tmp12:cdiv(tmp12:clone():add(0.000001))
		
		local tmp1 = tmp11:cmul(tmp12)
		tmp1:cdiv(tmp1:clone():add(0.00000001))
		
		local tmp21 = compare_normal_lr:clone():add(-nm_tol):mul(-1)
		tmp21:add(tmp21:clone():abs())
		tmp21:cdiv(tmp21:clone():add(0.000001))
		local tmp22 = compare_normal_ud:clone():add(-nm_tol):mul(-1)
		tmp22:add(tmp22:clone():abs())
		tmp22:cdiv(tmp22:clone():add(0.000001))
		
		local tmp2 = tmp21:cmul(tmp22)
		tmp2:cdiv(tmp2:clone():add(0.00000001))
		
		local tmp3 = normal_map:select(3,3):clone():abs():add(-z_tol):mul(-1)
		tmp3:add(tmp3:clone():abs())
		tmp3:cdiv(tmp3:clone():add(0.000001))
		
		local connections = tmp1:clone():cmul(tmp2):cmul(tmp3):ceil()
		
		local nm_tol = 0.05
		local pc_tol = 0.05
		local z_tol = 0.1
		
		local tmp11 = compare_plane_lr:clone():add(-pc_tol)
		tmp11:add(tmp11:clone():abs())
		tmp11:cdiv(tmp11:clone():add(0.000001))
		--[[local tmp12 = compare_plane_ud:clone():add(-pc_tol):mul(-1)
		tmp12:add(tmp12:clone():abs())
		tmp12:cdiv(tmp12:clone():add(0.000001))]]
		
		local tmp1 = tmp11--:cmul(tmp12)
		tmp1:cdiv(tmp1:clone():add(0.00000001))
		
		local tmp21 = compare_normal_lr:clone():add(-nm_tol)
		tmp21:add(tmp21:clone():abs())
		tmp21:cdiv(tmp21:clone():add(0.000001))
		--[[local tmp22 = compare_normal_ud:clone():add(-nm_tol):mul(-1)
		tmp22:add(tmp22:clone():abs())
		tmp22:cdiv(tmp22:clone():add(0.000001))]]
		
		local tmp2 = tmp21--:cmul(tmp22)
		tmp2:cdiv(tmp2:clone():add(0.00000001))
		
		local tmp3 = normal_map:select(3,3):clone():abs():add(-z_tol):mul(-1)
		tmp3:add(tmp3:clone():abs())
		tmp3:cdiv(tmp3:clone():add(0.000001))
		
		local corners = tmp1:add(tmp2):cdiv(tmp1:clone():add(0.0000001)):cmul(tmp3):ceil()
		
		return connections,corners

	end
end

function PointCloud:make_xyz_map(recompute)
   if (recompute or (not self.xyz_map)) and self.format == 1 then
      local img = torch.zeros(self.height, self.width, 3):add(math.huge)
      local tmp = torch.range(1,self.count)
      tmp:apply(function(i)
                   local ind = self.hwindices[i]
                   img[ind[1] ][ind[2] ] = self.points[i]
                end)
      self.xyz_map = img:clone()
   end
   return self.xyz_map
end

function PointCloud:make_normal_map()
	if self.format == 1 then
		if not (self.normal_map) then
			local height = self.height
			local width = self.width
			self:make_xyz_map(true)
			local img = self.xyz_map:clone()
	    
			local minus_lr = torch.zeros(height,width,3)
			local minus_ud = torch.zeros(height,width,3)
		
			minus_lr:sub(1,height,1,width-1):add(
	    			 img:sub(1,height,1,width-1):clone():mul(-1):add(
    				 img:sub(1,height,2,width)))
			minus_lr:sub(1,height,width,width):add(
		   			 img:sub(1,height,width,width):clone():mul(-1):add(
	    			 img:sub(1,height,1,1)))
	    	
			minus_ud:sub(1,height-1,1,width):add(
	    			 img:sub(1,height-1,1,width):clone():mul(-1):add(
					 img:sub(2,height,1,width)))
			minus_ud:sub(self.height,self.height,1,width):add(
	    			 img:sub(self.height-1,self.height-1,1,width):clone())
	    
		    local minus_lr_t = minus_lr:transpose(1,3)
		    local minus_ud_t = minus_ud:transpose(1,3)
	    
	    	local minus_lr_tx = minus_lr_t:select(1,1)--minus_lr_t[1]
		    local minus_lr_ty = minus_lr_t:select(1,2)--minus_lr_t[2]
		    local minus_lr_tz = minus_lr_t:select(1,3)--minus_lr_t[3]
	    	
		    local minus_ud_tx = minus_ud_t:select(1,1)--minus_ud_t[1]
		    local minus_ud_ty = minus_ud_t:select(1,2)--minus_ud_t[2]
	    	local minus_ud_tz = minus_ud_t:select(1,3)--minus_ud_t[3]
	  	
			crossprod = torch.Tensor(3,width,height)
	  	
			crossprod[1] = minus_lr_ty:clone():cmul(minus_ud_tz):add(
	  					   minus_lr_tz:clone():cmul(minus_ud_ty):mul(-1))
	    	crossprod[2] = minus_lr_tz:clone():cmul(minus_ud_tx):add(
		  				   minus_lr_tx:clone():cmul(minus_ud_tz):mul(-1))
			crossprod[3] = minus_lr_tx:clone():cmul(minus_ud_ty):add(
	  					   minus_lr_ty:clone():cmul(minus_ud_tx):mul(-1))
	  				   
		    local crossprodnorm = crossprod:clone():pow(2):sum(1):sqrt():squeeze():repeatTensor(3,1,1):add(0.0000000000000000001)
				
	    	crossprod = crossprod:clone():cdiv(crossprodnorm:clone()):transpose(1,3)
		
			self.normal_map=crossprod:clone()
		else
			self:make_xyz_map(true)
		end
	end
end

function PointCloud:make_panoramic_normal_map()
   if self.format == 1 then
      if not self.normal_map then
         self:make_normal_map()
      end

      local pmap = self.normal_map:clone()

      local pmap = pmap:add(1):div(2)

      pmap = pmap:transpose(1,3):transpose(2,3)

      return pmap
   else
      print("can't make panoramic image, no w/h info given")
   end
end

function PointCloud:downsample(leafsize)
   --leafsize is edge length of voxel
   scale = leafsize + 0.000000001
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
   coord2:mul(-1):add(coord)
   local neq = coord2:ne(torch.zeros(coord2:size())):sum(2):squeeze()
   local uniquecount = neq:sum()
   if uniquecount <= 0 then
      downsampled.count = 0
      return downsampled
   end
   local ptss = coord:clone():add(-1):mul(scale):add(self.minval:squeeze():repeatTensor(self.count,1))

   local points = torch.zeros(uniquecount,3)
   local rgb = torch.zeros(uniquecount,3):type('torch.ByteTensor')

   local tmp = torch.range(1,self.count)
   local count = 0
   tmp:apply(function(i)
                --if neq[i] == 1 then
                   local c = coord[i]
                   local c1 = c[1]
                   local c2 = c[2]
                   local c3 = c[3]
                   local tmpbin = bin[{c1,c2,c3}]
                   if tmpbin < 1 then
                      count = count + 1

                      bin[{c1,c2,c3}] = tmpbin + 1
                      points[count] = ptss[i]
                      rgb[count] = self.rgb[i]
                   end
                --end
             end)

   
   downsampled.points = points:sub(1,count)
   downsampled.rgb = rgb:sub(1,count)
   downsampled.count = count

   downsampled:reset_point_stats()
   collectgarbage()
   return downsampled
end

function PointCloud:make_3dtree()
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
   points = self:get_points()
   if (not self.index_map) or force then
      index_map = torch.LongTensor(self.height,self.width):fill(1)
      mask_map  = torch.ByteTensor(self.height,self.width):fill(1)
      -- make sure we get the reverse index
      for i = 1,self.count do
         if not self.hwindices then
            error("this pointcloud has no hwindices. can't make maps")
         end
         hw = self.hwindices[i]
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
      points      = self:get_points()
      index, mask = self:get_index_and_mask()

      self.xyz_map = util.addr.remap(points,index,mask)
   end

   return self.xyz_map

end

function PointCloud:get_depth_map()

   if not self.depth_map then
      xyz_map = self:get_xyz_map():clone()
      center = self.centroid:squeeze()
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
   rowid   = math.floor((degree_above / (degree_above + degree_below)) * self.height)
   xyz_map = self:get_xyz_map()
   midrow  = xyz_map[{3,rowid,{}}]
   return torch.Tensor({0,0,midrow[midrow:gt(0)]:mean()})
end


-- TODO make pose and rot getter and setter methods inherited from a View

-- store the position of the scan center in local coordiantes
function PointCloud:set_local_scan_center(pose)
   self.local_scan_center = pose
end
function PointCloud:get_global_scan_center()
   pose = self:get_local_to_global_pose()
   return self.local_scan_center + pose
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

local function saveHelper(points, rgb, fname)
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
   saveHelper(self:get_global_points(), self.rgb, fname)
end
function PointCloud:save_downsampled_to_xyz(leafsize, fname)
   local downsampled = self:downsample(leafsize)
   saveHelper(downsampled.points, downsampled.rgb, fname)
end
function PointCloud:save_downsampled_global_to_xyz(leafsize, fname)
   local downsampled = self:downsample(leafsize)
   local pose = self:get_local_to_global_pose()
   local rot  = self:get_local_to_global_rot()
   downsampled.points = rotate_translate(rot,pose,downsampled.points)
   saveHelper(downsampled.points, downsampled.rgb, fname)
end
