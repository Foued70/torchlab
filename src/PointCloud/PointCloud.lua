local io = require 'io'
local kdtree = kdtree.kdtree
local ffi = require 'ffi'
local ctorch = util.ctorch -- ctorch needs to be loaded before we reference THTensor stuff in a cdef

-- angular to radians and back
local pi = math.pi
local r2d = 180/pi
local d2r = pi/180

local PointCloud = Class()

local PC_OD_EXTENSION = '.od'
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
	    elseif util.fs.extname(pcfilename)==PC_OD_EXTENSION then
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
	self.minval,self.minind = self.points:min(1)
    self.maxval,self.maxind = self.points:max(1)
    local d = torch.Tensor(2,3)
    d[1] = self.maxval:clone():add(self.centroid:clone():mul(-1)):squeeze()
    d[2] = self.centroid:clone():add(self.minval:clone():mul(-1)):squeeze()
    self.radius = d:max(1):squeeze()
    d = nil
    collectgarbage()
    self.count = self.points:size(1)
end

function PointCloud:set_pc_ascii_file(pcfilename, radius, numstd)
	
    local file = io.open(pcfilename, 'r');
    	
    local count = 0;
    self.height = 0;
    self.width = 0;
    local meanx = 0
	local meany = 0
	local meanz = 0
	local hw_table={}
	local xyz_table={}
	local rgb_table={}
	-- assume z is flatter
	local rad2d = math.sqrt(math.pow(radius,2)*2/2.25)
		
    while true do
    	if (not self.format) then
    		self.format = 1
        	local line = file:read();
			if line == nil or line:len() < 5 then 
       			break 
	    	end
	    	-- on first pass determine format
   			local begp = 1;
   			local endp = line:find(' ', begp) - 1;
	      	begp = endp + 2;
    		endp = line:find(' ', begp) - 1;
		    begp = endp + 2;
    		endp = line:find(' ', begp) - 1;
		    begp = endp + 2;
    		endp = line:find(' ', begp) - 1;
		    begp = endp + 2;
    		endp = line:find(' ', begp) - 1;
		    begp = endp + 2;
    		endp = line:find(' ', begp);
		    if not endp then
    			-- x y z r g b format
		    	self.format = 0
    		end
    		file:close()
    		file = io.open(pcfilename, 'r');
    	else
    		local h,w,x,y,z,r,g,b
			if self.format==1 then
	    		h,w = file:read('*number', '*number')
	    		if h==nil then
	    			break
	    		end
	    		h=h+1
	    		w=w+1
	    	end
	   	  	x,y,z,r,g,b = file:read('*number', '*number','*number', '*number','*number', '*number')
  			if x == nil then
  				break
  			end
  			
  			if math.sqrt(math.pow(x,2)+math.pow(y,2)) < rad2d then
  				count = count + 1
   	  			meanz = meanz + z
				table.insert(xyz_table,{x,y,z})
				table.insert(rgb_table,{r,g,b})
    	  		if self.format == 1 then
   	  				if h > self.height then
    	    			self.height = h;
		      		end
   			  		if w > self.width then
       					self.width = w;
      				end
      				table.insert(hw_table,{h,w})
      			end
   			end
		end
   	end
   
   	file:close()
   	collectgarbage()
  
    self.points = torch.Tensor(xyz_table)
	self.count = count;
	self.centroid = torch.Tensor({{meanx, meany, meanz}}):div(count+0.000001)
	local stdrd = math.sqrt((self.points-self.centroid:repeatTensor(self.count,1)):pow(2):sum(2):mean())
	local perc = 0.75
	
	print("pass 1: count: "..self.count..", height: "..self.height..", width: "..self.width);
	print("radius: "..radius..", stdrd: "..(stdrd*numstd))
	
	if (perc*radius) > (stdrd * numstd) then
		--only run this if stdrd significantly smaller
		radius = stdrd * numstd
		print("make second pass with new radius: "..radius)
	
		count = 0
		hw_table={}
		xyz_table={}
		rgb_table={}
		meanx = 0
		meany = 0
  		meanz = 0
		
		file = io.open(pcfilename, 'r');
	
	    while true do
    		local h,w,x,y,z,r,g,b
			if self.format==1 then
	    		h,w = file:read('*number', '*number')
	    		if h==nil then
	    			break
	    		end
	    		h=h+1
	    		w=w+1
	    	end
	   	  	x,y,z,r,g,b = file:read('*number', '*number','*number', '*number','*number', '*number')
  			if x == nil then
  				break
  			end
	  			
  			if self.centroid[1]:dist(torch.Tensor({x,y,z})) < radius then
  				count = count + 1
   	  			meanz = meanz + z
				table.insert(xyz_table,{x,y,z})
				table.insert(rgb_table,{r,g,b})
    	  		if self.format == 1 then
      				table.insert(hw_table,{h,w})
      			end
   			end
		end
		file:close()
		collectgarbage()
			
		self.points = torch.Tensor(xyz_table)
		self.count = count;
		self.centroid = torch.Tensor({{meanx, meany, meanz}}):div(count+0.000001)
   	end
   	collectgarbage()
   	
   	xyz_table = nil
    	
   	self.rgb = torch.ByteTensor(rgb_table)
   	
   	rgb_table = nil
   	
    if self.format == 1 then
    	self.hwindices = torch.ShortTensor(hw_table)
    	hw_table = nil
	end
	collectgarbage()

   	self:reset_point_stats()
    	
   	print("pass 2: count: "..self.count..", height: "..self.height..", width: "..self.width);
    
end

function PointCloud:write(filename)
	
	if util.fs.extname(filename)==PC_OD_EXTENSION then
		local pts = self.points:clone():mul(10000):type('torch.IntTensor')
		torch.save(filename, {self.format, self.hwindices, pts, self.rgb, self.normal_map})
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

--[[
function PointCloud:make_flattened_images(scale)

	scale = scale+0.000000001
	local ranges = self.radius:clone():mul(2)
	local minv = self.radius:clone():mul(-1)
	local maxv = self.radius:clone()
	local pix = ranges:clone():div(scale):floor()
	local height = pix[1]+1
	local width = pix[2]+1
	local imagez = torch.zeros(height,width)
	
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
		local imgpts, minus_lr, minus_ud = self:make_normal_map()
		local nmp = self.normal_map
		local hght = self.height
		local wdth = self.width
		local points = imgpts:sub(1,hght,1,wdth,1,2)
		local coords = torch.Tensor(points:size()):copy(points)
		coords:add(minv:sub(1,2):repeatTensor(hght,wdth,1):mul(-1)):div(scale):floor():add(1)
		
		centerpt = self.centroid:squeeze():sub(1,2)
		
		local udind = torch.range(1,hght)
		local lrind = torch.range(1,wdth)
		
		local norm_z_tolerance = 0.025
		local norm_dist_tolerance = 0.25
		local linear_slope_tolerance = 0.1
		local in_line_of_scanner_tolerance = 0.25
		
		udind:apply(function(h)
						local nmph = nmp[h]
						local crdh = coords[h]
						local ptsh = points[h]
						--local dsth = dists[h]
						lrind:apply(function(w)
							-- add points[h][w]
							local crd = crdh[w]
							local y = crd[1]
							local x = crd[2]
							
							norm = nmph[w]
							
							if y < math.huge and x < math.huge and math.abs(norm[3]) < norm_z_tolerance then
								--imagez[y][x] = imagez[y][x]+dst
								
								local prevw = w-1
								local nextw = w+1
								if prevw == 0 then
									prevw = wdth
								end
								if nextw == wdth + 1 then
									nextw = 1
								end
								
								normp = nmph[prevw]
								normn = nmph[nextw]
								
								if norm:dist(normp) < norm_dist_tolerance then
									local double_flag = false
									if norm:dist(normn) > norm_dist_tolerance then
										double_flag = true
									end
										
									-- check colinearity of 3 pts in xy and to origin
									-- connect points[h][w+1] to points[h][w-1]
									local prevcrd = crdh[prevw]
									local prevy = prevcrd[1]
									local prevx = prevcrd[2]
									
									local nextcrd = crdh[nextw]
									local nexty = nextcrd[1]
									local nextx = nextcrd[2]
									
									local ptw = ptsh[w]
									local ptp = ptsh[prevw]
									local ptn = ptsh[nextw]
									
									local w_to_p = geom.util.normalize(ptw-ptp)
									local n_to_w = geom.util.normalize(ptn-ptw)
									local n_to_p = geom.util.normalize(ptn-ptp)
									local c_to_w = geom.util.normalize(centerpt-ptw):abs()
									
									if w_to_p:dist(n_to_w) < linear_slope_tolerance and 
									   n_to_p:dist(n_to_w) < linear_slope_tolerance and
									   n_to_p:dist(w_to_p) < linear_slope_tolerance and
									   c_to_w:dist(n_to_w:clone():abs()) > in_line_of_scanner_tolerance then
									
										--local dst = dsth[w]
										local dst = 1
										local minx = math.min(x,prevx)
										local maxx = math.max(x,prevx)
										local miny = math.min(y,prevy)
										local maxy = math.max(y,prevy)
										local dffx = x-prevx
										local dffy = y-prevy
											
										if double_flag then
											minx = math.min(nextx,prevx)
											maxx = math.max(nextx,prevx)
											miny = math.min(nexty,prevy)
											maxy = math.max(nexty,prevy)
											dffx = nextx-prevx
											dffy = nexty-prevy
										end
									
										if math.abs(dffx) >= math.abs(dffy) then
											local slp = dffy / dffx
											if minx < maxx then
												torch.range(minx, maxx):apply(
													function(xx)
														if not (xx == x) then
															local yy = prevy + (xx-prevx)*slp
															if xx <= width and yy <=height and xx >=1 and yy >=1 then
																imagez[yy][xx] = imagez[yy][xx] + dst
															end
														end
													end)
											end
										else
											local slp = dffx / dffy
											if miny < maxy then
												torch.range(miny, maxy):apply(
													function(yy)
														if not (yy == y) then
															local xx = prevx + (yy-prevy)*slp
															if xx <= width and yy <=height and xx >=1 and yy >=1 then
																imagez[yy][xx] = imagez[yy][xx] + dst
															end
														end
													end)
											end
										end
									end
								end
							end
						end)
					end)
		
		
		--imagez:pow(2)
		show_threshold = 0.02
		imagez:div(imagez:max()+0.000001):add(-show_threshold)
		imagez:add(imagez:clone():abs()):div(2)
		imagez:div(imagez:max()+0.000001)
						
		--imagez=imagez:mul(256):floor()
			
		self.imagez = imagez:clone():repeatTensor(3,1,1)	

	end

	collectgarbage()
	return self.imagez
	
end
]]

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
	local maxx = math.max(x1,x2)-1
	local miny = math.min(y1,y2)+1
	local maxy = math.max(y1,y2)-1
	
	local dffx = x1-x2
	local dffy = y1-y2
	
	if math.abs(dffx) >= math.abs(dffy) then
		local slp = dffy / dffx
		if minx < maxx then
			torch.range(minx, maxx):apply(
				function(xx)
					local yy = y2 + (xx-x2)*slp
					if xx <= width and yy <=height and xx >=1 and yy >=1 then
						img[yy][xx] = img[yy][xx] + dst
						--img[yy][xx] = dst
					end
				end)
		else
			img[y1][x1] = img[y1][x1] + dst
		end
	else
		local slp = dffx / dffy
		if miny < maxy then
			torch.range(miny, maxy):apply(
				function(yy)
					local xx = x2 + (yy-y2)*slp
					if xx <= width and yy <=height and xx >=1 and yy >=1 then
						img[yy][xx] = img[yy][xx] + dst
						--img[yy][xx] = dst
					end
				end)
		else
			img[y1][x1] = img[y1][x1] + dst
		end
	end
end

function PointCloud:make_flattened_images(scale)

	scale = scale+0.000000001
	local ranges = self.radius:clone():mul(2)
	local minv = self.radius:clone():mul(-1)
	local maxv = self.radius:clone()
	local pix = ranges:clone():div(scale):floor()
	local height = pix[1]+1
	local width = pix[2]+1
	local imagez = torch.zeros(height,width)
	
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
		local edges, imgpts = self:find_edges()
		local hght = self.height
		local wdth = self.width
		local points = imgpts:sub(1,hght,1,wdth,1,2)
		local coords = torch.Tensor(points:size()):copy(points)
		coords:add(minv:sub(1,2):repeatTensor(hght,wdth,1):mul(-1)):div(scale):floor():add(1)
		
		local dists = torch.Tensor(points:size()):copy(points)
		dists:add(self.centroid:squeeze():sub(1,2):repeatTensor(self.height,self.width,1):mul(-1)):pow(2)
		dists = dists:sum(3):squeeze()
		
		centerpt = self.centroid:squeeze():sub(1,2)
		
		local udind = torch.range(1,hght)
		local lrind = torch.range(1,wdth)
		--local dst = 1
		
		local inline_tol = 0.1
		
		udind:apply(function(h)
						local crdh = coords[h]
						local ptsh = points[h]
						local edgh = edges[h]
						local dsth = dists[h]
						local ph = h-1
						if ph == 0 then
							ph = 1
						end
						local crdph = coords[ph]
						local ptsph = points[ph]
						local edgph = edges[ph]
						lrind:apply(function(w)
							-- add points[h][w]
							local crd = crdh[w]
							local y = crd[1]
							local x = crd[2]
							local edgehw = edgh[w]
							
							if edgehw == 1 then
								
								local pw = w-1
								if pw == 0 then
									pw = wdth
								end
								
								local edgephw = edgph[w]
								local edgehpw = edgh[pw]
								local edgephpw = edgph[pw]
								
								local dst = dsth[w]
								
								--imagez[y][x] = imagez[y][x] + dst
								
								local xyz = ptsh[w]
								local xyzphw = ptsph[w]
								local xyzhpw = ptsh[pw]
								local xyzphpw = ptsph[pw]
								
								if edgephw == 1 then
									local cc = crdph[w]
									local yc = cc[1]
									local xc = cc[2]
									local xyzc = ptsph[w]
									if geom.util.normalize(xyz:sub(1,2) - xyzc:sub(1,2)):dist(
									   geom.util.normalize(xyz:sub(1,2) - self.centroid:squeeze():sub(1,2))) > inline_tol and
									   geom.util.normalize(xyzc:sub(1,2) - xyz:sub(1,2)):dist(
									   geom.util.normalize(xyz:sub(1,2) - self.centroid:squeeze():sub(1,2))) > inline_tol then								   
										connect_lines(imagez,x,y,xc,yc,height,width,dst)
									end
								end
								
								if edgehpw == 1 then
									local cc = crdh[pw]
									local yc = cc[1]
									local xc = cc[2]
									local xyzc = ptsh[pw]
									if geom.util.normalize(xyz:sub(1,2) - xyzc:sub(1,2)):dist(
									   geom.util.normalize(xyz:sub(1,2) - self.centroid:squeeze():sub(1,2))) > inline_tol and
									   geom.util.normalize(xyzc:sub(1,2) - xyz:sub(1,2)):dist(
									   geom.util.normalize(xyz:sub(1,2) - self.centroid:squeeze():sub(1,2))) > inline_tol then								   
										connect_lines(imagez,x,y,xc,yc,height,width,dst)
									end
								end
								
								if edgephpw == 1 then
									local cc = crdph[pw]
									local yc = cc[1]
									local xc = cc[2]
									local xyzc = ptsph[pw]
									if geom.util.normalize(xyz:sub(1,2) - xyzc:sub(1,2)):dist(
									   geom.util.normalize(xyz:sub(1,2) - self.centroid:squeeze():sub(1,2))) > inline_tol and
									   geom.util.normalize(xyzc:sub(1,2) - xyz:sub(1,2)):dist(
									   geom.util.normalize(xyz:sub(1,2) - self.centroid:squeeze():sub(1,2))) > inline_tol then								   
										connect_lines(imagez,x,y,xc,yc,height,width,dst)
									end
								end
								--[[]]
							end
						end)
					end)
		
		
		--imagez:pow(2)
		show_threshold = 0.01
		imagez:div(imagez:max()+0.000001):add(-show_threshold)
		imagez:add(imagez:clone():abs()):div(2)
		imagez:div(imagez:max()+0.000001)
						
		--imagez=imagez:mul(256):floor()
			
		self.imagez = imagez:clone():repeatTensor(3,1,1)	

	end

	collectgarbage()
	return self.imagez
	
end

function PointCloud:find_edges()
	if self.format == 1 then
		local pts, minus_lr, minus_ud = self:make_normal_map()
		
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
					pts:clone():sub(1,self.height,2,self.width)):sum(3):squeeze()):abs():cdiv(
					plane_const_norm:sub(1,self.height,1,self.width-1)))
		compare_plane_lr:sub(1,self.height,1,1):add(
					plane_const:clone():sub(1,self.height,self.width,self.width):mul(-1):add(
					normal_map:clone():sub(1,self.height,self.width,self.width):cmul(
					pts:clone():sub(1,self.height,1,1)):sum(3):squeeze()):abs():cdiv(
					plane_const_norm:sub(1,self.height,self.width,self.width)))
					
		compare_plane_ud:sub(2,self.height,1,self.width):add(
					plane_const:clone():sub(1,self.height-1,1,self.width):mul(-1):add(
					normal_map:clone():sub(1,self.height-1,1,self.width):cmul(
					pts:clone():sub(2,self.height,1,self.width)):sum(3):squeeze()):abs():cdiv(
					plane_const_norm:sub(1,self.height-1,1,self.width)))
		compare_plane_ud:sub(1,1):add(compare_plane_ud:clone():sub(1,1))
		
		compare_normal_lr:sub(1,self.height,2,self.width):add(
					normal_map:clone():sub(1,self.height,2,self.width):mul(-1):add(
					normal_map:clone():sub(1,self.height,1,self.width-1)):pow(2):sum(3):squeeze())
		compare_normal_lr:sub(1,self.height,1,1):add(
					normal_map:clone():sub(1,self.height,1,1):mul(-1):add(
					normal_map:clone():sub(1,self.height,self.width,self.width)):pow(2):sum(3):squeeze())
		
		compare_normal_ud:sub(2,self.height,1,self.width):add(
					normal_map:clone():sub(2,self.height,1,self.width):mul(-1):add(
					normal_map:clone():sub(1,self.height-1,1,self.width)):pow(2):sum(3):squeeze())
		compare_normal_ud:sub(1,1):add(compare_normal_ud:clone():sub(1,1))
		
		local nm_tol = 0.25
		local pc_tol = 0.10
		local z_tol = 0.25
		
		local tmp11 = compare_plane_lr:clone():add(-pc_tol):mul(-1)
		tmp11:add(tmp11:clone():abs())
		tmp11:cdiv(tmp11:clone():add(0.000001))
		local tmp12 = compare_plane_ud:clone():add(-pc_tol):mul(-1)
		tmp12:add(tmp12:clone():abs())
		tmp12:cdiv(tmp12:clone():add(0.000001))
		
		local tmp1 = tmp11:clone():cmul(tmp12)
		
		local tmp21 = compare_normal_lr:clone():add(-nm_tol):mul(-1)
		tmp21:add(tmp21:clone():abs())
		tmp21:cdiv(tmp21:clone():add(0.000001))
		local tmp22 = compare_normal_ud:clone():add(-nm_tol):mul(-1)
		tmp22:add(tmp22:clone():abs())
		tmp22:cdiv(tmp22:clone():add(0.000001))
		
		local tmp2 = tmp21:clone():cmul(tmp22)
		
		local tmp3 = normal_map:select(3,3):clone():abs():add(-z_tol):mul(-1)
		tmp3:add(tmp3:clone():abs())
		tmp3:cdiv(tmp3:clone():add(0.000001))
		
		local edges = tmp1:cmul(tmp2):cmul(tmp3):ceil()
		--local edges = tmp1
		
		local s = 0
		edges:apply(function(x)
			if not (x < math.huge and x > -math.huge) then
				s = s+1
				return 0
			elseif x== 0 then
				s = s+1
			end
		end)
		
		--[[]]
		local kern = torch.Tensor({{1,1,1}, {1,0,1},{1,1,1}})
		local conv = torch.conv2(edges:clone(),kern,'F')
		conv:div(7):floor()
		conv:cdiv(conv:clone():add(0.000000001)):ceil()
		
		edges:add(conv:sub(3,self.height+2,3,self.width+2))
		edges:cdiv(edges:clone():add(0.000000001)):ceil()
		
		kern = torch.Tensor({{1,1,1}, {1,0,1},{1,1,1}})
		conv = torch.conv2(edges:clone(),kern,'F')
		conv:div(8):floor()
		conv:cdiv(conv:clone():add(0.000000001)):ceil()
		
		edges:add(conv:sub(3,self.height+2,3,self.width+2))
		edges:cdiv(edges:clone():add(0.000000001)):ceil()
		
		--[[]]
		kern = torch.Tensor({{1,1,1},{1,0,1},{1,1,1}})
		conv = torch.conv2(edges:clone(),kern,'F')
		conv:div(5):floor()
		conv:cdiv(conv:clone():add(0.000000001)):ceil()
		
		edges:cmul(conv:sub(2,self.height+1,2,self.width+1)):ceil()
		--[[]]
		
		return edges, pts

	end
end

function PointCloud:make_normal_map()
	if self.format == 1 then
		local height = self.height
		local width = self.width
		local img = torch.zeros(height, width, 3):add(math.huge)
		local tmp = torch.range(1,self.count)
		tmp:apply(function(i)
					local ind = self.hwindices[i]
	   				img[ind[1] ][ind[2] ] = self.points[i]
			    end)
	    
		local minus_lr = torch.zeros(height,width,3)
		local minus_ud = torch.zeros(height,width,3)
		
		minus_lr:sub(1,height,1,width-1):add(
	    			 img:sub(1,height,1,width-1):clone():mul(-1):add(
    				 img:sub(1,height,2,width)))
		minus_lr:sub(1,height,width,width):add(
		   			 img:sub(1,height,width,width):clone():mul(-1):add(
	    			 img:sub(1,height,1,1)))
	    
	    --[[
		minus_ud:sub(2,height-1,1,width):add(
	    			 img:sub(1,height-2,1,width):clone():mul(-1):add(
					 img:sub(3,height,1,width)))
		minus_ud:sub(1,1,1,width):add(
	    			 img:sub(1,1,1,width):clone():mul(-1):add(
		   			 img:sub(2,2,1,width)))
		minus_ud:sub(height,height,1,width):add(
	    			 img:sub(height-1,height-1,1,width):clone():mul(-1):add(
					 img:sub(height,height,1,width)))
		--[[]]
		
		--[[]]
		minus_ud:sub(1,height-1,1,width):add(
	    			 img:sub(1,height-1,1,width):clone():mul(-1):add(
					 img:sub(2,height,1,width)))
		minus_ud:sub(self.height,self.height,1,width):add(
	    			 img:sub(self.height-1,self.height-1,1,width):clone())
		--[[]]
	    
	    local minus_lr_t = minus_lr:transpose(1,3)
	    local minus_ud_t = minus_ud:transpose(1,3)
	    
	    local minus_lr_tx = minus_lr_t[1]
	    local minus_lr_ty = minus_lr_t[2]
	    local minus_lr_tz = minus_lr_t[3]
	    
	    local minus_ud_tx = minus_ud_t[1]
	    local minus_ud_ty = minus_ud_t[2]
	    local minus_ud_tz = minus_ud_t[3]
	  	
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
		return img, minus_lr, minus_ud
	end
	return nil,nil,nil
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
	
	local downsampled = PointCloud.new()
	downsampled.height = 0;
  	downsampled.width = 0;
	
	local tmp = torch.range(1,self.count)
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
			  if neq[i] == 1 then
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
				end
			  end)
	
	downsampled.points = points:sub(1,count)
	downsampled.rgb = rgb:sub(1,count)
	downsampled.count = count
	
	downsampled:reset_point_stats()
	collectgarbage()
	return downsampled
end

--[[
function PointCloud:make_flattened_images(scale)
	scale = scale+0.000000001
	local ranges = self.radius:clone():mul(2)
	local minv = self.radius:clone():mul(-1)
	local maxv = self.radius:clone()
	local pix = ranges:clone():div(scale):floor()
	local height = pix[1]+1
	local width = pix[2]+1
	local imagez = torch.zeros(height*width)
	
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
	self.imagez = imagez:resize(height,width):repeatTensor(3,1,1)
	collectgarbage()
end
]]

function PointCloud:make_3dtree()
	self.k3dtree = kdtree.new(self.points)
	collectgarbage()
end

--[[

function PointCloud:make_panoramic_directional_map()
	if self.format == 1 then
		local img = torch.ones(self.height, self.width, 3)
		local norm_factor = self.radius
		for i = 1,self.count do
	    	local h = self.hwindices[i][1]
	    	local w = self.hwindices[i][2]
	    	img[h][w] = (self.points[i]-self.centroid):abs():cdiv(norm_factor)
	    end
		collectgarbage()
		local depth_map = img
		depth_map = depth_map:transpose(1,3):transpose(2,3)
		depth_map[1] = depth_map[1]/depth_map[1]:max()
		depth_map[2] = depth_map[2]/depth_map[2]:max()
		depth_map[3] = depth_map[3]/depth_map[3]:max()
		depth_map = (depth_map-1):abs()
		depth_map[1] = depth_map[1]/depth_map[1]:max()
		depth_map[2] = depth_map[2]/depth_map[2]:max()
		depth_map[3] = depth_map[3]/depth_map[3]:max()
		return depth_map
	else
		print("can't make panoramic image, no w/h info given")
	end
end

function PointCloud:make_sub_pointcloud(index_tensor)
	local sub_ptcld = PointCloud.new()
	if index_tensor then
		sub_ptcld.points = torch.zeros(index_tensor:size(1),3)
		sub_ptcld.rgb = torch.zeros(index_tensor:size(1),3)
		sub_ptcld.count = index_tensor:size(1)
		sub_ptcld.centroid = torch.zeros(1,3)
		
		for i=1,index_tensor:size(1) do
			sub_ptcld.points[i] = self.points[index_tensor[i] ]
			sub_ptcld.rgb[i] = self.rgb[index_tensor[i] ]
		end
		sub_ptcld:reset_point_stats()
    end
    collectgarbage()
    return sub_ptcld
end
]]