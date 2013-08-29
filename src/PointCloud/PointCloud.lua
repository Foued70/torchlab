local io = require 'io'
local kdtree = kdtree.kdtree

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
		torch.save(filename, {self.format, self.hwindices, pts, self.rgb})
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