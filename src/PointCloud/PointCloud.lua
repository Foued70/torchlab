local io = require 'io'
local kdtree = kdtree.kdtree

-- angular to radians and back
local pi = math.pi
local r2d = 180/pi
local d2r = pi/180

local PointCloud = Class()

local PC_ASCII_EXTENSION = '.xyz'
local PC_BINARY_EXTENSION = '.xyb'

function PointCloud:__init(pcfilename, radius, numstd, option)
  self.index = nil;
  self.height = 0;
  self.width = 0;
  self.points = nil;
  self.rgb = nil;
  self.count = 0;
  self.centroid = torch.Tensor({{0,0,0}});
  if (not radius) then
  	--default radius prune
  	radius = 250
  end
  if (not numstd) then
  	--default number of standard dev prune
  	numstd = 3
  end
  
  if pcfilename then
    if util.fs.is_file(pcfilename) then
    	if util.fs.extname(pcfilename)==PC_ASCII_EXTENSION then
	    	self:set_pc_ascii_file(pcfilename, radius, numstd, option)
	    	--self:set_pc_file(pcfilename, radius, numstd, option)
	    elseif util.fs.extname(pcfilename)==PC_BINARY_EXTENSION then
	    	self:set_pc_binary_file(pcfilename)
	    else
	    	error('arg #1 must either be empty or a valid file')
	    end	
    else
      error('arg #1 must either be empty or a valid file')
    end
  end
end

function PointCloud:reset_point_stats()
	self.centroid=torch.Tensor({{0,0,self.points:mean(1)[1][3]}})
	self.minval,self.minind = self.points:min(1)
    self.maxval,self.maxind = self.points:max(1)
    local d1 = (self.maxval-self.centroid)[1]
    local d2 = (self.centroid-self.minval)[1]
    self.radius=torch.Tensor({math.max(d1[1],d2[1]),math.max(d1[2],d2[2]),math.max(d1[3],d2[3])})
    
end

local function hex_to_integer(string_x)
	
	if string.len(string_x) > 8 then
		local neg1 = string.format('%X',-1)
		local n1 = tonumber(string.sub(neg1,string.len(neg1)-8,string.len(neg1)),16)
		local n2 = tonumber(string.sub(string_x,string.len(string_x)-8,string.len(string_x)),16)
		return n2-n1-1
	else
		return tonumber(string_x,16)
	end
end

function PointCloud:set_pc_binary_file(pcfilename)

	local count = 0;
    self.height = 0;
	self.width = 0;
	local xyz_table={}
	local rgb_table={}
    file = io.open(pcfilename,'r')
	self.format = 0
	
    local data = file:read('*all')
    file:close()
    
    data=data..'\n'
    data=string.gsub(data, '\r', '\n')
    data=string.gsub(data, '\n+', '\n')
    
    local i = -1
    local minx, miny, minz
    for x,y,z,r,g,b in string.gmatch(data, '(%x+) (%x+) (%x+) (%x+) (%x+) (%x+)\n') do
    	
    	x = hex_to_integer(x)
    	y = hex_to_integer(y)
    	z = hex_to_integer(z)
    	r = hex_to_integer(r)
    	g = hex_to_integer(g)
    	b = hex_to_integer(b)
    	
    	if i < 0 then
    		minx = x
			miny = y
			minz = z
			
			print(torch.Tensor({x/1000,y/1000,z/1000}))
    	else
    		x = (x+minx)/1000.0
    		y = (y+miny)/1000.0
    		z = (z+minz)/1000.0
    				
    		count = count + 1
			table.insert(xyz_table,{x,y,z})
			table.insert(rgb_table,{r,g,b})
    	end
    	
    	i=i+1
	end
    
	self.points = torch.Tensor(xyz_table)
	self.rgb = torch.Tensor(rgb_table)
	self.count = count
	
	self:reset_point_stats()
	
	collectgarbage()
		
end

function PointCloud:set_pc_ascii_file(pcfilename, radius, numstd)
    	
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
	
	file = io.open(pcfilename,'r')
	self.format = 1
	
    local data = file:read('*all')
    file:close()
    
    data=data..'\n'
    data=string.gsub(data, '\r', '\n')
    data=string.gsub(data, '\n+', '\n')
    
    local minx, miny, minz
    
    for w,h,x,y,z,r,g,b in string.gmatch(data, '(%d+) (%d+) (%-*%d+%p*%d*) (%-*%d+%p*%d*) (%-*%d+%p*%d*) (%d+) (%d+) (%d+)\n') do 
    
    	w = tonumber(w)
    	h = tonumber(h)
    	x = tonumber(x)
    	y = tonumber(y)
    	z = tonumber(z)
    	r = tonumber(r)
    	g = tonumber(g)
    	b = tonumber(b)
    	
    	h=h+1
		w=w+1
		if math.sqrt(math.pow(x,2)+math.pow(y,2)) < rad2d then
	  		count = count + 1
      	  	meanz = meanz + z
			table.insert(xyz_table,{x,y,z})
			table.insert(rgb_table,{r,g,b})
    	  	if h > self.height then
	        	self.height = h;
		    end
		 	if w > self.width then
        		self.width = w;
	  		end
      		table.insert(hw_table,{h,w,count})
      	end
    end
    
    if count == 0 then -- format = 0
    
    	self.format = 0
    	
    	for x,y,z,r,g,b in string.gmatch(data, '(%-*%d+%p*%d*) (%-*%d+%p*%d*) (%-*%d+%p*%d*) (%d+) (%d+) (%d+)\n') do 
    	
    		x = tonumber(x)
	    	y = tonumber(y)
    		z = tonumber(z)
    		r = tonumber(r)
	    	g = tonumber(g)
    		b = tonumber(b)
		    
    		if math.sqrt(math.pow(x,2)+math.pow(y,2)) < rad2d then
	  			count = count + 1
	      	  	meanz = meanz + z
				table.insert(xyz_table,{x,y,z})
				table.insert(rgb_table,{r,g,b})
			end
      	end
    end
    
    collectgarbage()
	
	if count == 0 then
    	print('no points found!')
    	return
    end
        
    self.points = torch.Tensor(xyz_table)
	self.count = count;
	self.centroid = torch.Tensor({{meanx, meany, meanz}})/(count+0.000001)
	local stdrd = math.sqrt((self.points-self.centroid:repeatTensor(self.count,1)):pow(2):sum(2):mean())
	local perc = 0.75
	
	print("radius: "..radius..", stdrd: "..(stdrd*numstd))
	
	if (perc*radius) > (stdrd * numstd) then
	
		radius = stdrd * numstd
		print("make second pass with new radius: "..radius)
		
		count = 0
		hw_table={}
		xyz_table={}
		rgb_table={}
		meanx = 0
		meany = 0
	  	meanz = 0
	  		
		if self.format == 1 then
			for w,h,x,y,z,r,g,b in string.gmatch(data, '(%d+) (%d+) (%-*%d+%p*%d*) (%-*%d+%p*%d*) (%-*%d+%p*%d*) (%d+) (%d+) (%d+)\n') do 
    
 			   	w = tonumber(w)
		    	h = tonumber(h)
    			x = tonumber(x)
		    	y = tonumber(y)
    			z = tonumber(z)
		    	r = tonumber(r)
    			g = tonumber(g)
		    	b = tonumber(b)
    	
    			h=h+1
				w=w+1
		
				if self.centroid[1]:dist(torch.Tensor({x,y,z})) < radius then
			  		count = count + 1
      			  	meanz = meanz + z
					table.insert(xyz_table,{x,y,z})
					table.insert(rgb_table,{r,g,b})
		    	  	if h > self.height then
	    		    	self.height = h;
				    end
				 	if w > self.width then
        				self.width = w;
			  		end
      				table.insert(hw_table,{h,w,count})
		      	end
		    end
		elseif self.format == 0 then
			for x,y,z,r,g,b in string.gmatch(data, '(%-*%d+%p*%d*) (%-*%d+%p*%d*) (%-*%d+%p*%d*) (%d+) (%d+) (%d+)\n') do 
    			x = tonumber(x)
	    		y = tonumber(y)
	    		z = tonumber(z)
    			r = tonumber(r)
	    		g = tonumber(g)
    			b = tonumber(b)
	    		if math.sqrt(math.pow(x,2)+math.pow(y,2)) < rad2d then
		  			count = count + 1
	    	  	  	meanz = meanz + z
					table.insert(xyz_table,{x,y,z})
					table.insert(rgb_table,{r,g,b})
				end
    	  	end
		end
		
		collectgarbage()
		
		self.points = torch.Tensor(xyz_table)
		self.count = count;
		self.centroid = torch.Tensor({{meanx, meany, meanz}})/(count+0.000001)
		
	end
	
	self.rgb = torch.Tensor(rgb_table)
	if self.format == 1 then
	    self.index = torch.zeros(self.height, self.width);
	    for i=1,#hw_table do
	    	local h = hw_table[i][1]
	    	local w = hw_table[i][2]
	    	local c = hw_table[i][3]
	    	if h==0 or w == 0 or h>self.index:size(1) or w>self.index:size(2) then
	    		print('i: '..i..', h: '..h..', w: '..w..', c: '..c)
	    		print(self.index:size())
	    	end
	    	self.index[h][w]=c
	    end
	end
	
	self:reset_point_stats()
    print("count: "..self.count..", height: "..self.height..", width: "..self.width);
    
end

function PointCloud:set_pc_file(pcfilename, radius, numstd)
	if pcfilename and util.fs.is_file(pcfilename) 
				  and util.fs.extname(pcfilename)==PC_ASCII_EXTENSION then
    
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
	      				table.insert(hw_table,{h,w,count})
	      			end
      			end
			end
    	end
    
    	file:close()
    	collectgarbage()
    
	    self.points = torch.Tensor(xyz_table)
		self.count = count;
		self.centroid = torch.Tensor({{meanx, meany, meanz}})/(count+0.000001)
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
	      				table.insert(hw_table,{h,w,count})
	      			end
      			end
			end
			file:close()
			collectgarbage()
			
			self.points = torch.Tensor(xyz_table)
			self.count = count;
			self.centroid = torch.Tensor({{meanx, meany, meanz}})/(count+0.000001)
    	end
    	collectgarbage()
    	
    	self.rgb = torch.Tensor(rgb_table)
	    if self.format == 1 then
		    self.index = torch.zeros(self.height, self.width);
		    for i=1,#hw_table do
		    	local h = hw_table[i][1]
		    	local w = hw_table[i][2]
		    	local c = hw_table[i][3]
		    	if h==0 or w == 0 or h>self.index:size(1) or w>self.index:size(2) then
		    		print('i: '..i..', h: '..h..', w: '..w..', c: '..c)
		    		print(self.index:size())
		    	end
		    	self.index[h][w]=c
		    end
		end
		collectgarbage()

    	self:reset_point_stats()
    	
    	print("pass 2: count: "..self.count..", height: "..self.height..", width: "..self.width);
    
	else
    	error('arg #1 must be a valid xyz file')
  	end
  
  	collectgarbage();
end

function PointCloud:write(filename, option)
	local pts = self.points:clone()
	local file = io.open(filename, 'w');
	if option and option == 'b' then
		file.close()
		file = io.open(filename, 'wb');
		local minx = math.floor(self.minval[1][1] * 1000)
		local miny = math.floor(self.minval[1][2] * 1000)
		local minz = math.floor(self.minval[1][3] * 1000)
		file:write(''..string.format('%X',minx)..' '..string.format('%X',miny)..' '..string.format('%X',minz)..' 0 0 0\n')
		pts = (pts*1000):floor()
		for i = 1,self.count do
			local x = string.format('%X',(pts[i][1]-minx))
			local y = string.format('%X',(pts[i][2]-miny))
			local z = string.format('%X',(pts[i][3]-minz))
			local r = string.format('%X',self.rgb[i][1])
			local g = string.format('%X',self.rgb[i][2])
			local b = string.format('%X',self.rgb[i][3])
			file:write(''..x..' '..y..' '..z..' '..r..' '..g..' '..b..'\n')
		end
	else
		for i=1,self.count do
			file:write(''..pts[i][1]..' '..pts[i][2]..' '..pts[i][3]..' '..self.rgb[i][1]..' '..self.rgb[i][2]..' '..self.rgb[i][3]..'\n')
		end
	end
	file:close()
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
	    self.image = torch.zeros(3, self.height, self.width);
    	local img = torch.zeros(self.height,self.width,3);
	    for i=1,self.height do
    	  for j=1,self.width do
        	if self.index[i][j]==0 then
	          img[i][j]=torch.Tensor({0,0,0});
    	    else
        	  img[i][j]=self.rgb[self.index[i][j]];
	        end
    	  end
	    end
    	collectgarbage()
	    self.image = img:transpose(1,3):transpose(2,3);
    	self.image = self.image/self.image:max();
    else
    	print("can't make panoramic image, no w/h info given")
    end
end

function PointCloud:downsample(leafsize)
	--leafsize is edge length of voxel
	scale = leafsize + 0.000000001
	local ranges = self.maxval[1] - self.minval[1]
	local pix = (ranges/scale+1):floor()
	--local bin = {}
	local pts = {}
	local rgb = {}
	
	local tmp = torch.Tensor(self.count)
	local bin = torch.zeros(pix[1], pix[2], pix[3])
	local coord = ((self.points-self.minval[1]:repeatTensor(self.count,1))/scale):floor() + 1
	local ptss = (coord-1)*scale + self.minval[1]:repeatTensor(self.count,1)
	
	local i = 0
	tmp:apply(function()
			  i = i+1
			  if bin[{coord[i][1],coord[i][2],coord[i][3]}] < 1 then
			  	bin[{coord[i][1],coord[i][2],coord[i][3]}] = bin[{coord[i][1],coord[i][2],coord[i][3]}] + 1
				table.insert(pts, {ptss[i][1],ptss[i][2],ptss[i][3]})
				table.insert(rgb, {self.rgb[i][1],self.rgb[i][2],self.rgb[i][3]})
			  end
			  end)
	
	local downsampled = PointCloud.new()
	downsampled.height = 0;
  	downsampled.width = 0;
	downsampled.points = torch.Tensor(pts)
	downsampled.rgb = torch.Tensor(rgb)
	downsampled.count = #pts
	
	downsampled:reset_point_stats()
	collectgarbage()
	return downsampled
end

function PointCloud:make_flattened_images(scale)
	scale = scale + 0.000000001
	local ranges = self.radius*2
	local minv = self.radius*(-1)
	local maxv = self.radius
	local pix = (ranges/scale):floor()
	local height = pix[1]+1
	local width = pix[2]+1
	local imagez = torch.zeros(height*width)
	
	--sort points
	local points = self.points:clone()[{{},{1,2}}]
	local coords = ((points-minv:sub(1,2):repeatTensor(self.count,1))/scale):floor()+1
	local index = (coords[{{},1}]-1):mul(width)+coords[{{},2}]
	--local dists = (points-self.centroid:squeeze():sub(1,2):repeatTensor(self.count,1)):pow(2):sum(2):sqrt():squeeze()
	local dists = (points-self.centroid:squeeze():sub(1,2):repeatTensor(self.count,1)):pow(2):sum(2):squeeze()
	
	local i=1
	index:apply(function(x)
					imagez[x]=imagez[x]+dists[i]
					i=i+1
					return x
				end)
	
	imagez=imagez:pow(2)
	imagez=(imagez*256/(imagez:max()+0.000001)):floor()
	self.imagez = imagez:resize(height,width):repeatTensor(3,1,1)
	collectgarbage()
end

function PointCloud:make_3dtree()
	self.k3dtree = kdtree.new(self.points)
	collectgarbage()
end

--[[
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


function PointCloud:compute_normals_at_all_points_nn(num)
	if self.points then
		if not self.kdtree then
			self:make_3dtree()
		end
		self.normals = torch.zeros(self.count,3)
		self.curvatures = torch.zeros(self.count)
		for i=1,self.count do
			local nnlist = self.kdtree:get_nearest_neighbors(self.points[i],num)
			local ptcld = self:make_sub_pointcloud(nnlist)
			local norm,curv = ptcld:compute_normal_for_cloud()
			self.normals[i] = norm
			self.curvatures[i] = curv
		end
	end
	collectgarbage()
end

function PointCloud:compute_normals_at_all_points_rad(rad)
	if self.points then
		if not self.kdtree then
			self:make_3dtree()
		end
		self.normals = torch.zeros(self.count,3)
		self.curvatures = torch.zeros(self.count)
		for i=1,self.count do
			local nnlist = self.kdtree:get_neighbors_in_radius(self.points[i],rad)
			local ptcld = self:make_sub_pointcloud(nnlist)
			local norm,curv = ptcld:compute_normal_for_cloud()
			self.normals[i] = norm
			self.curvatures[i] = curv
		end
	end
	collectgarbage()
end

function PointCloud:compute_normal_for_cloud()
	local norm = torch.zeros(3)
	local curv = 0
	if self.points and self.points:size(1) > 0 then
		local e = torch.zeros(3)
		local v = torch.zeros(3,3)
		local cov = self:compute_covariance_matrix()
		e,v = torch.symeig(cov,'V')
		minE = e[1]
		minI = 1
		for i=2,3 do
			if e[i] < minE then
				minI = i
				minE = e[i]
			end
		end
		norm = v:transpose(1,2)[minI]
	end
	collectgarbage()
	return norm,curv
end

function PointCloud:compute_covariance_matrix()
	local cov = torch.zeros(3,3)
	if self.points and self.points:size(1) > 0 then
		for i=1,self.count do
			local DT = (self.points[i]:repeatTensor(1,1)-self.centroid)
			local D = DT:transpose(1,2)
			cov = cov + (D * DT)
		end
		cov = cov / self.count
	end
	collectgarbage()
	return cov
end
]]