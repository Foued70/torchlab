local io = require 'io'
local kdtree = kdtree.kdtree
local ffi = require "ffi"
local ctorch = util.ctorch
ffi.cdef
[[

void * fopen (const char * filename, const char *mode );
int fclose   (void * stream );
int fscanf   (void * stream, const char * format, ...);
int fprintf  (void * stream, const char * format, ...);
void * malloc(int size);
void free(void * ptr);
]]

-- angular to radians and back
local pi = math.pi
local r2d = 180/pi
local d2r = pi/180

local PointCloud = Class()

local PC_OD_EXTENSION = '.od'
local PC_ASCII_EXTENSION = '.xyz'

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
		    	self.index = loaded[2]
		    end
		    local pts = loaded[3]
		    self.points = pts:type('torch.DoubleTensor') /10000.0
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
    local d1 = (self.maxval-self.centroid)[1]
    local d2 = (self.centroid-self.minval)[1]
    self.radius=torch.Tensor({math.max(d1[1],d2[1]),math.max(d1[2],d2[2]),math.max(d1[3],d2[3])})
end

function PointCloud:set_pc_ascii_file(pcfilename, radius, numstd)
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
    	
    	self.rgb = torch.ShortTensor(rgb_table)
	    if self.format == 1 then
		    self.index = torch.zeros(self.height, self.width);
		    self.index = self.index:type('torch.IntTensor')
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

function PointCloud:write(filename)
	
	if util.fs.extname(filename)==PC_OD_EXTENSION then
		local pts = (self.points*10000):type('torch.IntTensor')
		torch.save(filename, {self.format, self.index, pts, self.rgb})
	elseif util.fs.extname(filename)==PC_ASCII_EXTENSION then
		local file = io.open(filename, 'w');
		local pts = self.points
		for i=1,self.count do
			file:write(''..pts[i][1]..' '..pts[i][2]..' '..pts[i][3]..' '..self.rgb[i][1]..' '..self.rgb[i][2]..' '..self.rgb[i][3]..'\n')
		end
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

function PointCloud:set_pc_ascii_file(pcfilename, radius, numstd)
    	
	local count = 0;
    self.height = 0;
	self.width = 0;
    local meanx = 0
	local meany = 0
	local meanz = 0
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
    
    local xyz_table = {}
    local rgb_table = {}
    local hw_table = {}
    
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
	
	self.rgb = torch.ShortTensor(rgb_table)
	if self.format == 1 then
	    self.index = torch.zeros(self.height, self.width);
	    self.index = self.index:type('torch.IntTensor')
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

function PointCloud:set_pc_ascii_file(pcfilename, radius, numstd)
	local count = 0;
    self.height = 0;
	self.width = 0;
   -- assume z is flatter
	local rad2d = math.sqrt(math.pow(radius,2)*2/2.25)
	
	local h = ffi.new('int [1]')
	local w = ffi.new('int [1]')
	local x = ffi.new('float [1]')
	local y = ffi.new('float [1]')
	local z = ffi.new('float [1]')
	local r = ffi.new('int [1]')
	local g = ffi.new('int [1]')
	local b = ffi.new('int [1]')

	local file = ffi.C.fopen(pcfilename,'r')
	while ffi.C.fscanf(file, "%d %d %f %f %f %d %d %d", w, h, x, y, z, r, g, b) > 0 do
		if not self.format then
			self.format = 1
		end
		if math.sqrt(math.pow(x[0],2) + math.pow(y[0],2)) <= rad2d then
			count = count + 1	
			if w[0]+1 > self.width then
				self.width = w[0]+1
			end
			if h[0]+1 > self.height then
				self.height = h[0]+1
			end
		end
	end
	ffi.C.fclose(file)

	if count <= 1 then
		file = ffi.C.fopen(pcfilename, 'r')
		count = 0
		self.format = 0
		while ffi.C.fscanf(file, "%f %f %f %d %d %d", x, y, z, r, g, b) > 0 do
			if math.pow(x[0],2) + math.pow(y[0],2) > 25 then
				count = count + 1	
			end
		end
		ffi.C.fclose(file)
	end
	
	if count <= 1 then
		self.count = 0
		print('no points found!')
		return
	end
	
	self.count = count
	local points = torch.Tensor(count,3)
	local rgb = torch.ShortTensor(count,3)
	if self.format == 1 then
		self.index = torch.IntTensor(self.height,self.width)
	end
	local count_hw_index = torch.IntTensor(count,2)
	
	count = 0
	local file = ffi.C.fopen(pcfilename,'r')
	if self.format == 1 then
		while ffi.C.fscanf(file, "%d %d %f %f %f %d %d %d", w, h, x, y, z, r, g, b) > 0 do
			if math.pow(x[0],2) + math.pow(y[0],2) < rad2d then
				count = count + 1	
				points[count] = torch.Tensor({x[0],y[0],z[0]})
				rgb[count] = torch.ShortTensor({r[0],g[0],b[0]})
				self.index[{h[0]+1,w[0]+1}]=count
				count_hw_index[count]=torch.IntTensor({h[0]+1,w[0]+1})
			end
		end
	else
		while ffi.C.fscanf(file, "%f %f %f %d %d %d", x, y, z, r, g, b) > 0 do
			if math.pow(x[0],2) + math.pow(y[0],2) < rad2d then
				count = count + 1
				points[count] = torch.Tensor({x[0],y[0],z[0]})
				rgb[count] = torch.TShortensor({r[0],g[0],b[0]})
			end
		end
	end
	ffi.C.fclose(file)
	
	self.points = points
	self.rgb = rgb
	
	self:reset_point_stats()
    print("count: "..self.count..", height: "..self.height..", width: "..self.width);
    
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