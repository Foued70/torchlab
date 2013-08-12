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
	      				table.insert(hw_table,{h,w})
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
	      				table.insert(hw_table,{h,w})
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
    	
    	self.rgb = torch.ByteTensor(rgb_table)
	    if self.format == 1 then
	    	self.hwindices = torch.ShortTensor(hw_table)
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
		torch.save(filename, {self.format, self.hwindices, pts, self.rgb})
	elseif util.fs.extname(filename)==PC_ASCII_EXTENSION then
		local file = io.open(filename, 'w');
		local pts = self.points
		for i=1,self.count do
			if self.format == 1 then
				file:write(''..(self.hwindices[i][1]-1)..' '..(self.hwindices[i][2]-1)..' ')
			end
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
    	local img = torch.zeros(self.height,self.width,3);
    	local rgb = self.rgb:clone()
	    for i = 1,self.count do
	    	local h = self.hwindices[i][1]
	    	local w = self.hwindices[i][2]
	    	img[h][w] = rgb[i]
	    end
    	collectgarbage()
	    local pan_image = img:transpose(1,3):transpose(2,3);
    	pan_image = pan_image/pan_image:max();
    	return pan_image
    else
    	print("can't make panoramic image, no w/h info given")
    end
end

function PointCloud:make_panoramic_depth_map()
	if self.format == 1 then
		local img = torch.ones(self.height, self.width, 3)
		local norm_factor = self.radius:norm()
		for i = 1,self.count do
	    	local h = self.hwindices[i][1]
	    	local w = self.hwindices[i][2]
	    	img[h][w] = torch.Tensor(3):fill(self.centroid:squeeze():dist(self.points[i]))/norm_factor
	    end
		collectgarbage()
		local depth_map = img/img:max()
		depth_map = depth_map:transpose(1,3):transpose(2,3)
		depth_map = (depth_map-1):abs()
		depth_map = depth_map/depth_map:max()
		return depth_map
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