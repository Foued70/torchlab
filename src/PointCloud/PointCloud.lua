local io = require 'io'
local kdtree = kdtree.kdtree

-- angular to radians and back
local pi = math.pi
local r2d = 180/pi
local d2r = pi/180

local PointCloud = Class()

local PC_FILE_EXTENSION = '.xyz'
local PC_BINARY_EXTENSION = 'bin'

function PointCloud:__init(pcfilename, radius)
  self.index = nil;
  self.height = 0;
  self.width = 0;
  self.points = nil;
  self.rgb = nil;
  self.count = 0;
  self.centroid = torch.Tensor({{0,0,0}});

  
  if pcfilename then
    if util.fs.is_file(pcfilename) then
      self:set_pc_file(pcfilename, radius);
    else
      error('arg #1 must either be empty or a valid file')
    end
  end
end

function PointCloud:set_pc_file(pcfilename, radius)
	if pcfilename and util.fs.is_file(pcfilename) and util.fs.extname(pcfilename)==PC_FILE_EXTENSION then
    
    	local file = io.open(pcfilename, 'r');
	    local count = 0;
    	self.height = 0;
	    self.width = 0;
    	self.format = 1;
    	local meanx = 0
		local meany = 0
	  	local meanz = 0
	    while true do
        	local line = file:read();
			if line == nil or line:len() < 5 then 
        		break 
		    end
		    
      		count = count + 1
      		
	        if count == 1 then
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
			end
			
	  		local begp = 1;
		    local endp = line:find(' ', begp) - 1;
		    local endp = line:find(' ', begp) - 1;
		    if self.format==1 then
		    	local h = tonumber(line:sub(begp, endp)) + 1;
		   	  	begp = endp + 2;
	      		endp = line:find(' ', begp) - 1;
			   	local w = tonumber(line:sub(begp, endp)) + 1;
	      		if h > self.height then
    	    		self.height = h;
	      		end
    	  		if w > self.width then
        			self.width = w;
	      		end
	      		begp = endp + 2;
	      		endp = line:find(' ', begp) - 1;
	      	end
	  		--meanx = meanx + tonumber(line:sub(begp, endp));
		    begp = endp + 2;
			endp = line:find(' ', begp) - 1;
	    	--meany = meany + tonumber(line:sub(begp, endp));
			begp = endp + 2;
		    endp = line:find(' ', begp) - 1;
    		meanz = meanz + tonumber(line:sub(begp, endp));	
    	end
    
    	local meanpt = torch.Tensor({meanx, meany, meanz})/(count+0.000001)
    
	    self.points = torch.zeros(count,3);
    	self.rgb = torch.zeros(count,3);
	    if self.format == 1 then
		    self.index = torch.zeros(self.height, self.width);
		end
    	self.count = count;

    	print("count: "..self.count..", height: "..self.height..", width: "..self.width);
	    count = 0;
    	file = io.open(pcfilename, 'r');
	    while true do
    		local line = file:read();
	      	if line == nil or line:len() < 5 then
    	    	break
	      	end
    	  	local w,h,x,y,z,r,g,b
    	  	local begp = 1;
    	  	local endp = line:find(' ', begp) - 1;
	      	if self.format == 1 then
	      		h = tonumber(line:sub(begp,endp)) + 1;
      			begp = endp + 2;
		      	endp = line:find(' ', begp) - 1;
	    		w = tonumber(line:sub(begp,endp)) + 1;
		      	begp = endp + 2;
      			endp = line:find(' ', begp) - 1;
      		end
    	  	
		  	x = tonumber(line:sub(begp,endp));
		  	begp = endp + 2;
	  		endp = line:find(' ', begp) - 1;
	        y = tonumber(line:sub(begp,endp));
		    begp = endp + 2;
    	  	endp = line:find(' ', begp) - 1;
      		z = tonumber(line:sub(begp,endp));
	      	begp = endp + 2;
    	  	endp = line:find(' ', begp) - 1;
      		r = tonumber(line:sub(begp,endp));
	      	begp = endp + 2;
    	  	endp = line:find(' ', begp) - 1;
      		g = tonumber(line:sub(begp,endp));
	      	begp = endp + 2;
     		endp = line:len();
  			b = tonumber(line:sub(begp,endp));
      
	    	if (not radius) or meanpt:dist(torch.Tensor({x,y,z})) < radius then
      	  		count = count + 1
      	  		self.points[count] = torch.Tensor({x,y,z});
    	  		self.rgb[count] = torch.Tensor({r,g,b});
	      		if self.format == 1 then
    	  			self.index[h][w]=count;
	      		end
      		end
    	end
    
    	self.count = count
    	self.points = self.points:sub(1,count)
	    self.rgb = self.rgb:sub(1,count)
    
    	self.centroid = self.points:mean(1);
	    self.minval,self.minind = self.points:min(1)
    	self.maxval,self.maxind = self.points:max(1)
    
	else
    	error('arg #1 must be a valid xyz file')
  	end
  
  	collectgarbage();
end

function PointCloud:write(filename)
	local file = io.open(filename, 'w');
	for i=1,self.count do
		file:write(''..self.points[i][1]..' '..self.points[i][2]..' '..self.points[i][3]..' '..self.rgb[i][1]..' '..self.rgb[i][2]..' '..self.rgb[i][3]..'\n')
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

function PointCloud:downsample(leafsize, thresh)
	--leafsize is edge length of voxel
	--thresh is number of points required for a bin to count
	if not thresh then
		thresh = 1
	end
	scale = leafsize + 0.000000001
	local ranges = self.maxval[1] - self.minval[1]
	local pix = ranges/scale
	local bin = {}
	local pts = {}
	for i=1,self.count do
		local coord = (self.points[i]-self.minval[1])/scale + 1
		coord:floor()
		if not bin[coord[1]] then
			bin[coord[1]]={}
		end
		if not bin[coord[1]][coord[2]] then
			bin[coord[1]][coord[2]]={}
		end
		if not bin[coord[1]][coord[2]][coord[3]] then
			bin[coord[1]][coord[2]][coord[3]]=0
		end
		if bin[coord[1]][coord[2]][coord[3]] <= thresh then
			bin[coord[1]][coord[2]][coord[3]]=bin[coord[1]][coord[2]][coord[3]]+1
		end
		if bin[coord[1]][coord[2]][coord[3]] == thresh then
			table.insert(pts, {i, (coord-1)*scale + self.minval[1]})
		end
	end
	bin=nil
	local downsampled = PointCloud.new()
	downsampled.height = self.height;
  	downsampled.width = self.width;
	downsampled.points = torch.Tensor(#pts,3)
	downsampled.rgb = torch.Tensor(#pts,3)
	downsampled.index = torch.Tensor(#pts,2)
	downsampled.count = #pts
	
	for i=1,#pts do
		downsampled.points[i]=pts[i][2]
		downsampled.rgb[i] = self.rgb[pts[i][1]]
	end
	downsampled.centroid = downsampled.points:mean(1);
    downsampled.minval,downsampled.minind = downsampled.points:min(1)
    downsampled.maxval,downsampled.maxind = downsampled.points:max(1)
	collectgarbage()
	return downsampled
end

function PointCloud:make_flattened_images(scale)
	scale = scale + 0.000000001
	local ranges = self.maxval[1] - self.minval[1]
	local pix = ranges/scale
	
	--self.imagex = torch.zeros(pix[3]+1,pix[2]+1,3)
	--self.imagey = torch.zeros(pix[3]+1,pix[1]+1,3)
	self.imagez = torch.zeros(pix[1]+1,pix[2]+1,3)
	for i=1,self.count do
		local coord = (self.points[i]-self.minval[1])/scale
		--self.imagex[pix[3]-coord[3]+1][coord[2]+1] = self.imagex[pix[3]-coord[3]+1][coord[2]+1]+1
		--self.imagey[pix[3]-coord[3]+1][coord[1]+1] = self.imagey[pix[3]-coord[3]+1][coord[1]+1]+1
		self.imagez[coord[1]+1][coord[2]+1] = self.imagez[coord[1]+1][coord[2]+1]+1
	end
	collectgarbage()
	--self.imagex = (self.imagex:pow(2):transpose(1,3):transpose(2,3)*256/self.imagex:max()):floor()
	--self.imagey = (self.imagey:pow(2):transpose(1,3):transpose(2,3)*256/self.imagey:max()):floor()
	self.imagez = (self.imagez:pow(2):transpose(1,3):transpose(2,3)*256/self.imagez:max()):floor()
end

function PointCloud:make_3dtree()
	self.k3dtree = kdtree.new(self.points)
	collectgarbage()
end

function PointCloud:make_sub_pointcloud(index_tensor)
	local sub_ptcld = PointCloud.new()
	if index_tensor then
		sub_ptcld.points = torch.zeros(index_tensor:size(1),3)
		sub_ptcld.rgb = torch.zeros(index_tensor:size(1),3)
		sub_ptcld.count = index_tensor:size(1)
		sub_ptcld.centroid = torch.zeros(1,3)
		
		for i=1,index_tensor:size(1) do
			sub_ptcld.points[i] = self.points[index_tensor[i]]
			sub_ptcld.rgb[i] = self.rgb[index_tensor[i]]
		end
		sub_ptcld.centroid = sub_ptcld.points:mean(1)
	    sub_ptcld.minval,sub_ptcld.minind = sub_ptcld.points:min(1)
    	sub_ptcld.maxval,sub_ptcld.maxind = sub_ptcld.points:max(1)
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