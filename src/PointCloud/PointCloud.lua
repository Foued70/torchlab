local log = require '../util/log'
local io = require 'io'
local kdtree = PointCloud.kdtree

-- angular to radians and back
local pi = math.pi
local r2d = 180/pi
local d2r = pi/180

local PointCloud = Class()

local PC_FILE_EXTENSION = '.xyz'
local PC_BINARY_EXTENSION = 'bin'

function PointCloud:__init(pcfilename)
  self.index = nil;
  self.height = 0;
  self.width = 0;
  self.points = nil;
  self.rgb = nil;
  self.normals = nil;
  self.curvatures = nil;
  self.count = 0;
  self.meanpoint = torch.Tensor({{0,0,0}});
  self.image = nil;
  self.kdtree = nil;
  
  if pcfilename then
    if util.fs.is_file(pcfilename) then
      self:set_pc_file(pcfilename);
    else
      log.error('arg #1 must either be empty or a valid file')
    end
  end
end

function PointCloud:set_pc_file(pcfilename)
  if pcfilename and util.fs.is_file(pcfilename) and util.fs.extname(pcfilename)==PC_FILE_EXTENSION then
    
    local file = io.open(pcfilename, 'r');
    local count = 0;
    self.height = 0;
    self.width = 0;
    while true do
      local line = file:read();
      if line == nil or line:len() < 5 then 
        break 
      end
      count = count + 1
      local begp = 1;
      local endp = line:find(' ', begp) - 1;
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
    end
    
    self.points = torch.zeros(count,3);
    self.rgb = torch.zeros(count,3);
    self.normals = torch.zeros(count,3);
    self.curvatures = torch.zeros(count);
    self.index = torch.zeros(self.height, self.width);
    self.count = count;

    log.info("count: "..self.count..", height: "..self.height..", width: "..self.width);
    count = 0;
    file = io.open(pcfilename, 'r');
    while true do
      local line = file:read();
      if line == nil or line:len() < 5 then
        break
      end
      count = count + 1
      local begp = 1;
      local endp = line:find(' ', begp) - 1;
      local h = tonumber(line:sub(begp,endp)) + 1;
      begp = endp + 2;
      endp = line:find(' ', begp) - 1;
      local w = tonumber(line:sub(begp,endp)) + 1;
      begp = endp + 2;
      endp = line:find(' ', begp) - 1;
      local x = tonumber(line:sub(begp,endp));
      begp = endp + 2;
      endp = line:find(' ', begp) - 1;
      local y = tonumber(line:sub(begp,endp));
      begp = endp + 2;
      endp = line:find(' ', begp) - 1;
      local z = tonumber(line:sub(begp,endp));
      begp = endp + 2;
      endp = line:find(' ', begp) - 1;
      local r = tonumber(line:sub(begp,endp));
      begp = endp + 2;
      endp = line:find(' ', begp) - 1;
      local g = tonumber(line:sub(begp,endp));
      begp = endp + 2;
      endp = line:find('\r') - 1;
      local b = tonumber(line:sub(begp,endp));
      self.points[count] = torch.Tensor({x,y,z});
      self.rgb[count] = torch.Tensor({r,g,b});
      self.index[h][w]=count;
    end
    
    self.meanpoint = self.points:mean(1);
    self.minval,self.minind = self.points:min(1)
    self.maxval,self.maxind = self.points:max(1)
    
  else
    log.error('arg #1 must be a valid xyz file')
  end
  
  collectgarbage();
  
end

function PointCloud:flatten()
	self.flattenx = torch.zeros(self.count, 2)
	self.flatteny = torch.zeros(self.count, 2)
	self.flattenz = torch.zeros(self.count, 2)
	
	for i=1,self.count do
		self.flattenx[i] = torch.Tensor({self.points[i][2], self.points[i][3]})
		self.flatteny[i] = torch.Tensor({self.points[i][1], self.points[i][3]})
		self.flattenz[i] = torch.Tensor({self.points[i][1], self.points[i][2]})
	end
	collectgarbage()
end

function PointCloud:make_image()
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
end

function PointCloud:make_flattened_images(scale)
	scale = scale + 0.000001
	local ranges = self.maxval[1] - self.minval[1]
	local pix = ranges/scale
	self.imagex = torch.zeros(pix[3]+1,pix[2]+1,3)
	self.imagey = torch.zeros(pix[3]+1,pix[1]+1,3)
	self.imagez = torch.zeros(pix[1]+1,pix[2]+1,3)
	for i=1,self.count do
		local coord = (self.points[i]-self.minval[1])/scale
		self.imagex[pix[3]-coord[3]+1][coord[2]+1] = torch.Tensor({1,1,1})
		self.imagey[pix[3]-coord[3]+1][coord[1]+1] = torch.Tensor({1,1,1})
		self.imagez[coord[1]+1][coord[2]+1] = torch.Tensor({1,1,1})
	end
	collectgarbage()
	self.imagex = self.imagex:transpose(1,3):transpose(2,3)/self.imagex:max()
	self.imagey = self.imagey:transpose(1,3):transpose(2,3)/self.imagey:max()
	self.imagez = self.imagez:transpose(1,3):transpose(2,3)/self.imagez:max()
end

function PointCloud:make_3dtree()
	local numaxis = 3
	self.kdtree = kdtree.new(self.points,numaxis)
	collectgarbage()
end

function PointCloud:make_2dtrees()
	local numaxis = 2
	self.flattenx_2dtree = kdtree.new(self.flattenx,numaxis)
	self.flatteny_2dtree = kdtree.new(self.flatteny,numaxis)
	self.flattenz_2dtree = kdtree.new(self.flattenz,numaxis)
	collectgarbage()
end