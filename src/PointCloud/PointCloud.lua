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
  self.index = torch.Tensor(0,0);
  self.height = 0;
  self.width = 0;
  self.points = torch.Tensor(0,3);
  self.rgb = torch.Tensor(0,3);
  self.normals = torch.Tensor(0,3);
  self.curvatures = torch.Tensor(0);
  self.count = 0;
  self.meanpoint = torch.Tensor({{0,0,0}});
  self.image = nil;
  self.kdtree = nil;
  
  if pcfilename then
    if util.fs.is_file(pcfilename) then
      self:set_pc_file(pcfilename);
    else
      error('arg #1 must either be empty or a valid file')
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

    print("count: "..self.count..", height: "..self.height..", width: "..self.width);
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
    
  else
    error('arg #1 must be a valid xyz file')
  end
  
  collectgarbage();
  
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
    self.image = img:transpose(1,3):transpose(2,3);
    self.image = self.image/self.image:max();
end

function PointCloud:make_kdtree()
	self.kdtree = kdtree.new(self.points)
end