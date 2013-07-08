local io = require 'io'

-- angular to radians and back
local pi = math.pi
local r2d = 180/pi
local d2r = pi/180

local PointCloud = Class()

local PC_FILE_EXTENSION = '.xyz'
local PC_BINARY_EXTENSION = 'bin'

function PointCloud:__init(pcfilename)
  self.points = torch.Tensor(0,3);
  self.rgb = torch.Tensor(0,3);
  self.normals = torch.Tensor(0,3);
  self.curvatures = torch.Tensor(0);
  self.count = 0;
  self.meanpoint = torch.Tensor({{0,0,0}});
  
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
    while true do
      local line = file:read();
      if line == nil or line:len() < 5 then 
        break 
      end
      count = count + 1
    end
    
    self.points = torch.Tensor(count,3);
    self.rgb = torch.Tensor(count,3);
    self.normals = torch.Tensor(count,3);
    self.curvatures = torch.Tensor(count);
    self.count = count;
    count = 0;
    
    file = io.open(pcfilename, 'r');
    while true do
      local line = file:read();
      if line == nil or line:len() < 5 then
        break 
      end
      count = count + 1
      local begp = line:find(' ') + 1;
      begp = line:find(' ', begp) + 1;
      local endp = line:find(' ', begp) - 1;
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
    end
    
    self.meanpoint = self.points:mean(1);
    
  else
    error('arg #1 must be a valid xyz file')
  end
end