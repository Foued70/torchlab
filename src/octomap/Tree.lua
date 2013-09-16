ffi        = require 'ffi'
octomap_ffi  = require './octomap_ffi'

Tree = Class()

local function destructor ()
   return function (tree)
      octomap_ffi.OcTree_destroy(tree)
   end
end

function Tree:__init(resolution)
   self.name = "octomap.Tree"
   self.resolution = resolution or 0.05 -- 5 cm
   
   -- initialize an empty tree*
   self.tree = ffi.gc(octomap_ffi.OcTree_new(self.resolution), destructor())
end

-- points in Nx3 (x,y,z) points you want to add to Octree.
function Tree:add_points(points, origin, max_range)
   origin = origin or torch.zeros(3)
   max_range = max_range or -1 -- -1 is no limit
   octomap_ffi.OcTree_add_sweep(self.tree, torch.cdata(points), torch.cdata(origin),max_range)
end

function Tree:toTensor()
   points = torch.Tensor()
   octomap_ffi.OcTree_toTensor(self.tree,torch.cdata(points))
   return points
end

function Tree:stats()
   print("octomap.Tree with resolution: " .. self.resolution)
   octomap_ffi.OcTree_outputStatistics(self.tree)
end

function Tree:writeObjCubes(filename)
   io = require 'io'
   objf = assert(io.open(filename, "w"))  
   pts  = self:toTensor()
   face_radius = self.resolution * 0.5
   pid = 1
   for i = 1,pts:size(1) do 
      pt = pts[i]
      xplus  = pt[1] + face_radius
      xmin   = pt[1] - face_radius
      yplus  = pt[2] + face_radius
      ymin   = pt[2] - face_radius
      zplus  = pt[3] + face_radius
      zmin   = pt[3] - face_radius
      
      objf:write(string.format("v %f %f %f\n",xplus,yplus,zplus)) -- 0
      objf:write(string.format("v %f %f %f\n",xplus,yplus, zmin)) -- 1
      objf:write(string.format("v %f %f %f\n",xplus, ymin,zplus)) -- 2
      objf:write(string.format("v %f %f %f\n",xplus, ymin, zmin)) -- 3
      objf:write(string.format("v %f %f %f\n", xmin,yplus,zplus)) -- 4
      objf:write(string.format("v %f %f %f\n", xmin,yplus, zmin)) -- 5
      objf:write(string.format("v %f %f %f\n", xmin, ymin,zplus)) -- 6
      objf:write(string.format("v %f %f %f\n", xmin, ymin, zmin)) -- 7
      
      -- 1: (+-+)(+--)(---)(--+)
      objf:write(string.format("f %d %d %d %d\n",pid+6,pid+7,pid+3,pid+2))
      -- 2: (--+)(---)(-+-)(-++)
      objf:write(string.format("f %d %d %d %d\n",pid+4,pid+5,pid+7,pid+6))
      -- 3: (-++)(-+-)(++-)(+++)
      objf:write(string.format("f %d %d %d %d\n",pid+0,pid+1,pid+5,pid+4))
      -- 4: (+++)(++-)(+--)(+-+)
      objf:write(string.format("f %d %d %d %d\n",pid+2,pid+3,pid+1,pid+0))
      -- 5: (+-+)(--+)(-++)(+++)
      objf:write(string.format("f %d %d %d %d\n",pid+0,pid+4,pid+6,pid+2))
      -- 6: (+--)(++-)(-+-)(---)
      objf:write(string.format("f %d %d %d %d\n",pid+7,pid+5,pid+1,pid+3))
      
      pid = pid + 8
   end
   objf:close()
end
