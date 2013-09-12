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
