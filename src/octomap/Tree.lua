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
   resolution = resolution or 0.05 -- 5 cm
   -- initialize an empty tree*
   self.tree = ffi.gc(octomap_ffi.OcTree_new(resolution), destructor())
end

function Tree:add_points(pointcloud)

end
