ffi        = require 'ffi'
octomap_ffi  = require './octomap_ffi'

ColorTree = Class(Tree)

local function destructor ()
   return function (tree)
      octomap_ffi.ColorOcTree_destroy(tree)
   end
end

function ColorTree:__init(resolution)
   self.name = "octomap.ColorTree"
   self.resolution = resolution or 0.05 -- 5 cm

   -- initialize an empty tree*
   self.tree = ffi.gc(octomap_ffi.ColorOcTree_new(self.resolution), destructor())
end

function ColorTree:info()
   return octomap_ffi.ColorOcTree_getInfo(self.tree)
end

function ColorTree:save(filename)
   return octomap_ffi.ColorOcTree_write(self.tree,filename)
end

function ColorTree:load(filename)
   return octomap_ffi.ColorOcTree_read(self.tree,filename)
end

-- points in Nx3 (x,y,z) points you want to add to Octree.
function ColorTree:add_points(points, origin, max_range, rgb)
   origin = origin or torch.zeros(3)
   max_range = max_range or -1 -- -1 is no limit
   octomap_ffi.ColorOcTree_add_sweep(self.tree,
                                     torch.cdata(points), torch.cdata(origin), max_range,
                                     torch.cdata(rgb))
end

function ColorTree:add_pointcloud(pc)
   -- use rgb stored in the pointcloud
   pc_rgb_points = pc:get_rgb():contiguous()
   pc_points     = pc:get_global_points():contiguous()
   pc_pose       = pc:get_global_scan_center():contiguous()
   pc_max_radius = pc:get_max_radius()
   self:add_points(pc_points,pc_pose,pc_max_radius,pc_rgb_points)
end

function ColorTree:get_occupied()
   points = torch.Tensor()
   octomap_ffi.OcTree_OccupiedCellstoTensor(self.tree,torch.cdata(points))
   return points
end

function ColorTree:get_colored()
   points = torch.Tensor()
   rgb = torch.ByteTensor()
   octomap_ffi.ColorOcTree_OccupiedCellstoTensor(self.tree,
                                                 torch.cdata(points),
                                                 torch.cdata(rgb))
   return points,rgb
end

function ColorTree:get_empty()
   points = torch.Tensor()
   octomap_ffi.OcTree_EmptyCellstoTensor(self.tree,torch.cdata(points))
   return points
end

function ColorTree:ray_trace(origin, directions, max_range, output_rgb)
   origin     = origin:contiguous()
   directions = directions:contiguous()
   max_range  = max_range or -1
   output_rgb = output_rgb or torch.ByteTensor()
   -- TODO check types and sizes before calling the c++, can be flaky with wrong input.
   octomap_ffi.ColorOcTree_castRays(self.tree,
                                    torch.cdata(origin), torch.cdata(directions),
                                    max_range,
                                    torch.cdata(output_rgb))
   return output_rgb
end

-- for a list of points return the color of corresponding voxel grid
function ColorTree:get_color_for_xyz(points,rgb)
   output_rgb = output_rgb or torch.ByteTensor()
   points = points:contiguous()
   octomap_ffi.ColorOcTree_get_color_for_xyz(self.tree, torch.cdata(points), torch.cdata(output_rgb))
   return output_rgb
end

function ColorTree:stats()
   print("octomap.ColorTree with resolution: " .. self.resolution)
   octomap_ffi.ColorOcTree_outputStatistics(self.tree)
end

function ColorTree:writePointsObj(filename)
   pts  = self:get_occupied()
   if not (pts:nDimension() == 2) then
      print("error: no occupied points")
   else
      io = require 'io'
      objf = assert(io.open(filename, "w"))
      pid = 1

      for i = 1,pts:size(1) do
         pt = pts[i]
         objf:write(string.format("v %f %f %f\n",pt[1],pt[2],pt[3]))
      end
      objf:close()
   end
end


function ColorTree:writeColoredPointsPly(filename)
   pts,rgb  = self:get_colored()
   if not (pts:nDimension() == 2) then
      print("error: no occupied points")
   else
      npts = pts:size(1)
      io = require 'io'
      objf = assert(io.open(filename, "w"))
      objf:write ( [[ply
format ascii 1.0
comment author: Floored Inc.
comment object: colored pointcloud
element vertex ]] .. npts ..
                   [[

property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
end_header
]])
      pid = 1
      for i = 1,npts do
         pt = pts[i]
         c  = rgb[i]
         objf:write(string.format("%f %f %f %d %d %d\n",pt[1],pt[2],pt[3],c[1],c[2],c[3]))
      end
      objf:close()
   end
end

function ColorTree:writeColorCubesObj(filename)
   pts,rgb  = self:get_colored()
   if not (pts:nDimension() == 2) then
      print("error: no occupied points")
   else
      npts = pts:size(1)
      io = require 'io'
      objf = assert(io.open(filename, "w"))
      mtlf = assert(io.open(filename..".mtl", "w"))
      face_radius = self.resolution * 0.5
      pid = 1
      for i = 1,pts:size(1) do
         pt = pts[i]
         c =  rgb[i]

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
end
