io = require 'io'

_G.p = PointCloud.PointCloud.new("pts.xyz")

-- get middle z value
rowid   = math.floor((90 / (90 + 60)) * p.height)
xyz_map = p:get_xyz_map()
midrow  = xyz_map[{3,rowid,{}}]
mean_z  = midrow[midrow:gt(0)]:mean()

max_radius = torch.max(p.radius)

_G.t = octomap.Tree.new()

t:add_points(p.points,torch.Tensor({0,0,mean_z}),max_radius)

t:stats()

vth = t:toTensor()

-- write obj
filename = "voxel_grid.obj"
log.trace("writing "..filename)
objf = assert(io.open(filename, "w"))  
pid = 1
face_radius = t.resolution * 0.5
for i = 1,vth:size(1) do 
   pt = vth[i]
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
