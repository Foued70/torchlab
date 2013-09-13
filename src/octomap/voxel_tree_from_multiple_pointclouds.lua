output_filename = "296_297_voxel.obj"
f1 = "faro_kitchen/296.xyz"
f2 = "faro_kitchen/297.xyz"

od1 = f1:gsub("xyz","od")
od2 = f2:gsub("xyz","od")

pc1 = {}
pc2 = {}

if util.fs.is_file(od1) then
   pc1 = PointCloud.PointCloud.new(od1)
else
   pc1 = PointCloud.PointCloud.new(f1)
   pc1:write(od1)
end

if util.fs.is_file(od2) then
   pc2 = PointCloud.PointCloud.new(od2)
else
   pc2 = PointCloud.PointCloud.new(f2)
   pc2:write(od2)
end

_G.pc1 = pc1
_G.pc2 = pc2
-- rotation matrix from Stav's output (put 296 in 297 coordinates
_G.rot = torch.Tensor({{0.9058269262,-0.4292269647,0,-1.978203177},
                       {0.4292269647,0.9058269262,0,1.413987041,},
                       {0,0,1,0},
                       {0,0,0,1}})

rotq  = geom.quaternion.from_rotation_matrix(rot)
trans = rot[{{1,3},4}]:clone()

pc1.points = geom.quaternion.rotate_translate(rotq,trans,pc1.points)

-- get z's close.  subtract min value.
pc1.points[{{},3}]:add(-1*pc1.minval[{1,3}])

pc1_pose = pc1:estimate_faro_pose()
pc1_pose:add(trans)
pc1_max_radius = torch.max(pc1.radius)

log.trace("building tree ...")log.tic()
_G.t = octomap.Tree.new()
log.trace(" - in ".. log.toc())

log.trace(" - add points 1 ...")log.tic()
t:add_points(pc1.points,pc1_pose,pc1_max_radius)
log.trace(" - in ".. log.toc())

t:stats()

pc2.points[{{},3}]:add(-1*pc2.minval[{1,3}])
pc2_pose = pc2:estimate_faro_pose()
pc2_max_radius = torch.max(pc2.radius)

log.trace(" - add points 2 ...")log.tic()
t:add_points(pc2.points,pc2_pose,pc2_max_radius)
log.trace(" - in ".. log.toc())

t:stats()

-- write obj
log.trace("writing "..output_filename) log.tic()
t:writeObjCubes(output_filename)
log.trace(" - in ".. log.toc())
