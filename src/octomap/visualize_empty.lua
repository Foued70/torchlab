output_filename = "296_297_flatten.obj"
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
-- TODO add rotation and position to PointCloud or make PointCloud part of a View
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

m_per_px          = t.resolution
_G.empty_cells    = t:get_empty()    
_G.occupied_cells = t:get_occupied()

bbxmin = occupied_cells:min(1):squeeze()
bbxmax = occupied_cells:max(1):squeeze()

extent = (bbxmax - bbxmin)  -- in meters

extent[{{1,2}}]:mul(1/m_per_px)

-- make into index into pixel space
occupied_cells[{{},1}]:add(-bbxmin[1]):mul(1/m_per_px):floor():add(1)
occupied_cells[{{},2}]:add(-bbxmin[2]):mul(1/m_per_px):floor():add(1)

empty_cells[{{},1}]:add(-bbxmin[1]):mul(1/m_per_px):floor():add(1)
empty_cells[{{},2}]:add(-bbxmin[2]):mul(1/m_per_px):floor():add(1)

nslice = 5

zranges = torch.linspace(bbxmin[3]-0.1,bbxmax[3]+0.1,nslice+1)

_G.occ_images = {}
_G.emp_images = {}
for r = 1,nslice do 
   occ_out = torch.zeros(3,extent[2]+1,extent[1]+1)
   zmin = zranges[r]
   zmax = zranges[r+1]
   -- occupied RED
   occ_zvals = occupied_cells[{{},3}]
   occ_mask = occ_zvals:gt(zmin) + occ_zvals:lt(zmax)
   occ_mask = occ_mask:eq(2)

   occ_x = occupied_cells[{{},1}][occ_mask]
   occ_y = occupied_cells[{{},2}][occ_mask]

   for i = 1,occ_mask:sum() do 
      x = occ_x[i]
      y = occ_y[i]
      occ_out[{1,y,x}] = 1
   end

   table.insert(occ_images,occ_out)

   -- empty GREEN
   emp_out = torch.zeros(3,extent[2]+1,extent[1]+1)
   empty_zvals = empty_cells[{{},3}]
   empty_mask = empty_zvals:gt(zmin) + empty_zvals:lt(zmax)
   empty_mask = empty_mask:eq(2)

   empty_x = empty_cells[{{},1}][empty_mask]
   empty_y = empty_cells[{{},2}][empty_mask]

   for i = 1,empty_mask:sum() do 
      x = empty_x[i]
      y = empty_y[i]
      emp_out[{2,y,x}] = 1
   end

   table.insert(emp_images,emp_out)

end

image.display{image=occ_images}
image.display{image=emp_images}
