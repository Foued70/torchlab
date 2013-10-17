pi = math.pi
pi2 = pi/2
io = require 'io'
src_dir = 'arcs/temporary-circle-6132/source/po_scan/a/001/'
wrk_dir = src_dir:gsub("source","work")

-- new scans are in mm which makes most default measurements 1000x off...
max_radius      = 25000
tree_resolution = 15

pcfile = src_dir .. 'sweep.xyz'
rawfile = src_dir .. 'sweep.raw'
max_radius = 25000
_G.pc   = PointCloud.PointCloud.new(pcfile, max_radius)
_G.normals = pc:get_normal_map()

-- simple classifier
nangle = geom.util.unit_cartesian_to_spherical_angles(normals)
elevation = nangle[2]
eps = pi/6
_G.rgb = torch.Tensor(3,elevation:size(1),elevation:size(2))

_G.walls = elevation:gt(-eps):cmul(elevation:lt(eps))
-- floor
rgb[1] = elevation:gt(-pi2-eps):cmul(elevation:lt(-pi2+eps))
-- walls
rgb[2] = walls
-- ceiling
rgb[3] = elevation:gt(pi2-eps):cmul(elevation:lt(pi2+eps))

pts = pc:get_xyz_map()
_G.wallpts = torch.Tensor(3,walls:sum())
wallpts[1] = pts[1][walls]
wallpts[2] = pts[2][walls]
wallpts[3] = pts[3][walls]

wallpts = wallpts:transpose(1,2):contiguous()

scanner_pose = torch.Tensor({0,0,0})
tree_resolution = 20 

_G.tree = octomap.Tree.new(tree_resolution)

print("loading points")
tree:add_points(wallpts,scanner_pose,max_radius)

tree:stats()

mm_per_px          = tree.resolution
_G.empty_cells    = tree:get_empty()    
_G.occupied_cells = tree:get_occupied()

bbxmin = occupied_cells:min(1):squeeze()
bbxmax = occupied_cells:max(1):squeeze()

extent = (bbxmax - bbxmin)  -- in mm

extent[{{1,2}}]:mul(1/mm_per_px)

-- make into index into pixel space
occupied_cells[{{},1}]:add(-bbxmin[1]):mul(1/mm_per_px):floor():add(1)
occupied_cells[{{},2}]:add(-bbxmin[2]):mul(1/mm_per_px):floor():add(1)

empty_cells[{{},1}]:add(-bbxmin[1]):mul(1/mm_per_px):floor():add(1)
empty_cells[{{},2}]:add(-bbxmin[2]):mul(1/mm_per_px):floor():add(1)


_G.out_occ_RGB = torch.zeros(3,extent[2]+1,extent[1]+1)
_G.out_emp_RGB = torch.zeros(3,extent[2]+1,extent[1]+1)

for i = 1,occupied_cells:size(1) do 
   x = occupied_cells[{i,1}]
   y = occupied_cells[{i,2}]
   out_occ_RGB[{1,y,x}] = 1
end

for i = 1,empty_cells:size(1) do 
   x = empty_cells[{i,1}]
   y = empty_cells[{i,2}]
   out_emp_RGB[{2,y,x}] = 1
end

image.display{image={out_occ_RGB,out_emp_RGB}}
