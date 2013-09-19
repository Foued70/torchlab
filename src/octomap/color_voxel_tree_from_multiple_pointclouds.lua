cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('PointCloud to voxel grid')
cmd:text()
cmd:text('Options')
cmd:option('-pclouddir', '', 'dir with pointclouds to process')
cmd:option('-imagedir',  '', 'dir with images corresponding to pointcloud')
cmd:option('-datadir',   '', 'dir with transform matrices to pointcloud')
cmd:option('-res',       '0.05', 'resolution of voxel grid')
cmd:option('-outfile',   'output.ply', 'ouput .ply file')

cmd:text()

arg = ''

-- parse input params
params = cmd:parse(process.argv)

pointcloud_dir  = params.pclouddir

if not pointcloud_dir then
   error("you need to pass at least the directory containing the pointclouds to process")
end
print("pointcloud_dir: "..pointcloud_dir)

image_dir       = params.imagedir 
if image_dir == '' then
   image_dir = pointcloud_dir
end
print("image_dir:" .. image_dir)

transform_dir   = params.datadir
if transform_dir == '' then
   transform_dir = pointcloud_dir
end
print("transform_dir:" .. image_dir)

output_filename = params.outfile
res             = tonumber(params.res)

pointclouds = util.fs.glob(pointcloud_dir,{"od"})
transforms  = util.fs.glob(transform_dir,"dat")
images      = util.fs.glob(image_dir,"png")

if #pointclouds ~= #transforms or #pointclouds ~= #images then 
   print("warning not the same number of pointclouds and transforms")
end

log.trace("building tree ...")log.tic()
tree = octomap.ColorTree.new(res)
log.trace(" - in ".. log.toc())

for i,fname in pairs(pointclouds) do 
   transfname = transforms[i]
   imagefname = images[i]

   printf("[%d] loading %s and %s and %s",i,fname, transfname,imagefname)
   pc = PointCloud.PointCloud.new(fname)
   mat = torch.load(transfname)

   -- hack b.c z doesn't work yet
   mat[{3,4}] = -pc.minval[3]

   pc:set_pose_from_rotation_matrix(mat)
   pc:set_local_scan_center(pc:estimate_faro_pose())

   pc_points     = pc:get_global_points()
   pc_pose       = pc:get_global_scan_center()
   pc_max_radius = pc:get_max_radius()

   pc:load_rgb_map(imagefname)
   pc_rgb_points = pc:get_rgb()

   printf(" - adding points ...")log.tic()
   tree:add_points(pc_points,pc_pose,pc_max_radius,pc_rgb_points)
   printf(" - in %2.4fs",10e-4 * log.toc())

   tree:stats()
   pc = nil
   pc_points = nil
   collectgarbage()
end

-- write obj
log.trace("writing "..output_filename) log.tic()
tree:writeColoredPointsPly(output_filename)
log.trace(" - in ".. log.toc())
