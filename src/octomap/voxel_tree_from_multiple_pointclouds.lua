cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('PointCloud to voxel grid')
cmd:text()
cmd:text('Options')
cmd:option('-pclouddir', '', 'xyz pointcloud to process')
cmd:option('-datadir', '', 'pointcloud poses to process')
cmd:option('-res', '0.025', 'resolution of voxel grid')
cmd:option('-outfile', 'output.obj', 'ouput .obj file')

cmd:text()

arg = ''

-- parse input params
params = cmd:parse(process.argv)

pointcloud_dir  = params.pclouddir
transforms_dir  = params.datadir
res             = tonumber(params.res)
output_filename = params.outfile

pointclouds = util.fs.glob(pointcloud_dir,{"od","xyz"})
transforms  = util.fs.glob(transforms_dir,"dat")

if #pointclouds ~= # transforms then 
   print("warning not the same number of pointclouds and transforms")
end

log.trace("building tree ...")log.tic()
_G.tree = octomap.Tree.new(res)
log.trace(" - in ".. log.toc())

for i,fname in pairs(pointclouds) do 
   transfname = transforms[i]
   printf("[%d] loading %s and %s",i,fname, transfname)
   pc = PointCloud.PointCloud.new(fname)
   mat = torch.load(transfname)

   pc:set_pose_from_rotation_matrix(mat)
   pc:set_local_scan_center(pc:estimate_faro_pose())

   pc_points     = pc:get_global_points()
   pc_pose       = pc:get_global_scan_center()
   pc_max_radius = pc:get_max_radius()



   printf(" - adding points ...")log.tic()
   tree:add_points(pc_points,pc_pose,pc_max_radius)
   printf(" - in %2.4fs",10e-4 * log.toc())

   tree:stats()
   pc = nil
   pc_points = nil
   collectgarbage()
end

-- write obj
printf("writing %s",output_filename) log.tic()
tree:writeObjCubes(output_filename)
printf(" - in %2.4fs",log.toc()*10e-4)
