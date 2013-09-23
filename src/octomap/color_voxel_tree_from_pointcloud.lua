cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('PointCloud to voxel grid')
cmd:text()
cmd:text('Options')
cmd:option('-pcloudfile',        '', 'xyz pointcloud to process')
cmd:option('-imagefile',         '', 'color corresponding to pointcloud')
cmd:option('-res',           '0.05', 'resolution of voxel grid')
cmd:option('-outfile', 'output.ply', 'ouput .ply file')

cmd:text()

arg = ''

-- parse input params
params = cmd:parse(process.argv)

input_filename  = params.pcloudfile
image_filename  = params.imagefile
output_filename = params.outfile
res             = tonumber(params.res)

log.trace("loading ".. input_filename)
log.tic()
_G.pc = PointCloud.PointCloud.new(input_filename)
log.trace(" - in ".. log.toc())

printf(" - loading image %s", image_filename)
pc:load_rgb_map(image_filename)

log.trace("building tree ...")log.tic()
_G.tree = octomap.ColorTree.new(res)
log.trace(" - in ".. log.toc())

-- hack for faro
pc:set_local_scan_center(pc:estimate_faro_pose())

printf(" - adding points ...")log.tic()
tree:add_pointcloud(pc)
printf(" - in %2.4fs",10e-4 * log.toc())


tree:stats()
tree:info()

-- write obj
log.trace("writing "..output_filename) log.tic()
tree:writeColoredPointsPly(output_filename)
log.trace(" - in ".. log.toc())
