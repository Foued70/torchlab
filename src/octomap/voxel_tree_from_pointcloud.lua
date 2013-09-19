cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('PointCloud to voxel grid')
cmd:text()
cmd:text('Options')
cmd:option('-pcloudfile', '', 'xyz pointcloud to process')
cmd:option('-res', '0.05', 'resolution of voxel grid')
cmd:option('-outfile', 'output.obj', 'ouput .obj file')

cmd:text()

arg = ''

-- parse input params
params = cmd:parse(process.argv)

input_filename  = params.pcloudfile
output_filename = params.outfile
res             = tonumber(params.res)

log.trace("loading ".. input_filename)
log.tic()
_G.p = PointCloud.PointCloud.new(input_filename)
log.trace(" - in ".. log.toc())

scanner_pose = p:estimate_faro_pose()

max_radius = torch.max(p.radius)

log.trace("building tree ...")log.tic()
_G.t = octomap.Tree.new(res)
t:add_points(p.points,scanner_pose,max_radius)
log.trace(" - in ".. log.toc())

t:stats()
t:info()

_G.thres = t:get_thresholds()

-- write obj
log.trace("writing "..output_filename) log.tic()
t:writeObjCubes(output_filename)
log.trace(" - in ".. log.toc())
