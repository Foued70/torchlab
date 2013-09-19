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
_G.p = PointCloud.PointCloud.new(input_filename)
log.trace(" - in ".. log.toc())

img = image.load(image_filename,"float") -- ,nil,"LAB")
xyz_map = p:get_xyz_map()
index,mask = p:get_index_and_mask()

mask = mask:eq(0)

npts = mask:sum()
print(npts, p.points:size(1))
if (npts ~= p.points:size(1)) then 
   error("something doesn't sync with the image data")
end

rgb = torch.FloatTensor(3,npts)

rgb[1] = img[1][mask]
rgb[2] = img[2][mask]
rgb[3] = img[3][mask]

rgb = rgb:transpose(1,2):contiguous():mul(255)

_G.rgb = rgb

scanner_pose = p:estimate_faro_pose()

max_radius = torch.max(p.radius)

log.trace("building tree ...")log.tic()
_G.t = octomap.ColorTree.new(res)
t:add_points(p.points,scanner_pose,max_radius,rgb)
log.trace(" - in ".. log.toc())

t:stats()
t:info()

-- write obj
log.trace("writing "..output_filename) log.tic()
t:writeColoredPointsPly(output_filename)
log.trace(" - in ".. log.toc())
