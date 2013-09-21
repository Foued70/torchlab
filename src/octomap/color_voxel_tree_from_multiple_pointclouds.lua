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
cmd:option('-outfile',   'output.ply', 'output .ply file')
cmd:option('-dozhack', false, 'move points to 0 at min z')
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
print("image_dir: " .. image_dir)

transform_dir   = params.datadir
if transform_dir == '' then
   transform_dir = pointcloud_dir
end
print("transform_dir: " .. transform_dir)

output_filename = params.outfile
res             = tonumber(params.res)
do_z_hack       = params.dozhack

pointclouds = util.fs.glob(pointcloud_dir,{"od"})
transforms  = util.fs.glob(transform_dir,{"dat"})
images      = util.fs.glob(image_dir,{"png"})

load_transforms = false
load_images    = false

if (#transforms > 0) then 
   if (#pointclouds ~= #transforms) then 
      print("warning not the same number of pointclouds and transforms")
   else
      load_transforms = true
   end
end

if (#images > 0) then 
   if (#pointclouds ~= #images) then 
      print("warning not the same number of pointclouds and images")
   else
      load_images = true
   end
end

log.trace("building tree ...")log.tic()
_G.tree = octomap.ColorTree.new(res)
log.trace(" - in ".. log.toc())

for i,fname in pairs(pointclouds) do 

   printf("[%d] loading pointcloud %s",i,fname)
   pc = PointCloud.PointCloud.new(fname)

   if load_transforms then 
      trans_fname = transforms[i]
      printf(" - loading transform %s",trans_fname)
      mat = torch.load(trans_fname)
      if do_z_hack then 
         print(" - doing z hack")
         -- hack b.c z doesn't work yet
         mat[{3,4}] = -pc.minval[3]
      end
      pc:set_pose_from_rotation_matrix(mat)
   end

   if load_images then 
      image_fname = images[i]
      printf(" - loading image %s", image_fname)
      pc:load_rgb_map(image_fname)
   end

   -- hack for faro
   pc:set_local_scan_center(pc:estimate_faro_pose())

   printf(" - adding points ...")log.tic()
   tree:add_pointcloud(pc)
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
