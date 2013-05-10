local projection_util = util.projection
require 'image'

local pi = math.pi
local pi2 = pi * 0.5

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Compute image projections')
cmd:text()
cmd:text('Options')
cmd:option('-scandir', 'images/', 'directory with the images to load')
cmd:text()

-- parse input params
params = cmd:parse(arg)

scandir  = params.scandir

-- load images
if not images then
   images = {}
   if not paths.dirp(scandir) then
      error("Must set a valid path to directory of images to process default -scandir images/")
   end
   files = paths.files(scandir)
   files() -- .
   files() -- ..
   for f in files do
      if f == ".DS_Store" then -- exclude OS X automatically-created backup files
         printf("--- Skipping .DS_Store file")
      elseif (f:gmatch("_texture_info.txt")()) then
         pose_file = scandir.."/"..f
      elseif (f:gmatch("jpg$")() or f:gmatch("png$")()) then
         local imgfile = scandir.."/"..f
         table.insert(images, imgfile)
         printf("Found : %s", imgfile)
      end
   end
end
collectgarbage()

poses = util.mp.load_poses(pose_file)

img = image.load(images[1])

local width      = img:size(3)
local height     = img:size(2)
-- images are vertical

hfov = poses[1].degrees_per_px_x * width * math.pi / 180
vfov = poses[1].degrees_per_px_y * height * math.pi / 180
cx   = width * poses[1].center_u
cy   = height * poses[1].center_v
pi       = math.pi
pi2      = math.pi/2
out_fov  = pi2
out_size = 512

proj_from = projection.SphericalProjection.new(width,height, hfov,vfov,cx,cy)

proj_to   = projection.RectilinearProjection.new(out_size,out_size,out_fov,out_fov)

angle_map = proj_to:angles_map()

p("Testing Image Projection")

sys.tic()
-- make a skybox
index_and_masks = {}
centers = {{0,0},{pi2,0},{pi,0},{-pi2,0},{0,pi2},{0,-pi2}}
for _,off in ipairs(centers) do 
   local angles = angle_map:clone()
   angles[1]:add(off[1])
   angles[2]:add(off[2])
   local sign = torch.sign(angles)
   angles:abs()
   angles[1][angles[1]:gt(pi)] =  
      angles[1][angles[1]:gt(pi)]:add(-2*pi)
   angles[2][angles[2]:gt(pi2)] = 
      angles[2][angles[2]:gt(pi2)]:add(-pi)
   angles:cmul(sign)
   table.insert(index_and_masks,
                proj_from:angles_to_index1D_and_mask(angles))
end

local perElement = index_and_masks[1].index1D:nElement()

time = sys.toc()
printf(" - make map %2.4fs %2.4es per px", time, time*perElement)
sys.tic()

for i = 1,#images do
   img = image.load(images[i])
   img_out = {}
   for _,idx in ipairs(index_and_masks) do 
      table.insert(img_out, projection_util.remap(img,idx))
   end

   time = sys.toc()
   printf(" - reproject %2.4fs %2.4es per px", time, time*perElement)
   sys.tic()

   image.display{image=img_out,nrow=4}
end
