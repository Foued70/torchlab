require 'image'

sys.tic()

local pi = math.pi
local pi2 = pi * 0.5

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Compute image projections')
cmd:text()
cmd:text('Options')
cmd:option('-scandir', 'images/', 'directory with the images to load')
cmd:option('-outfile', 'skybox', 'basename for 6 output images for skybox')
cmd:option('-size', '1024', 'size in pixels of side of skybox cube')
cmd:text()

-- parse input params
params = cmd:parse(arg)

scandir  = params.scandir
out_size   = params.size
out_file   = params.outfile

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

p("Creating Skybox Projection")
time_prep = sys.toc()
printf(" - load image in %2.4fs", time_prep)
sys.tic()

-- make a skybox
centers = {{0,0},{pi2,0},{pi,0},{-pi2,0},{0,pi2},{0,-pi2}}
-- this naming comes from unity and is from the outside looking in
names   = {"front", "left", "back", "right", "down", "up"}

remappers = {}
for _,off in ipairs(centers) do 

   local proj_to = projection.GnomonicProjection.new(out_size,out_size,
                                               out_fov,out_fov,
                                               out_size/2,out_size/2,
                                               off[1],off[2])
   
   table.insert(remappers, projection.Remap.new(proj_from,proj_to))
end

time_map = sys.toc()
printf(" - make map %2.4fs", time_map)
sys.tic()

for i,r in ipairs(remappers) do 
   local face = r:remap(img)
   local outf = out_file .."_"..names[i]..".jpg"
   image.display(face)
   printf(" - saving: %s", outf)
   image.save(outf,face)
end

time_reproject = sys.toc()
printf(" - reproject %2.4fs", time_reproject)
sys.tic()


printf(" - Total %2.4fs", time_prep + time_map + time_reproject)
