require 'image'
require 'nn'

LensSensor = util.LensSensor
projection = util.projection
-- mp = util.mp

d2r = math.pi / 180

-- COPY from util/mp.lua until things are settled
-- <texture filename> <qx> <qy> <qz> <qw> <tx> <ty> <tz> <center u>
-- <center v> <degrees per px x> <degrees per px y> 
function load_poses(posefile)
  if not paths.filep(posefile) then return nil end
  
  print('Loading poses from', posefile)
  local mp_poses = {}
  
  for line in io.lines(posefile) do
    local pose_values = {}
    
    for value in line:gmatch("%S+") do    
      table.insert(pose_values, value)
    end

    local pose = {}
    pose.name = pose_values[1]
    pose.rotation = torch.Tensor({pose_values[2], pose_values[3], pose_values[4], pose_values[5]})
    pose.position = torch.Tensor({pose_values[6], pose_values[7], pose_values[8]})
    pose.center_u = pose_values[9]
    pose.center_v = pose_values[10]
    pose.degrees_per_px_x = pose_values[11]
    pose.degrees_per_px_y = pose_values[12]        
    table.insert(mp_poses, pose)
  end
  print(#mp_poses, 'poses loaded.')
  
  return mp_poses
end

-- Really need a FILE GLOB...
function file_match(dir,match) 
   if not paths.dirp(dir) then return nil end
   out = {}
   for f in paths.files(dir) do 
      if f:gmatch(match)() then
         table.insert(out,dir .. "/" .. f) 
      end 
   end
   return out
end

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Compute Perspective 3 points algorithm')
cmd:text()
cmd:text('Options')
cmd:option('-scan_dir',
           "../data/test/96_spring_kitchen/",
           'base directory for scan')
cmd:option('-matter_dir',
           "raw_scan/",
           "Directory for matterport data (relative to scan_dir)")

cmd:option('-sweep_prefix',
           "sweep_",
           "Directory prefix for DSLR image sweeps (relative to scan_dir)")

cmd:option("-lens_type", 
           "nikon_10p5mm_r2t_full",
           "data for image dewarping")

cmd:text()

-- parse input params
params = cmd:parse(arg)

scan_dir = params.scan_dir

matter_dir = scan_dir .. params.matter_dir

matter_pose_fname = file_match(matter_dir,"texture_info.txt")
if #matter_pose_fname > 0 then   
   matter_pose_fname = matter_pose_fname[1]
   printf("using : %s", matter_pose_fname)
end

poses = load_poses(matter_pose_fname)

sweep_dir  = scan_dir .. params.sweep_prefix

-- load lens calibration
lens = LensSensor.new(params.lens_type)

preprocess = nn.Sequential()
preprocess:add(nn.SpatialContrastiveNormalization(1,image.gaussian(9)))

-- Could loop over sweeps here.

-- load sweeps
sweep_no = 1

-- matterport texture
matter_texture_fname = file_match(matter_dir,poses[sweep_no].name)

if #matter_texture_fname > 0 then   
   matter_texture_fname = matter_texture_fname[1]
   printf("using : %s", matter_texture_fname)
end

matter_texture = image.load(matter_texture_fname)
mpout  = preprocess:forward(image.rgb2lab(matter_texture):narrow(1,1,1))
mpvfov = d2r * 0.5 * 512 * poses[1].degrees_per_px_y
image.display(mpout)

-- DSLR images

sweep_image_fname = file_match(sweep_dir .. sweep_no, ".jpg")

for ii = 1,#sweep_image_fname do 
   
   rawimg = image.load(sweep_image_fname[ii])

   if ii == 1 then
      lens:add_image(rawimg)

      -- find proper size for projection to match matterport fov

      scale = (512 / lens.image_h ) * ( lens.vfov / mpvfov)


      map = lens:make_projection_map("sphere",scale)
   end

   imgout = preprocess:forward(image.rgb2lab(projection.remap(rawimg,map)):narrow(1,1,1)):clone()

   rawimg = nil
   collectgarbage()
   image.display(imgout)
end
