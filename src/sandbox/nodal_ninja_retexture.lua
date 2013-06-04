-- Class()

require 'image'

dofile 'util.lua'

pi = math.pi
pi2 = pi * 0.5

d2r = math.pi / 180

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Align images in a sweep')
cmd:text()
cmd:text('Options')
cmd:option('-dslr_dir',
           "../data/test/96_spring_kitchen/nodal_ninja/",
           'base directory for images placed in sweep_1,2, etc.')
cmd:option('-sweep_prefix',
           "sweep_",
           "Directory prefix for DSLR image sweeps (relative to scan_dir)")
cmd:option('-matter_dir',
           "../data/test/96_spring_kitchen/blonde-beach-9765/",
           "Directory for matterport data")
cmd:option('-obj_file',
           "../data/test/96_spring_kitchen/blonde-beach-clean.obj",
           "Directory for obj to retexture")


cmd:text()

-- parse input params
params = cmd:parse(arg)

dslr_dir   = params.dslr_dir
matter_dir = params.matter_dir
obj_file   = params.obj_file

matter_pose_fname = util.fs.glob(matter_dir,"texture_info.txt")

if #matter_pose_fname > 0 and paths.filep(matter_pose_fname[1]) then
   matter_pose_fname = matter_pose_fname[1]
   printf("using : %s", matter_pose_fname)
   poses = model.mp.load_poses(matter_pose_fname)
else
   error("Can't find the pose file (set -matter_dir correctly)")
end

if paths.filep(obj_file) then   
   printf("using : %s", obj_file)
   model = model.Scan.new(dslr_dir,matter_pose_fname,obj_file)
else
   error("Can't find the obj file (set -obj_file correctly)")
end

sweep_dir  = dslr_dir .. "/" .. params.sweep_prefix

force  = true
delta  = 2 * pi / 6

-- images are vertical
vfov = (97.11/180) * pi
hfov = (74.22/180) * pi

hard_coded = { delta,   0, -0.5*delta, -3*delta } 
offset     = {   -20, -65,         35,      128 }
-- load sweeps
for sweep_no = 1,4 do

   -- matterport texture
   matter_texture_fname = util.fs.glob(matter_dir,poses[sweep_no].name)

   if #matter_texture_fname > 0 then
      matter_texture_fname = matter_texture_fname[1]
      printf("using : %s", matter_texture_fname)
   end

   matter_texture = image.load(matter_texture_fname)

   mp_width      = matter_texture:size(3)
   mp_height     = matter_texture:size(2)

   mp_rad_per_px_x = poses[sweep_no].degrees_per_px_x * d2r
   mp_rad_per_px_y = poses[sweep_no].degrees_per_px_y * d2r
   mp_hfov = mp_width * mp_rad_per_px_x
   mp_vfov = mp_height * mp_rad_per_px_y

   mp_cx   = mp_width  * poses[sweep_no].center_u
   mp_cy   = mp_height * poses[sweep_no].center_v

   -- load DSLR image

   images = util.fs.glob(sweep_dir .. sweep_no, ".JPG")

   img = image.load(images[1])

   width = img:size(3)
   height = img:size(2)


   proj_from = projection.GnomonicProjection.new(width,height,hfov,vfov)
   proj_to   = projection.SphericalProjection.new(mp_width,mp_height,mp_hfov,mp_vfov)

   p("Testing Image Projection")

   sys.tic()

   indices     = {}
   masks       = {}
   out_images  = {}

   rect_to_sphere = projection.Remap.new(proj_from,proj_to)

   time = sys.toc()
   printf(" - make map %2.4fs", time)
   sys.tic()
   lambda = hard_coded[sweep_no]  + (offset[sweep_no] * mp_rad_per_px_x)
   phi    = 0 -- - 5 * rad_per_px_y

   for i = 1,#images do

      img    = image.load(images[i])

      proj_from:set_lambda_phi(lambda,phi)
      index1D,stride,mask = rect_to_sphere:get_index_and_mask(force)
      img_out = rect_to_sphere:remap(img)

      table.insert(indices,index1D)
      table.insert(masks,mask)
      table.insert(out_images,img_out)

      time = sys.toc()
      printf(" - reproject %2.4fs", time)
      sys.tic()

      lambda = lambda + delta
   end

   -- blend

   blend_image = blend(out_images,masks)

   orig_texture = matter_texture:clone()

   matter_texture = matter_texture - blend_image
   collectgarbage()

   matter_texture:add(-matter_texture:min())
   matter_texture:mul(1/matter_texture:max())

   win = image.display{image={blend_image,orig_texture,matter_texture},nrow=1}
   image.save(string.format("align_sweep_%d.png",sweep_no),win.image)
end