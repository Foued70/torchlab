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
cmd:option('-scan_dir',
           "../data/test/96_spring_kitchen/nodal_ninja/",
           'base directory for scan')
cmd:option('-matter_dir',
           "../data/test/96_spring_kitchen/blonde-beach-9765/",
           "Directory for matterport data (relative to scan_dir)")

cmd:option('-sweep_prefix',
           "sweep_",
           "Directory prefix for DSLR image sweeps (relative to scan_dir)")
cmd:text()

-- parse input params
params = cmd:parse(arg)

scan_dir = params.scan_dir

matter_dir = params.matter_dir

matter_pose_fname = util.fs.glob(matter_dir,"texture_info.txt")

if #matter_pose_fname > 0 then
   matter_pose_fname = matter_pose_fname[1]
   printf("using : %s", matter_pose_fname)
end

poses = model.mp.load_poses(matter_pose_fname)

sweep_dir  = scan_dir .. "/" .. params.sweep_prefix

force  = true

-- images are vertical
vfov = (97.0/180) * pi
hfov = (74.8/180) * pi

offsets = {
   -- sweep 1
   {
      delta = torch.Tensor({1.0472, 1.0422, 1.0322, 1.0472, 1.0472, 1.0672}),
      initial = 1.0156043690219
   },
   -- sweep 2
   {
      delta = torch.Tensor({1.0472, 1.0497, 1.0472, 1.0597, 1.0547, 1.0247}),
      initial = 6.0183196949309
   },
   -- sweep 3
   {
      delta = torch.Tensor({1.0447, 1.0447, 1.0472, 1.0622, 1.0447, 1.0397}),
      initial = 5.8524838277737
   },
   -- sweep 4
   {
      delta = torch.Tensor({1.0397, 1.0372, 1.0472, 1.0497, 1.0472, 1.0622}),
      initial = 3.5634463280735
   }
}

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
   offset = offsets[sweep_no]
   lambda = offset.initial
   phi    = 0 

   local pr = xlua.Profiler()

   for i = 1,#images do
      pr:start("image")
      pr:start("load")
      img    = image.load(images[i])
      pr:lap("load")
      pr:start("project")
      pr:start("set lambda phi")
      proj_from:set_lambda_phi(lambda,phi)
      pr:lap("set lambda phi")
      pr:start("get index")
      index1D,stride,mask = rect_to_sphere:get_offset_and_mask(force)
      pr:lap("get index")
      pr:start("rect to sphere")
      img_out = rect_to_sphere:remap(img)
      pr:lap("rect to sphere")
      pr:lap("project")
      pr:start("store")
      table.insert(indices,index1D)
      table.insert(masks,mask)
      table.insert(out_images,img_out)
      pr:lap("store")
      time = sys.toc()
      printf(" - reproject %2.4fs", time)
      sys.tic()

      lambda = lambda + offset.delta[i]
      pr:lap("image")
      pr:printAll()
   end

   -- blend
   invert = true
   blend_image = blend(out_images,masks,invert)

   orig_texture = matter_texture:clone()

   matter_texture = matter_texture - blend_image
   collectgarbage()

   matter_texture:add(-matter_texture:min())
   matter_texture:mul(1/matter_texture:max())

   win = image.display{image={blend_image,orig_texture,matter_texture},nrow=1}
   image.save(string.format("align_sweep_%d.png",sweep_no),win.image)
end
