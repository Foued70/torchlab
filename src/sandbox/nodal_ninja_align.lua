-- Class()

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
           "../tmp/test/96_spring_kitchen/nodal_ninja/",
           'base directory for scan')
cmd:option('-matter_dir',
           "../tmp/test/96_spring_kitchen/blonde-beach-9765/",
           "Directory for matterport data (relative to scan_dir)")

cmd:option('-sweep_prefix',
           "sweep_",
           "Directory prefix for DSLR image sweeps (relative to scan_dir)")
cmd:text()

-- parse input params
params = cmd:parse(process.argv)

scan_dir = params.scan_dir
matter_dir = params.matter_dir

matter_pose_fname = util.fs.glob(matter_dir,"texture_info.txt")

if #matter_pose_fname > 0 then
   matter_pose_fname = matter_pose_fname[1]
   printf("using : %s", matter_pose_fname)
end

poses = model.Matterport_PoseFile.new(matter_pose_fname).poses

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

   pose = poses[sweep_no]

   -- matterport texture
   matter_texture_fname = util.fs.glob(matter_dir,poses[sweep_no].name)

   if #matter_texture_fname > 0 then
      matter_texture_fname = matter_texture_fname[1]
      printf("using : %s", matter_texture_fname)
   end

   matter_texture = image.load(matter_texture_fname)

   mp_width      = matter_texture:size(3)
   mp_height     = matter_texture:size(2)

   mp_rad_per_px_x = pose.degrees_per_px_x * d2r
   mp_rad_per_px_y = pose.degrees_per_px_y * d2r
   mp_hfov = mp_width * mp_rad_per_px_x
   mp_vfov = mp_height * mp_rad_per_px_y

   mp_cx   = mp_width  * pose.center_u
   mp_cy   = mp_height * pose.center_v

   -- load DSLR image

   images = util.fs.glob(sweep_dir .. sweep_no, ".JPG")

   img = image.load(images[1])

   width  = img:size(3)
   height = img:size(2)

   proj_from = projection.GnomonicProjection.new(width,height,hfov,vfov)
   proj_to   = 
      projection.SphericalProjection.new(mp_width,
                                         mp_height,
                                         mp_hfov,
                                         mp_vfov,
                                         mp_cx,
                                         mp_cy)

   p("Testing Image Projection")

   log.tic()

   indices     = {}
   masks       = {}
   out_images  = {}

   rect_to_sphere = projection.Remap.new(proj_from,proj_to)

   time = log.toc()
   printf(" - make map %2.4fs", time)
   log.tic()
   offset = offsets[sweep_no]
   lambda = offset.initial
   phi    = 0 

   for i = 1,#images do
      log.tic()
      img    = image.load(images[i])
      proj_from:set_lambda_phi(lambda,phi)
      index1D,stride,mask = rect_to_sphere:get_offset_and_mask(force)
      img_out = rect_to_sphere:remap(img)
      table.insert(indices,index1D)
      table.insert(masks,mask)
      table.insert(out_images,img_out)
      
      lambda = lambda + offset.delta[i]
      printf(" - reproject %2.4fs", log.toc())
      collectgarbage()
   end

   -- blend
   invert = true
   blend_image = util.alpha_masks.blend(out_images,masks,invert)

   orig_texture = matter_texture:clone()

   matter_texture:add(-1, blend_image)

   collectgarbage()

   matter_texture:add(-matter_texture:min())
   matter_texture:mul(1/matter_texture:max())

   blendOutFname = string.format("blend_sweep_%d.png",sweep_no)
   log.trace("Saving", blendOutFname)
   image.save(blendOutFname,blend_image)
   -- image.display(blend_image)

   matterOutFname = string.format("matter_sweep_%d.png",sweep_no)
   log.trace("Saving", matterOutFname)
   image.save(matterOutFname,matter_texture)
   -- image.display(matter_texture)

end
