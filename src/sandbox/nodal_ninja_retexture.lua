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
cmd:option('-obj_file',
--            "../tmp/test/96_spring_kitchen/blonde-beach-table.obj",
            "../tmp/test/96_spring_kitchen/blonde-beach-cube-new.obj",
--             "../tmp/test/96_spring_kitchen/blonde-beach-cube-flip.obj",
--             "../tmp/test/96_spring_kitchen/blonde-beach-clean.obj",
           "Directory for obj to retexture")
cmd:text()

-- parse input params
params = cmd:parse(process.argv)

scan_dir   = params.scan_dir
matter_dir = params.matter_dir
obj_file   = params.obj_file

maxsize = 1024
ppm     = 100

matter_pose_fname = util.fs.glob(matter_dir,"texture_info.txt")

if #matter_pose_fname > 0 then
   matter_pose_fname = matter_pose_fname[1]
   printf("using : %s", matter_pose_fname)
end

if util.fs.is_file(obj_file) then
   printf("using : %s", obj_file)
   _G.scan = data.Obj.new(obj_file)
else
   error("Can't find the obj file (set -obj_file correctly)")
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

views     = {}

-- load sweeps
for sweep_no = 1,4 do

   pose = poses[sweep_no]

   -- gather and load DSLR images

   images = util.fs.glob(sweep_dir .. sweep_no, ".JPG")

   p("Testing Image Projection")

   offset = offsets[sweep_no]
   lambda = offset.initial
   phi    = 0 

   for i = 1,#images do
      log.tic()
      view   = 
         model.View.new(pose.local_to_global_position,
                        pose.local_to_global_rotation,
                        hfov,
                        vfov) 

      view:set_image_path(images[i],maxsize)

      proj_from = projection.GnomonicProjection.new(view.width,view.height,hfov,vfov)
      proj_from:set_lambda_phi(lambda,phi)

      view:set_projection(proj_from)

      table.insert(views,view)

      lambda = lambda + offset.delta[i]
      printf(" - make view %2.4fs", log.toc())
      collectgarbage()
   end
end

scan.views = views

texture = retex.TextureBuilder.new(scan,{ppm=ppm})

texture:buildAll()
