-- Class()

require 'image'

dofile 'blend.lua'

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

cmd:option("-lens_type", "sigma_10_20mm", "data for image dewarping")

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

poses = util.mp.load_poses(matter_pose_fname)

sweep_dir  = scan_dir .. "/" .. params.sweep_prefix

force  = true
delta  = 2 * pi / 6

hard_coded = { delta,
               0, 
               -0.5*delta,
               -3*delta } 
offset = { - 20, 
           - 65,
           35,
           128}
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

   images = util.util.file_match(sweep_dir .. sweep_no, ".JPG")

   img = image.load(images[1])

   width = img:size(3)
   height = img:size(2)

   -- images are vertical
   vfov = (97.11/180) * pi
   hfov = (74.22/180) * pi

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