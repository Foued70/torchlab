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
cmd:option('-matter_dir',
           "../data/test/96_spring_kitchen/blonde-beach-9765/",
           "Directory for matterport data")
cmd:option('-obj_file',
            "../data/test/96_spring_kitchen/blonde-beach-cube-flip.obj",
--            "../data/test/96_spring_kitchen/blonde-beach-clean.obj",
           "Directory for obj to retexture")


cmd:text()

-- parse input params
params     = cmd:parse(arg)

matter_dir = params.matter_dir
obj_file   = params.obj_file

scale = 1

matter_pose_fname = util.fs.glob(matter_dir,"texture_info.txt")

if #matter_pose_fname > 0 and paths.filep(matter_pose_fname[1]) then
   matter_pose_fname = matter_pose_fname[1]
   printf("using : %s", matter_pose_fname)
   local mpfile = model.Matterport_PoseFile.new(matter_pose_fname)
   poses = mpfile.poses
   
else
   error("Can't find the pose file (set -matter_dir correctly)")
end

if paths.filep(obj_file) then
   printf("using : %s", obj_file)
   scan = data.Obj.new(obj_file)
else
   error("Can't find the obj file (set -obj_file correctly)")
end

views = {}

test_global_orientation = true

-- load sweeps
for sweep_no = 1,1 do -- #poses do
   local pose = poses[sweep_no]

   -- load matter_view
   local matter_fname  = util.fs.glob(matter_dir , pose.name)[1]
   local matter_image  = image.load(matter_fname)
   local matter_width  = matter_image:size(3)
   local matter_height = matter_image:size(2)
   matter_image = nil
   collectgarbage()
   
   local matter_hfov   = matter_width * pose.degrees_per_px_x * math.pi/180
   local matter_vfov   = matter_height * pose.degrees_per_px_y * math.pi/180
   local matter_proj   = projection.SphericalProjection.new(matter_width,matter_height,matter_hfov,matter_vfov)
   local rot = pose.global_to_local_rotation:clone()
   local step = geom.quaternion.from_euler_angle(torch.Tensor({0,math.pi*0.1}))

   for p = -math.pi,math.pi,math.pi*0.1 do
      print("rot",rot)
      print("step",step)
      local matter_view = model.View.new(pose.global_to_local_position,
                                         rot,
                                         matter_hfov,
                                         matter_vfov)  
      rot = geom.quaternion.product(rot,step)
      matter_view:set_image_path(matter_fname,scale)
      matter_view:set_projection(matter_proj)
   
      table.insert(views,matter_view)
   end
   
end

-- setup a "scan" object
scan.views = views

rt = retex.TextureBuilder.new(scan,{ppm=100})

for fid = 1,4 do -- scan.n_faces do 
   rt:test_face_to_texture(fid)

   face_xyz = rt:xyz_wireframe(fid)

   retexts = {}
   for vi = 1,#views do 
      local view = views[vi]
      view.color = view.color or torch.rand(3)
      log.trace("face: ", fid, " view: ", vid)
      woff, _ , wmask = view:global_xyz_to_offset_and_mask(face_xyz,true)
      invmask = wmask:eq(0)
      if (invmask:sum() > 0) then 
         print(" - Computing")
         woff = woff[invmask]
      
         img = view:get_image()
         for cid = 1,1 do 
             c  = img[cid]
             c:resize(c:nElement())
             c[woff] = 1 -- view.color[cid]
         end
      else
         print(" - Skipping")
      end
   end
end

imgs = {}
sweep_no = 1
for vid = 1,#scan.views do
   euler = geom.quaternion.to_euler_angle(views[vid].global_to_local_orientation)
   euler = euler:squeeze():mul(180/math.pi)
   image.display{image=scan.views[vid]:get_image(), 
                 legend=string.format("Sweep: %d euler (% 3.2f % 3.2f % 3.2f)",vid, euler[1],euler[2],euler[3])}
end 
