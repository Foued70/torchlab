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
   --    poses = model.mp.load_poses(matter_pose_fname)
else
   error("Can't find the pose file (set -matter_dir correctly)")
end

if paths.filep(obj_file) then
   printf("using : %s", obj_file)
   scan = model.Scan.new(dslr_dir,matter_pose_fname,obj_file)
else
   error("Can't find the obj file (set -obj_file correctly)")
end

poses = scan.poses

sweep_dir  = dslr_dir .. "/" .. params.sweep_prefix

force  = true
delta  = 2 * pi / 6

-- images are vertical
vfov = (97.11/180) * pi
hfov = (74.22/180) * pi


hard_coded = { delta,   0, -0.5*delta, -3*delta }
offset     = {   -20, -65,         35,      128 }

views = {}

-- load sweeps
for sweep_no = 1,4 do

   local pose = poses[sweep_no]
   -- matterport texture
   local matter_texture_fname = util.fs.glob(matter_dir,pose.name)

   if #matter_texture_fname > 0 then
      matter_texture_fname = matter_texture_fname[1]
      printf("using : %s", matter_texture_fname)
   end

   matter_texture = image.load(matter_texture_fname)

   local mp_width        = matter_texture:size(3)
   local mp_height       = matter_texture:size(2)

   local mp_rad_per_px_x = pose.degrees_per_px_x * d2r
   local mp_rad_per_px_y = pose.degrees_per_px_y * d2r
   local mp_hfov         = mp_width * mp_rad_per_px_x
   local mp_vfov         = mp_height * mp_rad_per_px_y

   local mp_cx           = mp_width  * pose.center_u
   local mp_cy           = mp_height * pose.center_v

   -- load DSLR image

   local images = util.fs.glob(sweep_dir .. sweep_no, {".jpg", ".JPG"})

   euler_angles = torch.Tensor(2,#images)
   local lambda = hard_coded[sweep_no]  + (offset[sweep_no] * mp_rad_per_px_x)
   local phi    = 0 -- - 5 * rad_per_px_y

   for i = 1,#images do
      euler_angles[1][i] = phi
      euler_angles[2][i] = lambda
      lambda = lambda + delta

   end

   -- compute orientation quaternions for each image in this sweep 
   q_rel_pose = geom.quaternion.from_euler_angle(euler_angles)
   q_global   = geom.quaternion.product(pose.rotation,q_rel_pose)

   for i = 1,#images do
      local view = model.View.new(pose.position,q_global[i],hfov,vfov)  
      table.insert(views,view)
   end
end

-- setup a "scan" object
scan.views = views
scan.view_frustrums = torch.Tensor(#views,views[1].frustrum:size(1),views[1].frustrum:size(2))

for i,v in pairs(views) do scan.view_frustrums[i] = v.frustrum ; v.frustrum = scan.view_frustrums[i]; end
scan.view_frustrums:resize(#views*views[1].frustrum:size(1),views[1].frustrum:size(2))

scan:get_model_data()

-- distance of each view frustum plane to each vertex 
function make_view_getter()
   local df    = torch.mm(scan.model_data.verts,scan.view_frustrums:t())
   local dfrs  = df:reshape(scan.model_data.n_verts,#scan.views,scan.views[1].n_planes)
   local dfrsm = dfrs:min(3):squeeze()
   
   local dbi   = dfrsm:gt(0)
   local r     = torch.range(1,#scan.views)

   df    = nil
   dfrs  = nil
   dfrsm = nil
   collectgarbage()
   
   return function (face_id)
      return r[dbi[scan.model_data.faces[face_id]:select(2,2)]:sum(1):gt(0)]
         end
end

get_views = make_view_getter()
p(get_views(100))
