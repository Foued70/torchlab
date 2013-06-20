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
           "../data/test/96_spring_kitchen/blonde-beach-cube-flip.obj",
           "Directory for obj to retexture")


cmd:text()

-- parse input params
params     = cmd:parse(arg)

dslr_dir   = params.dslr_dir
matter_dir = params.matter_dir
obj_file   = params.obj_file

scale = 0.1

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
   scan = data.Obj.new(obj_file)
else
   error("Can't find the obj file (set -obj_file correctly)")
end

sweep_dir  = dslr_dir .. "/" .. params.sweep_prefix

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

views = {}

test_global_orientation = false

-- load sweeps
for sweep_no = 1,4 do
   local pose = poses[sweep_no]

   -- load DSLR image

   local images = util.fs.glob(sweep_dir .. sweep_no, {".jpg", ".JPG"})
   sweep_offset = offsets[sweep_no]

   if not camera then 
      local img    = image.load(images[1])
      local width  = math.floor(img:size(3)*scale)
      local height = math.floor(img:size(2)*scale)
      camera = projection.RectilinearProjection.new(width,height,hfov,vfov)
   end

   euler_angles = torch.Tensor(2,#images)
   local lambda = sweep_offset.initial
   local phi    = 0 

   for i = 1,#images do
      euler_angles[1][i] = phi
      euler_angles[2][i] = lambda
      lambda = lambda + sweep_offset.delta[i] -- delta[i] moves from i -> i+1
   end

   -- compute orientation quaternions for each image in this sweep 
   q_rel_pose = geom.quaternion.from_euler_angle(euler_angles)
   q_global   = geom.quaternion.product(geom.quaternion.conjugate(pose.rotation),q_rel_pose)

   -- TEST the orientations (setup) 
   if test_global_orientation then 
      if not spherical_remapper then 
         proj_from = projection.GnomonicProjection.new(camera.width,camera.height,hfov,vfov)
         proj_to   = projection.SphericalProjection.new(1024,512,2*math.pi,math.pi)
         spherical_remapper = projection.Remap.new(proj_from,proj_to)
      end
      indices    = {}
      masks      = {}
      out_images = {}
   end

   for i = 1,#images do
      local view = model.View.new(pose.position,geom.quaternion.conjugate(q_global[i]),hfov,vfov)  
      view.position[3] = view.position[3] + 0.25
      view:set_image_path(images[i],scale)
      view:set_projection(camera)
      table.insert(views,view)

      -- TEST the orientations (collect)
      if test_global_orientation then 
         img = view:get_image()
         
         ea = geom.quaternion.to_euler_angle(view.orientation)
         proj_from:set_lambda_phi(ea[2],ea[1])

         index1D,stride,mask = spherical_remapper:get_offset_and_mask(force)
         img_out = spherical_remapper:remap(img)

         table.insert(indices,index1D)
         table.insert(masks,mask)
         table.insert(out_images,img_out)
         -- image.display{image=img_out,legend=string.format("Sweep %d Image: %d", sweep_no, i)}         
      end
   end

   -- Test the orientations (blend)
   if test_global_orientation then 
      invert = true
      blend_image = blend(out_images,masks,invert)
      image.display{image=blend_image, legend=string.format("Sweep %d", sweep_no)}
   end
end

-- setup a "scan" object
scan.views = views
scan.view_frustrums = torch.Tensor(#views,views[1].frustrum:size(1),views[1].frustrum:size(2))

for i,v in pairs(views) do scan.view_frustrums[i] = v.frustrum ; v.frustrum = scan.view_frustrums[i]; end
scan.view_frustrums:resize(#views*views[1].frustrum:size(1),views[1].frustrum:size(2))

-- distance of each view frustum plane to each vertex 
function make_view_getter(self)
   local df    = torch.mm(self.verts,self.view_frustrums:t())
   local dfrs  = df:reshape(self.n_verts,#self.views,self.views[1].n_planes)
   local dfrsm = dfrs:min(3):squeeze()
   
   local dbi   = dfrsm:gt(0)
   local r     = torch.range(1,#self.views)

   df    = nil
   dfrs  = nil
   dfrsm = nil
   collectgarbage()
   
   return function (face_id)
      return r[dbi[self.faces[face_id]:select(2,2)]:sum(1):gt(0)]
         end
end

get_view = make_view_getter(scan)

-- produce list of points in xyz for painting edges of a face at a
-- certain pixels per meter.
-- TODO move to scan object
function face_wireframe_xyz(face_verts,n_verts,ppm)

   n_verts = n_verts or face_verts:size(1)

   ppm = ppm or 100   -- pixels per meter

  -- don't need homogeneous coords here
  local f = face_verts:narrow(1,1,n_verts):narrow(2,1,3)
  local directions = torch.Tensor(n_verts, 3)
  local pvert      = f[n_verts]
  for vi = 1,n_verts do
     local cvert = f[vi]
     directions[vi]:copy(cvert - pvert)
     pvert = cvert
  end

  local lengths = directions:norm(2,2)
  local n_px    = torch.mul(lengths,ppm):ceil()
  local steps    = torch.cdiv(directions,n_px:expand(directions:size()))

  local total_pts   = n_px:sum()
  local pts     = torch.Tensor(total_pts, 3)
  local pi      = 1
  prev  = f[n_verts]
  for vi = 1,n_verts do
     local step = steps[vi]
     local nptx = n_px[vi]:squeeze() 
     for s = 1,nptx do
        pts[pi] = prev + step     
        prev    = pts[pi]
        pi      = pi + 1
     end
     -- local err = torch.abs(prev - f[vi])
     -- printf("error[%d]: %f, %f, %f",vi, err[1],err[2],err[3])
     
  end
  return pts
end

-- p(get_view(100))
rt = retex.TextureBuilder.new(scan,{ppm=100})

vseen = torch.ByteTensor(#scan.views):fill(0)
for fid = 1,4 do -- scan.n_faces do 
   -- xyz = rt:compute_xyz(fid)
   -- rt:test_face_to_texture(fid)
   face_xyz = face_wireframe_xyz(scan.face_verts[fid],scan.n_verts_per_face[fid])

   vs = get_view(fid)

   retexts = {}
   for vi = 1,vs:size(1) do 
      local vid = vs[vi]
      vseen[vid] = 1
      local view = scan.views[vid]
      view.color = view.color or torch.rand(3)
      log.trace("face: ", fid, " view: ", vid)
      woff, _ , wmask = view:global_xyz_to_offset_and_mask(face_xyz)
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
   table.insert(imgs,scan.views[vid]:get_image())
   if (math.mod(vid,6) == 0) then 
      image.display{image=imgs, legend=string.format("Sweep: %d",sweep_no)}
      imgs = {}
      sweep_no = sweep_no + 1
   end 
end
