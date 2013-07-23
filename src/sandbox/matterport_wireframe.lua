-- Class()
path = require 'path'

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
           "../tmp/test/96_spring_kitchen/nodal_ninja/",
           'base directory for images placed in sweep_1,2, etc.')
cmd:option('-matter_dir',
           "../tmp/test/96_spring_kitchen/blonde-beach-9765/",
           -- "../tmp/test/96_spring_kitchen/raw_scan/",
           "Directory for matterport data")
cmd:option('-obj_file',
--            "../tmp/test/96_spring_kitchen/blonde-beach-table.obj",
            "../tmp/test/96_spring_kitchen/blonde-beach-cube-new.obj",
--             "../tmp/test/96_spring_kitchen/blonde-beach-cube-flip.obj",
--             "../tmp/test/96_spring_kitchen/blonde-beach-clean.obj",
           "Directory for obj to retexture")


cmd:text()

-- parse input params
params     = cmd:parse(process.argv)

matter_dir = params.matter_dir
obj_file   = params.obj_file

matter_pose_fname = util.fs.glob(matter_dir,"texture_info.txt")

if #matter_pose_fname > 0 and util.fs.is_file(matter_pose_fname[1]) then
   matter_pose_fname = matter_pose_fname[1]
   printf("using : %s", matter_pose_fname)
   mpfile = model.Matterport_PoseFile.new(matter_pose_fname)
   poses = mpfile.poses
   
else
   error("Can't find the pose file (set -matter_dir correctly)")
end

if util.fs.is_file(obj_file) then
   printf("using : %s", obj_file)
   scan = data.Obj.new(obj_file)
else
   error("Can't find the obj file (set -obj_file correctly)")
end

views = {}

test_global_rotation = true

-- load sweeps
for sweep_no = 1,#poses do
   local pose = poses[sweep_no]

   -- load matter_view
   local matter_fname  = util.fs.glob(matter_dir , pose.name)[1]
   local matter_image  = image.load(matter_fname)
   local matter_width  = matter_image:size(3)
   local matter_height = matter_image:size(2)
   matter_image = nil
   collectgarbage()
   
   local matter_hfov   = matter_width  * pose.degrees_per_px_x * d2r
   local matter_vfov   = matter_height * pose.degrees_per_px_y * d2r

   local matter_proj   = 
      projection.SphericalProjection.new(
         matter_width,
         matter_height,
         matter_hfov,
         matter_vfov,
         matter_width * pose.center_u,
         matter_height * pose.center_v
      )
   local ea = geom.quaternion.to_euler_angle(pose.local_to_global_rotation)
   log.trace(ea)
   local matter_gnom = 
      projection.GnomonicProjection.new(
         matter_height, matter_height,
         matter_vfov, matter_vfov)
   matter_gnom:set_lambda_phi(ea[2],ea[1])

   local remapper = projection.Remap.new(matter_proj,matter_gnom)

   local matter_view   = 
      model.Photo.new(pose.local_to_global_position,
                     pose.local_to_global_rotation,
                     matter_hfov,
                     matter_vfov)  

   matter_view:set_image_path(matter_fname)
   matter_view:set_projection(matter_proj)
   matter_view.remapper = remapper
   
   table.insert(views,matter_view)
end

-- setup a "scan" object
scan.views = views

rt = retex.TextureBuilder.new(scan,{ppm=100})

for fid = 1,scan.n_faces do 
   -- rt:test_face_to_texture(fid)

   face_xyz = rt:xyz_wireframe(fid)

   retexts = {}
   for vi = 1,#views do 
      local view = views[vi]
      woff, _ , wmask = view:global_xyz_to_offset_and_mask(face_xyz)
      invmask = wmask:eq(0)
      if (invmask:sum() > 0) then 
         -- print(" - Computing")
         woff = woff[invmask]
      
         img = view:get_image()
         for cid = 1,1 do 
             c  = img[cid]
             c:resize(c:nElement())
             c[woff] = 1 -- view.color[cid]
         end
      else
         -- print(" - Skipping")
      end
   end
end

-- display
for vid = 1,#scan.views do
   local view = scan.views[vid]

   local base_name = path.normalize(matter_dir):gsub("/","-") .. 
      "_-_" .. path.basename(obj_file):gsub("%.[^%.]+$","")

   local pano = scan.views[vid]:get_image()
   local pano_name = string.format("%s_-_panorama_wireframe_%02d.png",base_name,vid)
   log.trace("saving:",pano_name)
   image.save(pano_name,pano)
   -- image.display(pano)

   local gnom = view.remapper:remap(view:get_image())
   local gnom_name = string.format("%s_-_gnomonic_wireframe_%02d.png",base_name,vid)
   log.trace("saving:",gnom_name)
   image.save(gnom_name,gnom)
   -- image.display(gnom)

end 
