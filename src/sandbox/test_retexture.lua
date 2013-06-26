-- Class()

require 'image'

dofile 'util.lua'

pi = math.pi
pi2 = pi * 0.5

d2r = math.pi / 180

obj_file = "cube.obj"

if paths.filep(obj_file) then
   printf("using : %s", obj_file)
   -- scan = model.Scan.new(dslr_dir,matter_pose_fname,obj_file)
   scan = data.Obj.new(obj_file)
else
   error("Can't find the obj file (set -obj_file correctly)")
end

scan.face_verts[{1,{},1}] = -1.2
scan.face_verts[{2,{},2}] =  1.2
scan.face_verts[{3,{},1}] =  1.2
scan.face_verts[{4,{},2}] = -1.2
scan.face_verts[{5,{},3}] = -1.2
scan.face_verts[{6,{},3}] =  1.2

scale = 0.2

force  = true

vfov = math.pi - 0.1
hfov = 2*math.pi

views = {}

width  = 512
height = 256

camera = projection.SphericalProjection.new(width,height,hfov,vfov)
-- camera = projection.StereoGraphicProjection.new(width,width,vfov,vfov)

n_views = 2*10

euler_angles = torch.Tensor(2,n_views)

local lambda = 0
local phi    = 0 
local step   = math.pi*0.1

for i = 1,n_views do
   euler_angles[1][i] = phi
   euler_angles[2][i] = lambda
   lambda = lambda + step
   -- phi = phi + step
end

-- compute orientation quaternions for each image in this sweep 
q_rel_pose = geom.quaternion.from_euler_angle(euler_angles)
-- q_global   = geom.quaternion.product(pose.rotation,q_rel_pose)

pose_global = torch.zeros(3)
for i = 1,n_views do 
   local view = model.View.new(pose_global,q_rel_pose[i],hfov,vfov)  
   view:set_projection(camera)
   table.insert(views,view)
end

-- setup a "scan" object
scan.views = views

rt = retex.TextureBuilder.new(scan,{ppm=100})
colors = torch.Tensor(
{
   { 1, 1, 1},  -- white
   { 1, 1, 0},  -- yellow
   { 1, 0, 1},  -- cyan
   { 0, 1, 1},  -- green + blue
   { 0, 0, 1},  -- blue
   { 0, 1, 0}   -- green
})

for fid = 1,scan.n_faces do 
   color = colors[fid]
   print("color",color)
   face_xyz = rt:xyz_wireframe(fid)

   retexts = {}
   for vi = 1,#views do 
      local view = views[vi]
      log.trace("face: ", fid, " view: ", vid)

      woff, _ , wmask = view:global_xyz_to_offset_and_mask(face_xyz,true)
      invmask = wmask:eq(0)
      if (invmask:sum() > 0) then 
         print(" - Computing")
         woff = woff[invmask]
      
         view.img = view.img or torch.zeros(3,height,width)
         for cid = 1,3 do 
             c  = view.img[cid]
             c:resize(c:nElement())
             c[woff] = color[cid]
         end
      else
         print(" - Skipping")
      end
   end
end

for vid = #scan.views,1,-1 do
   image.display{image=scan.views[vid].img, legend=string.format("Sweep: %d",vid)}
end 

