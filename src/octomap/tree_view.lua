if (not tree) then 
   error("you need to build the tree first")
end
pi = math.pi
camera_width  = 2048
camera_height = 1024
n_shots = 4

camera_hfov   = 2*pi
camera_vfov   = camera_hfov/2

camera_pose = torch.Tensor({0,0,1.4})
camera_delta = torch.Tensor({0.5,0.5})

if pc then
   camera_pose = pc.centroid:clone():squeeze()
end
if camera_centers then 
   camera_pose = camera_centers[1] 
   camera_delta = camera_centers[#camera_centers] - camera_pose
   camera_delta:mul(1/(camera_delta:norm()*n_shots))
end

proj = projection.SphericalProjection.new(camera_width,camera_height,camera_hfov,camera_vfov)


_G.images = {}
for i = 1,n_shots do 

   angles = proj:angles_map()

   quats = geom.quaternion.from_euler_angle(angles)
   dirs  = geom.quaternion.rotate(quats,geom.util.normalize(torch.Tensor({0,-1,0})))
   dirs[1]:mul(-1)

   -- TODO change rotations to DHW
   dirs  = dirs:transpose(1,2):contiguous():resize(3,camera_height, camera_width)
   
   table.insert(images,tree:ray_trace(camera_pose, dirs, 25))
   camera_pose:add(camera_delta)

end

_G.combo_image = image.display{image=images,nrow=2}
