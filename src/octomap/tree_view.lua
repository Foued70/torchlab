if (not tree) then
   error("you need to build the tree first")
end
pi = math.pi
camera_width  = 512 -- 2048
camera_height = 256 -- 1024
n_shots = 3

camera_hfov   = 2*pi
camera_vfov   = camera_hfov/2

forward_vector = torch.Tensor({0,1,0})
max_range = 50

for c = 1, #camera_centers -1 do

   camera_pose = camera_centers[c]
   camera_delta = camera_centers[c+1] - camera_pose
   direction_vector = geom.util.normalize(camera_delta)
   camera_delta:mul(1/(camera_delta:norm()*(n_shots + 1)))

   -- rotation to move direction to forward
   rotation = geom.quaternion.angle_between(direction_vector, forward_vector)

   proj = projection.SphericalProjection.new(camera_width,camera_height,camera_hfov,camera_vfov)

   for s = 1,n_shots do

      angles = proj:angles_map()

      quats = geom.quaternion.from_euler_angle(angles)
      -- quats = geom.quaternion.product(quats,rotation)
      -- dirs  = geom.quaternion.rotate(quats,forward_vector)
      
      dirs = geom.quaternion.rotate(quats,)
      -- dirs:mul(-1)
      -- dirs[1]:mul(-1)

      -- TODO change rotations to DHW
      dirs  = dirs:transpose(1,2):contiguous():resize(3,camera_height, camera_width)

      frame = tree:ray_trace(camera_pose, dirs, max_range)

      -- image.display(frame)
      image.save(string.format("frame_%03d.%02d.png", c, s), frame)

      -- next
      camera_pose:add(camera_delta)
      collectgarbage()
   end
end
