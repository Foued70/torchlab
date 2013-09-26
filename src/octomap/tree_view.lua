if (not tree) then
   error("you need to build the tree first")
end
pi = math.pi
camera_width  = 1024 -- 512 -- 2048 -- 
camera_height = 512 -- 256 -- 1024 -- 
n_shots = 3

camera_hfov   = 2*pi
camera_vfov   = camera_hfov/2

-- forward_vector = torch.Tensor({0,1,0})
max_range = 50

axes = torch.eye(3)

for c = 1, #camera_poses -1 do

   camera_pose = camera_poses[c]
   camera_delta = camera_poses[c+1] - camera_pose
   direction_vector = geom.util.normalize(camera_delta)
   camera_delta:mul(1/(camera_delta:norm()*(n_shots + 1)))

   -- rotation to move direction to forward
   -- rotation = geom.quaternion.angle_between(direction_vector, forward_vector)

   proj = projection.SphericalProjection.new(camera_width,camera_height,camera_hfov,camera_vfov)

   for s = 1,n_shots do

      _G.angles = proj:angles_map()

      dirs = geom.util.spherical_angles_to_unit_cartesian(angles)
      
      frame = tree:ray_trace(camera_pose, dirs, max_range)

      -- image.display(frame)
      fname = fname or "frame"
      image.save(string.format("%s_%03d.%02d.png", fname, c, s), frame)

      -- next
      camera_pose:add(camera_delta)
      collectgarbage()
   end
end
