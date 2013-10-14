if (not tree) then
   error("you need to build the tree first")
end
pi            = math.pi
pi2           = pi/2
normalize     = geom.util.normalize
e2q           = geom.quaternion.from_euler_angle
rotate        = geom.quaternion.rotate

aspect_ratio  = 3/4
camera_width  = 1024  
camera_height = camera_width * aspect_ratio
n_steps                  = 24
n_shots_per_turn         = n_steps
n_steps_to_stop_and_turn = n_steps/2

camera_hfov   = pi/2
camera_vfov   = camera_hfov * aspect_ratio

max_range     = 50

axes           = torch.eye(3)
forward_vector = axes[2]

proj     = projection.GnomonicProjection.new(camera_width,camera_height,camera_hfov,camera_vfov)
angles   = geom.util.projection_to_spherical_angles(proj:angles_map())
dirs     = geom.util.spherical_angles_to_unit_cartesian(angles)

rotation = normalize(e2q(torch.Tensor({0,-pi2}))):squeeze()
dirs     = normalize(rotate(rotation,dirs),1)
rstep    = normalize(e2q(torch.Tensor({0,2*pi/n_shots_per_turn}))):squeeze()
upstep   = normalize(e2q(torch.Tensor({pi/2*n_shots_per_turn,0}))):squeeze()
downstep = normalize(e2q(torch.Tensor({-pi/2*n_shots_per_turn,0}))):squeeze()

camera_pose      = camera_poses[1]
camera_delta     = camera_poses[#camera_poses] - camera_pose
direction_vector = geom.util.normalize(camera_delta)

camera_delta:mul(1/n_steps)

c = 1
for s = 1,n_steps do

   if ((s % n_steps_to_stop_and_turn) == 1) or (s == n_steps) then
      -- up
      -- for t = 1,n_shots_per_turn/4 do 
      --    printf(" - turning up: %d,%d",c,t)
      --    dirs = normalize(rotate(upstep,dirs)) 
      --    frame = tree:ray_trace(camera_pose, dirs, max_range)
      --    fname = fname or "frame"
      --    image.save(string.format("%s_%05d.png", fname, c), frame)
      --    c = c+1
      --    collectgarbage()
      -- end
      -- horizontal
      for t = 1,n_shots_per_turn/4 do 
         printf(" - turning: %d,%d",c,t)
         dirs = normalize(rotate(rstep,dirs)) 
         frame = tree:ray_trace(camera_pose, dirs, max_range)
         fname = fname or "frame"
         image.save(string.format("%s_%05d.png", fname, c), frame)
         c = c+1
         collectgarbage()
      end
      -- -- down
      -- for t = 1,n_shots_per_turn/2 do 
      --    printf(" - turning down: %d,%d",c,t)
      --    dirs = normalize(rotate(downstep,dirs)) 
      --    frame = tree:ray_trace(camera_pose, dirs, max_range)
      --    fname = fname or "frame"
      --    image.save(string.format("%s_%05d.png", fname, c), frame)
      --    c = c+1
      --    collectgarbage()
      -- end
      -- -- up
      -- for t = 1,n_shots_per_turn/4 do 
      --    printf(" - turning up: %d,%d",c,t)
      --    dirs = normalize(rotate(upstep,dirs)) 
      --    frame = tree:ray_trace(camera_pose, dirs, max_range)
      --    fname = fname or "frame"
      --    image.save(string.format("%s_%05d.png", fname, c), frame)
      --    c = c+1
      --    collectgarbage()
      -- end
   end

   printf(" - stepping: %d,%d (%f,%f,%f)",c,s,camera_pose[1],camera_pose[2],camera_pose[3])
   frame = tree:ray_trace(camera_pose, dirs, max_range)
   fname = fname or "frame"
   image.save(string.format("%s_%05d.png", fname, c), frame)
   -- next
   camera_pose:add(camera_delta)
   c = c+1
   collectgarbage()
end

