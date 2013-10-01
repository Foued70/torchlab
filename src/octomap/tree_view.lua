if (not tree) then
   error("you need to build the tree first")
end
pi = math.pi
pi2 = pi/2
aspect_ratio  = 3/4
camera_width  = 1024  
camera_height = camera_width * aspect_ratio
n_shots = 50

camera_hfov   = pi/2 
camera_vfov   = camera_hfov * aspect_ratio

max_range     = 50

axes = torch.eye(3)
forward_vector = axes[2]

proj     = projection.GnomonicProjection.new(camera_width,camera_height,camera_hfov,camera_vfov)
angles   = geom.util.projection_to_spherical_angles(proj:angles_map())
dirs     = geom.util.spherical_angles_to_unit_cartesian(angles)

rotation = geom.util.normalize(geom.quaternion.from_euler_angle(torch.Tensor({0,-pi2}))):squeeze()
dirs     = geom.util.normalize(geom.quaternion.rotate(rotation,dirs),1)

camera_pose      = camera_poses[1]
camera_delta     = camera_poses[#camera_poses] - camera_pose
direction_vector = geom.util.normalize(camera_delta)
camera_delta:mul(1/n_shots)

for s = 1,n_shots do

   frame = tree:ray_trace(camera_pose, dirs, max_range)
   
   -- image.display(frame)
   fname = fname or "frame"
   image.save(string.format("%s_%05d.png", fname, s), frame)
   -- next
   camera_pose:add(camera_delta)
   collectgarbage()
end

