local Matterport_PoseFile = Class()

-- A matterport pose file is provided by matterport with as a record
-- of their estimated position and rotation for each sweep in a scan.

-- The file has one line for each texture, and the lines look like:

--     scanner206_job108000_000.jpg 0.0015876 0.0115254 1.83032e-005 0.999932 0.727866 -0.311403 1.07462 0.5 0.499534 0.19043 0.209375

-- The fields that make up each line are:

--     <texture filename> <qx> <qy> <qz> <qw> <tx> <ty> <tz> <center u> <center v> <degrees per px x> <degrees per px y>

-- The first seven numbers define a transform from local sweep
-- coordinates to the global coordinates used for vertices in the .obj.
-- In the local sweep coordinates, the center of the texture is what
-- you'd see looking along the +x axis, and the +z axis is up (towards
-- the top of the texture).  The quaternion defined by the coefficients
-- (qx, qy, qz, qw) is a rotation from local sweep coordinates to global
-- coordinates, and the position (tx, ty, tz) is the origin of the sweep
-- in global coordinates.

-- The next two numbers define the uv coordinates of the point you
-- should treat as the center of the texture.  The u will always be
-- 0.5, but the v will vary a little bit due to how we crop the
-- texture. Origin of uv is bottom left


-- The final two numbers are change in degrees per pixel as you move
-- around in the image horizintally (x) or vertically (y).

-- An example of how to interpret all those numbers for a specific
-- pixel:

--  + The texture is 2048 x 512.  Center u and v are 0.5 and 0.49805,
--    which work out to pixel xy coordinates of (1024, 257).  If we're
--    looking at the pixel at xy coordinates (1600, 100), which means
--    the center of that pixel is at (1600.5, 100.5).  The distance
--    from the center coordinates is (576.5, -156.5).

--  + Our degrees per pixel are 0.19 (x) and 0.21 (y), meaning those
--    distances from the center turn into angles of: 109.535 degrees
--    right of center (azimuth), 32.865 degrees up from center
--    (elevation).
 
--  + To convert this to a unit vector, we first consider azimuth in
--    xy only, which gives us (cos(-109.535), sin(-109.535)) =
--    (-0.334, -0.942).  Elevation then gives us a z of sin(32.865) =
--    0.543, and then we scale xy to make the whole thing a unit
--    vector, giving (-0.281, 0.792, 0.543). [MS math is wrong here]

--  + So in local sweep coordinates, the pixel at (1600.5, 100.5) is what
--    you'd see looking from the origin along the vector (-0.281, 0.792,
--    0.543).

--  + If you rotate that unit vector by the quaternion (qx, qy, qz, qw),
--    you'll get a new unit vector.  In global coordinates, that pixel at
--    (1600.5, 100.5) represents what you'd see looking from the point
--    (tx, ty, tz) along the new (rotated) unit vector.

function Matterport_PoseFile:__init(posefile)

   self:loadfile(posefile)

end

function Matterport_PoseFile:loadfile(posefile)
   if not paths.filep(posefile) then return nil end

   log.trace('Loading poses from', posefile)

   local poses = {}

   for line in io.lines(posefile) do
      local pose_values = {}

      for value in line:gmatch("%S+") do
         table.insert(pose_values, value)
      end

      local pose = {}

      -- <texture filename> <qx> <qy> <qz> <qw> <tx> <ty> <tz> <center u>
      -- <center v> <degrees per px x> <degrees per px y>
      pose.name = pose_values[1]
      pose.local_to_global_rotation = 
         torch.Tensor({pose_values[2], pose_values[3], pose_values[4], pose_values[5]})
      pose.local_to_global_position = 
         torch.Tensor({pose_values[6], pose_values[7], pose_values[8]})
      -- unit vector staring down +x axis needs to become unit vector staring down +y axis
      local offset = geom.quaternion.from_euler_angle(torch.Tensor({0,-math.pi/2}))
      pose.local_to_global_rotation = 
         geom.quaternion.product(offset,pose.local_to_global_rotation)

      pose.global_to_local_rotation = 
         geom.quaternion.conjugate(pose.local_to_global_rotation)
      pose.global_to_local_position = 
         torch.mul(pose.local_to_global_position,-1)

      pose.center_u = pose_values[9]
      pose.center_v = pose_values[10]

      pose.degrees_per_px_x = pose_values[11]
      pose.degrees_per_px_y = pose_values[12]

      table.insert(poses, pose)
   end
   local n_poses = #poses
   log.trace(n_poses, 'poses loaded.')

   local positions_local  = torch.Tensor(n_poses,3)
   local rotations_local  = torch.Tensor(n_poses,4)
   local positions_global = torch.Tensor(n_poses,3)
   local rotations_global = torch.Tensor(n_poses,4)
   for i = 1,n_poses do 
      local pose = poses[i]
      positions_local[i]  = pose.local_to_global_position
      rotations_local[i]  = pose.local_to_global_rotation
      positions_global[i] = pose.global_to_local_position
      rotations_global[i] = pose.global_to_local_rotation
   end

   self.poses            = poses
   self.n_poses          = n_poses
   self.positions_local  = positions_local
   self.rotations_local  = rotations_local
   self.positions_global = positions_global
   self.rotations_global = rotations_global

   return self.poses
end
