local Matterport_PoseFile = Class()

-- A matterport pose file is provided by matterport with as a record
-- of their estimated position and rotation for each sweep in a scan.

-- <texture filename> <qx> <qy> <qz> <qw> <tx> <ty> <tz> <center u>
-- <center v> <degrees per px x> <degrees per px y>
-- origin of uv is bottom left
function Matterport_PoseFile:__init(posefile)

   self:loadfile(posefile)

end

function Matterport_PoseFile:loadfile(posefile)
   if not paths.filep(posefile) then return nil end

   log.trace('Loading poses from', posefile)

   self.poses = {}

   for line in io.lines(posefile) do
      local pose_values = {}

      for value in line:gmatch("%S+") do
         table.insert(pose_values, value)
      end

      local pose = {}
      pose.name = pose_values[1]
      pose.local_to_global_rotation = torch.Tensor({pose_values[2], pose_values[3], pose_values[4], pose_values[5]})

      pose.local_to_global_position = torch.Tensor({pose_values[6], pose_values[7], pose_values[8]})

      pose.global_to_local_rotation = geom.quaternion.conjugate(pose.local_to_global_rotation)
      pose.global_to_local_position = torch.mul(pose.local_to_global_position,-1)

      pose.center_u = pose_values[9]
      pose.center_v = pose_values[10]

      pose.degrees_per_px_x = pose_values[11]
      pose.degrees_per_px_y = pose_values[12]

      table.insert(self.poses, pose)
   end
   log.trace(#self.poses, 'poses loaded.')

   return self.poses
end
