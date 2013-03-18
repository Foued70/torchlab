local Pose = torch.class('Pose')

function Pose:__init(pose_string)
  local pose_values = torch.Tensor(1, 11)
  local k = 1
  local first_value = true
  for value in pose_string:gmatch("%S+") do    
    if first_value then
      self.name = value
      first_value = false      
    else      
      pose_values[1][k] = tonumber(value)
      k = k + 1
    end
  end
  
  self.quat = pose_values:narrow(2,1,4)
  self.xyz = pose_values:narrow(2,5,3)
  self.center_u = pose_values:select(2,8)
  self.center_v = pose_values:select(2,9) 
  self.degree_per_px_x = pose_values:select(2,10)
  self.degree_per_px_y = pose_values:select(2,11)
end

return Pose