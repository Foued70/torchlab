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
  
  self.rotation = torch.Tensor(4):copy(pose_values:narrow(2,1,4))
  self.position = torch.Tensor(3):copy(pose_values:narrow(2,5,3))
end

return Pose