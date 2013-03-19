require 'torch'
local geom = require 'util.geom'
local SweepCamera = require('SweepCamera')

local Sweep = torch.class('Sweep')

function Sweep:__init(lens, position, rotation, image_paths)
  self.postion = position
  self.rotation = rotation
  self.cameras = {}

  self:init_cameras(lens, image_paths)
end

function Sweep:init_cameras(lens, image_paths)
  local num_images = #image_paths
  local sweep_coverage = 2 * math.pi
  local angular_velocity = -sweep_coverage / num_images --Negative because Matterport rotates clockwise
  local rotation_axis = torch.Tensor({{0,0,1}}) --Up
  local forward_vector = torch.Tensor({{0,1,0}})

  for image_number = 1, #image_paths do
    local offset_position = torch.Tensor(1,3):fill(0)
    local offset_rotation = torch.Tensor(1,4)

    --For now, assume the 1st shot has no offset rotation. It's global rotation == sweep.rotation
    if image_number == 1 then
      offset_rotation[{{1,3}}] = 0
      offset_rotation[4] = 1
    else
      geom.quaternion_from_axis_angle(rotation_axis, angular_velocity, offset_rotation)
    end

    local camera = SweepCamera.new(lens, offset_position, offset_rotation, image_paths[image_number])
    table.insert(self.cameras, camera)
  end
end

function Sweep:calculate_camera_world(camera_number)
  local position = self.position + self.cameras[camera_number].offset_position
  local rotation = torch.Tensor(1,4):copy(self.rotation)

  --Accumulate all the camera's rotations up to the camera in question
  for i = 1, camera_number do
    rotation = quat_product(rotation, self.cameras[camera_number].offset_rotation, rotation)
  end

  return position, rotation
end

return Sweep