require 'torch'
local geom = require 'util.geom'
local fs = require 'util.fs'
local paths = require 'paths'
local config = require 'config'
local Photo = require('Photo')
local Sweep = torch.class('Sweep')


function Sweep:__init(sweep_dir)
  self.path = sweep_dir -- keep track of the sweep_dir; maybe use relative path instead? 
  self:set_photos()
end

function Sweep:set_photos()
  local img_dir = paths.concat(self.path, config.img_folder)
  if not img_dir or not paths.dirp(img_dir) then log.trace(img_dir, 'not found. Photos not created') return end
  
  self.photos = {}
  for i, f in ipairs(fs.files_only(img_dir, unpack(config.img_extensions))) do
    table.insert(self.photos, Photo.new(f))
  end
end

function Sweep:set_pose(pose) 
  self.position = pose.position -- + config.offset
  self.rotation = pose.rotation -- + config.offset
  
  local sweep_coverage = 2 * math.pi
  local angular_velocity = -sweep_coverage / #self.photos --Negative because Matterport rotates clockwise
  local rotation_axis = torch.Tensor({0,0,1}) --Up

  for i, photo in ipairs(self.photos) do
    local offset_position = torch.Tensor(3):fill(0)
    local offset_rotation = torch.Tensor(4)

    if i == 1 then
      geom.quaternion_from_axis_angle(rotation_axis, config.delayed_start_rotation, offset_rotation)
    else
      geom.quaternion_from_axis_angle(rotation_axis, angular_velocity, offset_rotation)
    end
    geom.quaternion_normalize(offset_rotation)

   photo.offset_position = offset_position
   photo.offset_rotation = offset_rotation
  end
end

function Sweep:calculate_camera_world(photo_number)
  local position = torch.Tensor(3):copy(self.position)
  local rotation = torch.Tensor(4):copy(self.rotation)

  --Accumulate all the camera's rotations up to the camera in question
  local temp_quat = torch.Tensor(4)
  for i = 1, photo_number do
    geom.quat_product(rotation, self.photos[i].offset_rotation, temp_quat)
    geom.quaternion_normalize(temp_quat)
    rotation:copy(temp_quat)
  end
  
  local camera_rig_offset_position = torch.Tensor({config.rig_offset_position[1], config.rig_offset_position[2], config.rig_offset_position[3]})
  local camera_rig_offset_rotation = torch.Tensor({config.rig_offset_rotation[1], config.rig_offset_rotation[2], config.rig_offset_rotation[3], config.rig_offset_rotation[4]})
  geom.quaternion_normalize(camera_rig_offset_rotation)

  local forward_vector = torch.Tensor({0,1,0})
  local camera_look_direction = geom.rotate_by_quat(forward_vector, camera_rig_offset_rotation)
  local camera_center = camera_rig_offset_position + camera_look_direction
  local camera_center_rotated = geom.rotate_by_quat(camera_center, rotation)

  local camera_rig_offset_position_magnitude = camera_rig_offset_position:norm()
  geom.normalize(camera_rig_offset_position)
  
  local camera_rig_offset_position_rotated = geom.rotate_by_quat(camera_rig_offset_position, rotation)
  torch.mul(camera_rig_offset_position_rotated, camera_rig_offset_position_rotated, camera_rig_offset_position_magnitude)
  torch.add(position, position, camera_rig_offset_position_rotated)
  torch.add(position, position, self.photos[photo_number].offset_position)
  local camera_look_direction_rotated = camera_center_rotated - camera_rig_offset_position_rotated

  local final_rotation = torch.Tensor(4)
  geom.quaternion_from_to(forward_vector, camera_look_direction_rotated, final_rotation)
  geom.quaternion_normalize(final_rotation)
  
  return position, final_rotation 
end

return Sweep