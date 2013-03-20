require 'torch'
local geom = require 'util.geom'
local fs = require 'util.fs'
local paths = require 'paths'
local config = require 'config'
local SweepCamera = require('SweepCamera')
local Sweep = torch.class('Sweep')


function Sweep:__init(sweep_dir)
  self.path = sweep_dir -- keep track of the sweep_dir; maybe use relative path instead? 
  self:set_cameras()
end

function Sweep:set_cameras()
  local img_dir = paths.concat(self.path, config.img_folder)
  if not img_dir or not paths.dirp(img_dir) then log.trace(img_dir, 'not found. Cameras not created') return end
  
  self.cameras = {}
  for i, f in ipairs(fs.files_only(img_dir, unpack(config.img_extensions))) do
    table.insert(self.cameras, SweepCamera.new(f))
  end
end

function Sweep:set_pose(pose)
  log.trace("pose", pose)    
  self.position = pose.position -- + config.offset
  self.rotation = pose.rotation -- + config.offset
  
  local sweep_coverage = 2 * math.pi
  local angular_velocity = -sweep_coverage / #self.cameras --Negative because Matterport rotates clockwise
  local rotation_axis = torch.Tensor({0,0,1}) --Up
  local forward_vector = torch.Tensor({0,1,0})

  for i, camera in ipairs(self.cameras) do
    local offset_position = torch.Tensor(3):fill(0)
    local offset_rotation = torch.Tensor(4)

    --The dlsr is aligned 90 degrees off from the matterport. TODO: move to config file
    if i == 1 then
      geom.quaternion_from_axis_angle(rotation_axis, (math.pi*0.5), offset_rotation)
    else
      geom.quaternion_from_axis_angle(rotation_axis, angular_velocity, offset_rotation)
    end

   camera.offset_position = offset_position
   camera.offset_rotation = offset_rotation
  end
end

function Sweep:calculate_camera_world(camera_number)
  p("self.position")
  p(self.position)
  p("self.cameras[camera_number].offset_position")
  p(self.cameras[camera_number].offset_position)

  local position = torch.Tensor(3):copy(self.position) + self.cameras[camera_number].offset_position
  local rotation = torch.Tensor(4):copy(self.rotation)

  --Accumulate all the camera's rotations up to the camera in question
  for i = 1, camera_number do
    rotation = geom.quat_product(rotation, self.cameras[camera_number].offset_rotation, rotation)
  end

  return position, rotation
end

return Sweep