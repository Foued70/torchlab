require 'torch'
local paths  = require 'paths'
local config = require 'model.config'

local fs    = util.fs
local Photo = model.Photo
local Sweep = Class()

function Sweep:__write_keys()
  return {'path', 'photos', 'position', 'rotation'}
end

function Sweep:__after_read()
  for i=1, #self.photos do
    self.photos[i].sweep = self
  end
end

function Sweep:__init(parent_scan, sweep_dir)
  self.scan = parent_scan
  self.path = sweep_dir -- keep track of the sweep_dir; maybe use relative path instead? 
  self:set_photos()
end

function Sweep:set_photos()
  local img_dir = self.path -- paths.concat(self.path, config.img_folder)
  if not img_dir or not paths.dirp(img_dir) then log.trace(img_dir, 'not found. Photos not created') return end
  
  self.photos = {}
  print(unpack(config.img_extensions))
  for i, f in ipairs(fs.glob(img_dir, config.img_extensions)) do
    table.insert(self.photos, Photo.new(self, f))
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
      geom.rotation.quaternion_from_axis_angle(rotation_axis, config.delayed_start_rotation, offset_rotation)
    else
      geom.rotation.quaternion_from_axis_angle(rotation_axis, angular_velocity, offset_rotation)
    end
    geom.util.normalize(offset_rotation)

    -- TODO: figure out when/how these should be set when p3p is added
    -- offset is relative to the sweep
    photo.offset_position = offset_position
    photo.offset_rotation = offset_rotation
    
    photo.position, photo.rotation = self:calculate_camera_world(i)
    photo.rotation_r = torch.Tensor(4):copy(photo.rotation)
    photo.rotation_r:narrow(1,1,3):mul(-1)
  end
end

function Sweep:calculate_camera_world(photo_number)
  local position = torch.Tensor(3):copy(self.position)
  local rotation = torch.Tensor(4):copy(self.rotation)

  --Accumulate all the camera's rotations up to the camera in question
  local temp_quat = torch.Tensor(4)
  for i = 1, photo_number do
    geom.rotation.quat_product(rotation, self.photos[i].offset_rotation, temp_quat)
    geom.util.normalize(temp_quat)
    rotation:copy(temp_quat)
  end
  
  local camera_rig_offset_position = 
     torch.Tensor({config.rig_offset_position[1], 
                   config.rig_offset_position[2], 
                   config.rig_offset_position[3]})

  local camera_rig_offset_rotation = 
     torch.Tensor({config.rig_offset_rotation[1],
                   config.rig_offset_rotation[2],
                   config.rig_offset_rotation[3],
                   config.rig_offset_rotation[4]})

  geom.util.normalize(camera_rig_offset_rotation)

  local forward_vector = torch.Tensor({0,1,0})
  local camera_look_direction = 
     geom.rotate_by_quat(forward_vector, camera_rig_offset_rotation)
  local camera_center = 
     camera_rig_offset_position + camera_look_direction
  local camera_center_rotated = 
     geom.rotate_by_quat(camera_center, rotation)

  local camera_rig_offset_position_magnitude = camera_rig_offset_position:norm()
  geom.normalize(camera_rig_offset_position)
  
  local camera_rig_offset_position_rotated = geom.rotate_by_quat(camera_rig_offset_position, rotation)
  torch.mul(camera_rig_offset_position_rotated, camera_rig_offset_position_rotated, camera_rig_offset_position_magnitude)
  torch.add(position, position, camera_rig_offset_position_rotated)
  torch.add(position, position, self.photos[photo_number].offset_position)
  local camera_look_direction_rotated = camera_center_rotated - camera_rig_offset_position_rotated

  local final_rotation = torch.Tensor(4)
  geom.quaternion_from_to(forward_vector, camera_look_direction_rotated, final_rotation)
  geom.normalize(final_rotation)
  
  return position, final_rotation 
end

function Sweep:save_wireframes_red_alpha()   
  local ext = '_wireframe.png'
  local dir = paths.concat(self.path, 'wireframe')
  sys.execute("mkdir -p " .. dir)
  
  for pi = 1,self.photos do
    local photo = self.photos[pi]
    local wimage = photo:draw_wireframe()
    image.display(wimage)
    -- save
    local photo_ext = fs.extname(photo.name)
    local wimagename = paths.concat(dir, photo.name:gsub(photo_ext,ext))
    log.trace("Saving:", wimagename)
    image.save(wimagename,wimage)
  end
end

function Sweep:save_wireframes_image_blacklines()
  local ext = '_wireframe_RGB.png'
  local dir = paths.concat(self.path, 'wireframe_RGB')
  sys.execute("mkdir -p " .. dir)
  
  for pi = 1,self.photos do 
    local photo = self.photos[pi]
    local wimage = photo:draw_wireframe()
    -- invert 0 and 1 in red channel
    wimage = wimage[1]:mul(-1):add(1)
    local cimage = photo.image:clone()
    cimage[1]:cmul(wimage)
    cimage[2]:cmul(wimage)
    cimage[3]:cmul(wimage)
    image.display(cimage) 
    local photo_ext = fs.extname(photo.name)
    local wimagename = paths.concat(dir, photo.name:gsub(photo_ext,ext))
    log.trace("Saving:", wimagename)
    image.save(wimagename,cimage)
  end
end