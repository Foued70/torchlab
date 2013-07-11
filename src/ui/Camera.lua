local gl = require './gl'

local Camera = Class()

function Camera:__init(widget, name)
  self.widget = widget
  self.name = name
  self.eye = torch.Tensor({0,0,0})
  self.clip_near = 0.0001
  self.clip_far = 1000
  self.vfov = math.pi / 4 -- 45 degrees

  -- direction vectors
  self.look_dir  = torch.Tensor({0,1,0})
  self.up_dir    = torch.Tensor({0,0,1})
  self.right_dir = torch.Tensor({1,0,0})

  self.projection_matrix = torch.Tensor(4,4):t()

  self.frame_buffer = nil

  self.width = 0
  self.height = 0
end

function Camera:resize(width, height)
  self.width = width
  self.height = height

  self:update_projection_matrix()
  log.trace(self.name, " resized to:", self.width, self.height)
  self:rebuild_buffers()
end

function Camera:update_projection_matrix()
  local aspect = self.width / self.height

  local f = 1 / (math.tan(self.vfov/2))
  
  self.projection_matrix:eye(4)
  self.projection_matrix[{1,1}] = f / aspect
  self.projection_matrix[{2,2}] = f
  self.projection_matrix[{3,3}] = (self.clip_far + self.clip_near) / (self.clip_near - self.clip_far)
  self.projection_matrix[{3,4}] = (2 * self.clip_far * self.clip_near) / (self.clip_near - self.clip_far)
  self.projection_matrix[{4,3}] = -1
end

function Camera:rebuild_buffers()
  if self.frame_buffer then
    self.frame_buffer:__gc()
  end
  self.frame_buffer = ui.FrameBuffer.new(self.widget, self.name..'_frame_buffer', self.width, self.height)
end

function Camera:update_matrix(context)
  self:update()

  context.model_view_matrix:eye(4,4)
  context.model_view_matrix[{1, {1,3}}] = self.right_dir
  context.model_view_matrix[{2, {1,3}}] = self.up_dir
  context.model_view_matrix[{3, {1,3}}] = -self.look_dir

  context:translate(-self.eye)

  context:set_projection(self.projection_matrix)

  self.model_view_matrix = context.model_view_matrix:double()

  context.screen_width = self.width
  context.screen_height = self.height
end

function Camera:update() end -- virtual


function Camera:screen_to_world(screen_position)  
  screen_position:resize(4,1)
  local mvp_matrix = torch.mm(self.projection_matrix, self.model_view_matrix)
  local inverse_camera_transform = torch.inverse(mvp_matrix)
  local view_position = torch.mm(inverse_camera_transform, screen_position);
  screen_position:resize(4)
  view_position:resize(4)
  torch.div(view_position, view_position, view_position[4])
  return view_position[{{1,3}}]
end

function Camera:world_to_screen(world_position)
  world_position_matrix = torch.Tensor(4,1)
  world_position_matrix[1] = world_position[1]
  world_position_matrix[2] = world_position[2]
  world_position_matrix[3] = world_position[3]
  world_position_matrix[4] = 1
  local mvp_matrix = torch.mm(self.projection_matrix, self.model_view_matrix)
  local screen_position = torch.mm(mvp_matrix, world_position_matrix);
  screen_position:resize(4)
  torch.div(screen_position, screen_position, screen_position[4])
  if (screen_position[1] > 1) or (screen_position[1] < -1) or (screen_position[2] > 1) or (screen_position[2] < -1) then
    return nil
  end
  return screen_position
end

function Camera:pixel_to_world(x, y)
  local z = self.frame_buffer:read_depth_pixel(x, y)
  if z >= 1 then
    return nil
  end
  
  local screen_position = torch.Tensor(4)
  screen_position[1] = 2 * x / self.width - 1
  screen_position[2] = 1 - 2 * y / self.height -- invert y
  screen_position[3] = 2 * z - 1
  screen_position[4] = 1
    
  return self:screen_to_world(screen_position)
end

function Camera:screen_to_pixel(screen_position)
  local pixels = torch.Tensor(2)
  pixels[1] = screen_position[1]
  pixels[2] = -screen_position[2] --invert y

  torch.add(pixels, pixels, 1)
  torch.div(pixels, pixels, 2)
  pixels[1] = pixels[1] * self.width
  pixels[2] = pixels[2] * self.height
  return pixels
end


function Camera:set_eye(x, y, z)
  self.eye[1] = x
  self.eye[2] = y
  self.eye[3] = z
end

function Camera:set_look_dir(x, y, z)
  self.look_dir[1] = x
  self.look_dir[2] = y
  self.look_dir[3] = z
end

function Camera:set_up_dir(x, y, z)
  self.up_dir[1] = x
  self.up_dir[2] = y
  self.up_dir[3] = z
end

function Camera:set_right_dir(x, y, z)
  self.right_dir[1] = x
  self.right_dir[2] = y
  self.right_dir[3] = z
end


function Camera:rotate(rotation_quat)
  geom.rotation.by_quaternion(self.look_dir, rotation_quat, self.look_dir)
  geom.rotation.by_quaternion(self.up_dir, rotation_quat, self.up_dir)
  geom.rotation.by_quaternion(self.right_dir, rotation_quat, self.right_dir)
end


