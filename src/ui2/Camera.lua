local gl = require 'ui2.gl'
local geom = require 'util.geom'

local Camera = torch.class('Camera')

local Z_AXIS_POS = torch.Tensor({0,0,1})
local Z_AXIS_NEG = torch.Tensor({0,0,-1})

local function memlog(label, x)
  local s = x:storage()
  p(label)
  p(("%.4f %.4f %.4f %.4f"):format(s[1], s[2], s[3], s[4]))
  p(("%.4f %.4f %.4f %.4f"):format(s[5], s[6], s[7], s[8]))
  p(("%.4f %.4f %.4f %.4f"):format(s[9], s[10], s[11], s[12]))
  p(("%.4f %.4f %.4f %.4f"):format(s[13], s[14], s[15], s[16]))
  p('')
end

local function vlog(label, x)
  p(("%-10s %.4f %.4f %.4f "):format(label, x[1], x[2], x[3]))
end

function Camera:__init(widget, name)
  self.widget = widget
  self.name = name
  self.eye = torch.Tensor({0,0,0})
  self.center = torch.Tensor({0,1,0})
  self.clip_near = 0.0001
  self.clip_far = 1000
  self.fov_y = math.pi / 4 -- 45 degrees

  -- intermediate direction vectors
  self.look_dir = torch.Tensor({0,1,0})
  self.up_dir = torch.Tensor({0,0,1})
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
  self:rebuild_buffers()
  log.trace(self.name, " resized to:", self.width, self.height)
end

function Camera:update_projection_matrix()
  local aspect = self.width / self.height

  local f = 1 / (math.tan(self.fov_y/2))
  
  self.projection_matrix:eye(4)
  self.projection_matrix[{1,1}] = f / aspect
  self.projection_matrix[{2,2}] = f
  self.projection_matrix[{3,3}] = (self.clip_far + self.clip_near) / (self.clip_near - self.clip_far)
  self.projection_matrix[{3,4}] = (2 * self.clip_far * self.clip_near) / (self.clip_near - self.clip_far)
  self.projection_matrix[{4,3}] = -1

  --self.projection_matrix = context.projection_matrix:double()
  -- memlog('projection_matrix', context.projection_matrix)
end

function Camera:rebuild_buffers()
  if self.frame_buffer then
    self.frame_buffer:__gc()
  end
  self.frame_buffer = require('ui2.FrameBuffer').new(self.widget, self.name..'_frame_buffer', self.width, self.height)
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
end

function Camera:update()
  -- look direction
  torch.add(self.look_dir, self.center, -1, self.eye) -- look_dir = center - eye
  geom.normalize(self.look_dir)
  
  local up
  -- calculate a temporary up
  if geom.eq(self.look_dir, Z_AXIS_POS) or geom.eq(self.look_dir, Z_AXIS_NEG) then
    -- use our last up value but make it horizontal
    self.up_dir[3] = 0
    up = self.up_dir
    geom.normalize(up)
  else
    up = Z_AXIS_POS
  end

  torch.cross(self.right_dir, self.look_dir, up) -- right_dir = look_dir X up
  geom.normalize(self.right_dir)

  torch.cross(self.up_dir, self.right_dir, self.look_dir)
end

-- move eye in camera space
function Camera:move_eye(x, y, z)
  -- assume we don't need an update here, but maybe we do?
  torch.add(self.eye, self.eye, x, self.right_dir)
  torch.add(self.eye, self.eye, y, self.up_dir)

  -- don't let eye move past center
  if self:eye_dist() - z > 0 then
    torch.add(self.eye, self.eye, z, self.look_dir)
  end

  -- move the center, but not along the line of sight, for now
  -- the z direction changes the range
  -- __center += forwardDir * _z;
  torch.add(self.center, self.center, x, self.right_dir)
  torch.add(self.center, self.center, y, self.up_dir)
end


function Camera:rotate_a_around_b(a, b, unit_a, x, y, radians_per_unit)
  local x_angle = x * radians_per_unit
  local y_angle = y * radians_per_unit

  -- Move a so that b is at the origin
  torch.add(a, a, -1, b)
  
  -- Rotate around a horizontal axis (y rotation)
  local z_axis_angle = math.acos(torch.dot(unit_a, Z_AXIS_POS))

  if z_axis_angle - y_angle < 0 then
    -- this would take us past vertical (up), so lock to Z up
    torch.mul(a, Z_AXIS_POS, a:norm())
    -- rotate up for x instead of eye
    geom.rotate_axis_angle(self.up_dir, Z_AXIS_POS, x_angle);
  elseif z_axis_angle - y_angle > math.pi then
    -- this would take us past vertical (down), so lock to Z down
    torch.mul(a, Z_AXIS_NEG, a:norm())
    -- rotate up for x instead of eye
    geom.rotate_axis_angle(self.up_dir, Z_AXIS_POS, x_angle);
  else
    geom.rotate_axis_angle(a, self.right_dir, y_angle);
    geom.rotate_axis_angle(a, Z_AXIS_POS, x_angle);
  end
  
  -- move a back into position
  torch.add(a, a, b)

  self:update()
end

function Camera:rotate_center_around_eye(x, y)
  self:rotate_a_around_b(self.center, self.eye, self.look_dir, x, y, 0.001)
end


function Camera:rotate_eye_around_center(x, y)
  self:rotate_a_around_b(self.eye, self.center, -self.look_dir, x, y, 0.02)
end

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

function Camera:set_center(x, y, z)
  self.center[1] = x
  self.center[2] = y
  self.center[3] = z
end

function Camera:set_up(x, y, z)
  self.up_dir[1] = x
  self.up_dir[2] = y
  self.up_dir[3] = z
end

function Camera:eye_dist()
  return geom.dist(self.eye, self.center)
end

return Camera
