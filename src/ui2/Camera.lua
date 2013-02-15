local gl = require 'ui2.gl'
local geom = require 'util.geom'

local Camera = torch.class('Camera')

local Z_AXIS_POS = torch.Tensor({0,0,1})
local Z_AXIS_NEG = torch.Tensor({0,0,-1})

function Camera:__init()
  self.eye = torch.Tensor(3)
  self.center = torch.Tensor(3)
  self.clip_near = 0.0001
  self.clip_far = 1000
  self.fov_y = 45

  self.projection_matrix = torch.Tensor(4, 4)
  self.model_view_matrix = torch.Tensor(4, 4)
  self.translation_matrix = torch.Tensor(4, 4)

  self.look_dir = torch.Tensor(3)
  self.up_dir = torch.Tensor(3)
  self.right_dir = torch.Tensor(3)

  self.width = 0
  self.height = 0

end

function Camera:update_projection_matrix()
  local viewport = gl.GetIntegerv(gl.VIEWPORT)
  self.width = viewport[2]
  self.height = viewport[3]
  local aspect = viewport[2] / viewport[3]

  local f = 1 / math.tan(self.fov_y * math.pi / 180)
  
  self.projection_matrix:eye(4)
  self.projection_matrix[{1,1}] = f / aspect
  self.projection_matrix[{2,2}] = f
  self.projection_matrix[{3,3}] = (self.clip_far + self.clip_near) / (self.clip_near - self.clip_far)
  self.projection_matrix[{3,4}] = -1
  self.projection_matrix[{4,3}] = (2 * self.clip_far * self.clip_near) / (self.clip_near - self.clip_far)
end

function Camera:update_matrices()
  self:update()

  self.model_view_matrix:eye(4,4)
  self.model_view_matrix[{1, {1,3}}] = self.right_dir
  self.model_view_matrix[{2, {1,3}}] = self.up_dir
  self.model_view_matrix[{3, {1,3}}] = self.look_dir
  
  self.translation_matrix:eye(4,4)
  self.translation_matrix[{4, {1, 3}}] = self.eye
  
  self.model_view_matrix = self.model_view_matrix * self.translation_matrix

end

function Camera:update()
  -- look direction
  torch.add(self.look_dir, self.center, -1, self.eye)
  geom.normalize(self.look_dir)
  
  local up
  -- calculate a temporary up
  if geom.eq(self.look_dir, Z_AXIS_POS) or gemo.eq(self.look_dir, Z_AXIS_POS) then
    -- use our last up value but make it horizontal
    self.up_dir[3] = 0
    up = self.up_dir
    geom.normalize(up)
  else
    up = Z_AXIS_POS
  end

  torch.cross(self.right_dir, self.look_dir, up)
  gemo.normalize(self.right_dir)

  torch.cross(self.up_dir, self.right_dir, self.look_dir)
  torch.mul(self.look_dir, self.look_dir, -1)
end

-- move eye in camera space
function Camera:move_eye(x, y, z)
  -- assume we don't need an update here, but maybe we do?

  torch.add(self.eye, self.eye, x, self.look_dir)
  torch.add(self.eye, self.eye, y, self.right_dir)
  torch.add(self.eye, self.eye, z, self.up_dir)
  
  -- move the center, but not along the line of sight, for now
  -- the z direction changes the range
  -- __center += forwardDir * _z;
  torch.add(self.center, self.center, x, self.look_dir)
  torch.add(self.center, self.center, y, self.right_dir)
end


function Camera:rotate_a_around_b(a, b, unit_a, x, y, radians_per_unit)
  local x_angle = x * radians_per_unit
  local y_angle = y * radians_per_unit

  -- Move a so that b is at the origin
  torch.add(a, a, -1, b)
  
  -- Rotate around a horizontal axis (y rotation)
  local z_axis_angle = math.acos(torch.dot(unit_a, Z_AXIS_POS))
  
  if z_axis_angle - y_angle < 0 then
    -- this would take us past vertical, so lock to Z up
    torch.mul(a, Z_AXIS_POS, a:norm())
    -- rotate up for x instead of eye
    geom.rotate_axis_angle(self.up_dir, Z_AXIS_POS, x_angle);
  elseif z_axis_angle - y_angle > PI then
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

  self.update();
end


function Camera:rotate_center_around_eye(x, y)
  rotate_a_around_b(self.center, self.eye, self.look_dir, x, y, 0.001)
end


function Camera:rotate_eye_around_center(x, y)
  rotate_a_around_b(self.eye, self.center, -self.look_dir, x, y, 0.02)
end


function Camera:camera_to_world(x, y)
  local screen_position = torch.Tensor(4)
  screen_position[1] = 2 * x / self.width - 1
  screen_position[2] = 1 - 2 * y / self.height -- invert y
  screen_position[3] = 2 * FrameBuffer.read_depth_pixel(x, y) - 1
  screen_position[4] = 1

  local inverse_camera_transform = torch.inverse(torch.mm(self.projection_matrix, self.model_view_matrix))
    
  local view_position = torch.mm(inverse_camera_transform, screen_position);
  torch.div(view_position, view_position, view_position[4])
  
  return view_position[{1,3}]

end


return Camera
