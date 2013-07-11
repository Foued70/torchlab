local UpCamera = Class(ui.Camera)

local Z_AXIS_POS = torch.Tensor({0,0,1})
local Z_AXIS_NEG = torch.Tensor({0,0,-1})

function UpCamera:__init(widget, name)
  __super__.__init(self, widget, name)
  self.center = torch.Tensor({0,1,0})
end


function UpCamera:update()
  -- look direction
  torch.add(self.look_dir, self.center, -1, self.eye) -- look_dir = center - eye
  geom.util.normalize(self.look_dir)
  
  local up
  -- calculate a temporary up
  if geom.util.eq(self.look_dir, Z_AXIS_POS) or geom.util.eq(self.look_dir, Z_AXIS_NEG) then
    -- use our last up value but make it horizontal
    self.up_dir[3] = 0
    up = self.up_dir
    geom.util.normalize(up)
  else
    up = Z_AXIS_POS
  end

  torch.cross(self.right_dir, self.look_dir, up) -- right_dir = look_dir X up
  geom.util.normalize(self.right_dir)

  torch.cross(self.up_dir, self.right_dir, self.look_dir)
end

-- move eye in camera space
function UpCamera:move_eye(x, y, z)
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


function UpCamera:rotate_a_around_b(a, b, unit_a, x, y, radians_per_unit)
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
    geom.rotation.axis_angle(self.up_dir, Z_AXIS_POS, x_angle);
  elseif z_axis_angle - y_angle > math.pi then
    -- this would take us past vertical (down), so lock to Z down
    torch.mul(a, Z_AXIS_NEG, a:norm())
    -- rotate up for x instead of eye
    geom.rotation.axis_angle(self.up_dir, Z_AXIS_POS, x_angle);
  else
    geom.rotation.axis_angle(a, self.right_dir, y_angle);
    geom.rotation.axis_angle(a, Z_AXIS_POS, x_angle);
  end
  
  -- move a back into position
  torch.add(a, a, b)

  self:update()
end

function UpCamera:rotate_center_around_eye(x, y)
  self:rotate_a_around_b(self.center, self.eye, self.look_dir, x, y, 0.001)
end


function UpCamera:rotate_eye_around_center(x, y)
  self:rotate_a_around_b(self.eye, self.center, -self.look_dir, x, y, 0.02)
end

function UpCamera:eye_dist()
  return self.eye:dist(self.center)
end



function UpCamera:set_center(x, y, z)
  self.center[1] = x
  self.center[2] = y
  self.center[3] = z
end



