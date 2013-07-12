local gl = require './gl'
local glfw = require './glfw'

local ModelWalkerWidget = Class(ui.GLWidget)

local NAV_MODE = 1
local FOCUS_MODE = 2

function ModelWalkerWidget:__init(width, height)
  ui.GLWidget.__init(self, width, height, 'Model Walker')
  self.renderer = ui.Renderer.new(self)

  self.mode = NAV_MODE

  local eye = torch.Tensor{0,0,80}
  local center = torch.Tensor{0.1,0.1,0}
  local viewport_camera = self.renderer:create_camera('viewport_camera', ui.UpCamera)
  viewport_camera.vfov = (math.pi/4)
  viewport_camera:resize(width, height)
  viewport_camera:set_eye(eye[1], eye[2], eye[3])
  viewport_camera:set_center(center[1], center[2], center[3])
  self.renderer:activate_camera('viewport_camera')
end

function ModelWalkerWidget:resize(width, height)
  glfw.SetWindowSize(self.window, width, height)
  self.renderer.cameras.viewport_camera:resize(width, height)
end

function ModelWalkerWidget:mouse_click(button, mods, x, y)
  if button == glfw.MOUSE_BUTTON_LEFT then
    self.mode = NAV_MODE
    rotateMode = false
    -- FlyTo Behavior
    local clicked_pos_world = self.renderer.cameras.viewport_camera:pixel_to_world(x, y)
    if clicked_pos_world ~= nil then
      self:fly_to(clicked_pos_world)
    end
    
  elseif button == glfw.MOUSE_BUTTON_RIGHT then
    self.mode = FOCUS_MODE
    local object_id, triangle_index = self.renderer.cameras.viewport_camera.frame_buffer:read_pick_pixel(self.mouse_x, self.mouse_y)
    local object = self.renderer.scenes.viewport_scene[object_id]
    local verts, center, normal = object:get_triangle(triangle_index)

    -- todo: animate
    self.renderer.cameras.viewport_camera.center[{{1,3}}] = center[{{1,3}}]
    self.renderer.cameras.viewport_camera.eye[{{1,3}}] = center[{{1,3}}] + (normal * 5)
  end

  self:update()
end

function ModelWalkerWidget:mouse_drag(button, x, y)
  if button == glfw.MOUSE_BUTTON_RIGHT then
    -- p('mouse_move', event)
    local dx = x - self.drag_last_x
    local dy = y - self.drag_last_y

    local  rotation_speed = 1;
    if self.mode == NAV_MODE then
      self.renderer.cameras.viewport_camera:rotate_center_around_eye(dx*rotation_speed, dy*rotation_speed)
    else
      self.renderer.cameras.viewport_camera:rotate_eye_around_center(dx*rotation_speed, dy*rotation_speed)
    end

    self.drag_last_x = x
    self.drag_last_y = y
    self:update()
  end
end

function ModelWalkerWidget:mouse_scroll(xoffset, yoffset)
  -- p('mouse_wheel', event)
  local move_units = yoffset / 60
  self.renderer.cameras.viewport_camera:move_eye(0, 0, move_units)
  self:update()
end

function ModelWalkerWidget:key_press(key, scancode, action, mods)
  -- p('key_press', event)
  if key == glfw.KEY_LEFT then
    self.renderer.cameras.viewport_camera:move_eye(-0.3,0,0)
  elseif key == glfw.KEY_RIGHT then
    self.renderer.cameras.viewport_camera:move_eye(0.3,0,0)
  elseif key == glfw.KEY_UP then
    self.renderer.cameras.viewport_camera:move_eye(0,0.3,0)
  elseif key == glfw.KEY_DOWN then
    self.renderer.cameras.viewport_camera:move_eye(0,-0.3,0)
  end
  self:update()
end

function ModelWalkerWidget:fly_to(goal_position)
  local VIEW_DISTANCE = 8
  local VIEW_HEIGHT = 2.25
  local eye_offset_direction = geom.util.direction(goal_position, self.renderer.cameras.viewport_camera.eye)
  local eye_position = goal_position - eye_offset_direction * VIEW_DISTANCE
  
  --TODO: {0,-0.01, -1} idealy would be {0,0,-1} to raycast straight down. 
  -- Without this slight tilt raycast always returns nil. An issue with the up vector? camera matrix?
  local hit_location = self.renderer:raycast(eye_position, torch.Tensor({0,0.01,-1}))
  if hit_location ~= nil then
    eye_position[3] = hit_location[3] + VIEW_HEIGHT
  end

  self.renderer.animation_manager:add(self.renderer.cameras.viewport_camera.center, goal_position, 500, ui.AnimationManager.BEZIER_START_BEHAVIORS.FAST, ui.AnimationManager.BEZIER_END_BEHAVIORS.SLOW)
  self.renderer.animation_manager:add(self.renderer.cameras.viewport_camera.eye, eye_position, 500, ui.AnimationManager.BEZIER_START_BEHAVIORS.FAST, ui.AnimationManager.BEZIER_END_BEHAVIORS.SLOW)
end

