local gl = require './gl'
local glfw = require './glfw'

local GLWidget = Class()

local NAV_MODE = 1
local FOCUS_MODE = 2

function GLWidget:__init(gl_init_callback)
  self.gl_init_callback = gl_init_callback

  self.window = glfw.CreateWindow(640, 480, "My Title", nil, nil)
  self:make_current()
  
  log.info("OpenGL "..gl.GetString(gl.VERSION).." GLSL "..gl.GetString(gl.SHADING_LANGUAGE_VERSION))

  self.renderer = ui.Renderer.new(self)
  self.mode = NAV_MODE

  self:resize(640, 480)
  
  self:setup_callbacks()

  if self.gl_init_callback then self.gl_init_callback(self) end
end

function GLWidget:setup_callbacks()
  glfw.SetKeyCallback(self.window, function(window, key, scancode, action, mods) 
    self:key_press(key, scancode, action, mods) 
  end)

  glfw.SetFramebufferSizeCallback(self.window, function(window, width, height) 
    self:resize(width, height) 
  end)

  glfw.SetCursorPosCallback(self.window, function(window, x, y)
    self:mouse_move(x,y)
  end)

  glfw.SetMouseButtonCallback(self.window, function(window, button, action, mods)
    if action == glfw.PRESS then self:mouse_press(button, mods) else self:mouse_release(button, mods) end
  end)

  glfw.SetScrollCallback(self.window, function(window, button, action, mods)

  end)
end

function GLWidget:make_current()
  glfw.MakeContextCurrent(self.window)
end

function GLWidget:__gc()
  glfw.DestroyWindow(self.window)
end

function GLWidget:update()
  self:paint()
end

function GLWidget:resize(width, height)
  log.trace("resize")
  if self.renderer.cameras.viewport_camera == nil then
    self.renderer:init(width, height)
  else
    self.renderer.cameras.viewport_camera:resize(width, height)
  end

  -- this is the last thing that happens after the widget is created
  self.initialized = true
end

function GLWidget:paint()
  self.renderer:render()
  glfw.SwapBuffers(self.window)
end

function GLWidget:mouse_press(button, mods)
  -- p('mouse_press', event)
  self.drag_start_x = self.last_x
  self.drag_start_y = self.last_y
  self.drag_last_x = self.last_x
  self.drag_last_y = self.last_y

  if button == glfw.MOUSE_BUTTON_LEFT then self.left_button = true end
  if button == glfw.MOUSE_BUTTON_RIGHT then self.right_button = true end
end

function GLWidget:mouse_release(button, mods)
  -- redirtect simple clicks
  if self.drag_start_x == self.last_x and self.drag_start_y == self.last_y then 
    return self:mouse_click(button, mods)
  end

  if button == glfw.MOUSE_BUTTON_LEFT then self.left_button = false end
  if button == glfw.MOUSE_BUTTON_RIGHT then self.right_button = false end
end

function GLWidget:mouse_click(button, mods)
  if button == glfw.MOUSE_BUTTON_LEFT then
    self.mode = NAV_MODE
    rotateMode = false
    -- FlyTo Behavior
    local clicked_pos_world = self.renderer.cameras.viewport_camera:pixel_to_world(self.last_x, self.last_y)
    if clicked_pos_world ~= nil then
      self:fly_to(clicked_pos_world)
    end
    
  elseif button == glfw.MOUSE_BUTTON_RIGHT then
    self.mode = FOCUS_MODE
    local object_id, triangle_index = self.renderer.cameras.viewport_camera.frame_buffer:read_pick_pixel(self.last_x, self.last_y)
    local object = self.renderer.scenes.viewport_scene[object_id]
    local verts, center, normal = object:get_triangle(triangle_index)

    -- todo: animate
    self.renderer.cameras.viewport_camera.center[{{1,3}}] = center
    self.renderer.cameras.viewport_camera.eye[{{1,3}}] = center + (normal * 5)
  end

  self:update()
end

function GLWidget:mouse_move(x, y)
  self.last_x = x
  self.last_y = y

  if self.right_button then
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

function GLWidget:mouse_wheel(event)
  -- p('mouse_wheel', event)
  local move_units = event.delta / 120
  self.renderer.cameras.viewport_camera:move_eye(0, 0, move_units)
  self:update()
end

function GLWidget:key_press(key, scancode, action, mods)
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

function GLWidget:fly_to(goal_position)
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

  self.renderer.animation_manager:add(self.renderer.cameras.viewport_camera.center, goal_position, 0.5, ui.AnimationManager.BEZIER_START_BEHAVIORS.FAST, ui.AnimationManager.BEZIER_END_BEHAVIORS.SLOW)
  self.renderer.animation_manager:add(self.renderer.cameras.viewport_camera.eye, eye_position, 0.5, ui.AnimationManager.BEZIER_START_BEHAVIORS.FAST, ui.AnimationManager.BEZIER_END_BEHAVIORS.SLOW)
  self:update()
end