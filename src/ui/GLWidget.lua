local libui = require 'libui'
local gl = require 'ui.gl'
local key = require 'ui.key'

require 'qt'
require 'qttorch'
require 'image'
require 'qtwidget'
require 'qtuiloader'

local GLWidget = Class()

local NAV_MODE = 1
local FOCUS_MODE = 2

function GLWidget:__init(gl_init_callback)
  self.gl_init_callback = gl_init_callback
  self.initialized = false

  self.renderer = ui.Renderer.new(self)
  self.mode = NAV_MODE

  qt.qcall(qt.qApp, libui.attach_qt, self)
end

function GLWidget:wait_for_init()
  while self.initialized ~= true do
    qt.doevents(false)
  end

end

function GLWidget:init(qt_widget)
  log.info("OpenGL "..gl.GetString(gl.VERSION).." GLSL "..gl.GetString(gl.SHADING_LANGUAGE_VERSION))
  self.qt_widget = qt_widget

  if self.gl_init_callback then self.gl_init_callback(self) end
end

function GLWidget:update()
  libui.update_gl(self.qt_widget)
end

function GLWidget:resize_widget(width, height)
  libui.widget_resize(self.qt_widget, width, height)
end

function GLWidget:resize(width, height)
  log.trace("resize")
  if self.initialized == false then
    self.renderer:init(width, height)
  end
  
  if self.renderer.cameras.viewport_camera ~= nil then
    self.renderer.cameras.viewport_camera:resize(width, height)
  end

  -- this is the last thing that happens after the widget is created
  self.initialized = true
end

function GLWidget:paint()
  if not self.initialized then 
    log.info('not ready to paint')
    return 
  end

  self.renderer:render()
end

function GLWidget:mouse_press(event)
  -- p('mouse_press', event)
  self.drag_start_x = event.global_x
  self.drag_start_y = event.global_y
  self.drag_last_x = event.global_x
  self.drag_last_y = event.global_y
end

function GLWidget:mouse_release(event)
  -- redirtect simple clicks
  if self.drag_start_x == event.global_x and self.drag_start_y == event.global_y then 
    return self:mouse_click(event)
  end
end

function GLWidget:mouse_click(event)
  if event.left_button then
    self.mode = NAV_MODE
    rotateMode = false
    -- FlyTo Behavior
    local clicked_pos_world = self.renderer.cameras.viewport_camera:pixel_to_world(event.x, event.y)
    if clicked_pos_world ~= nil then
      self:fly_to(clicked_pos_world)
    end
    
  elseif event.right_button then
    self.mode = FOCUS_MODE
    local object_id, triangle_index = self.renderer.cameras.viewport_camera.frame_buffer:read_pick_pixel(event.x, event.y)
    local object = self.renderer.scenes.viewport_scene[object_id]
    local verts, center, normal = object:get_triangle(triangle_index)

    -- todo: animate
    self.renderer.cameras.viewport_camera.center[{{1,3}}] = center
    self.renderer.cameras.viewport_camera.eye[{{1,3}}] = center + (normal * 5)
  end

  self:update()
end

function GLWidget:mouse_move(event)
  -- p('mouse_move', event)
  local dx = event.global_x - self.drag_last_x
  local dy = event.global_y - self.drag_last_y

  if event.right_button then
    local  rotation_speed = 1;
    if self.mode == NAV_MODE then
      self.renderer.cameras.viewport_camera:rotate_center_around_eye(dx*rotation_speed, dy*rotation_speed)
    else
      self.renderer.cameras.viewport_camera:rotate_eye_around_center(dx*rotation_speed, dy*rotation_speed)
    end

    self.drag_last_x = event.global_x
    self.drag_last_y = event.global_y
    self:update()
  end

end

function GLWidget:mouse_wheel(event)
  -- p('mouse_wheel', event)
  local move_units = event.delta / 120
  self.renderer.cameras.viewport_camera:move_eye(0, 0, move_units)
  self:update()
end

function GLWidget:key_press(event)
  -- p('key_press', event)
  if event.key == key.Left then
    self.renderer.cameras.viewport_camera:move_eye(-0.3,0,0)
  elseif event.key == key.Right then
    self.renderer.cameras.viewport_camera:move_eye(0.3,0,0)
  elseif event.key == key.Up then
    self.renderer.cameras.viewport_camera:move_eye(0,0.3,0)
  elseif event.key == key.Down then
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

  self.renderer.animation_manager:add(self.renderer.cameras.viewport_camera.center, goal_position, 0.5, BEZIER_START_BEHAVIORS.FAST, BEZIER_END_BEHAVIORS.SLOW)
  self.renderer.animation_manager:add(self.renderer.cameras.viewport_camera.eye, eye_position, 0.5, BEZIER_START_BEHAVIORS.FAST, BEZIER_END_BEHAVIORS.SLOW)
  self:update()
end