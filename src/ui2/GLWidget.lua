local libui = require 'libui2'
local gl = require 'ui2.gl'
local key = require 'ui2.key'
local geom = require 'util.geom'

require 'qt'
require 'qttorch'
require 'image'
require 'qtwidget'
require 'qtuiloader'

local GLWidget = torch.class('GLWidget')

local NAV_MODE = 1
local FOCUS_MODE = 2

function GLWidget:__init()
  self.initialized = false
  libui.attach_qt(self)

  self.renderer = require('ui2.Renderer').new(self)
  self.mode = NAV_MODE
end

function GLWidget:init(qt_widget)
  log.info("OpenGL "..gl.GetString(gl.VERSION).." GLSL "..gl.GetString(gl.SHADING_LANGUAGE_VERSION))
  self.qt_widget = qt_widget
end

function GLWidget:update()
  libui.update_gl(self.qt_widget)
end

function GLWidget:resize(width, height)
  log.trace("resize")
  if self.initialized == false then
    self.renderer:init(width, height)
    self.initialized = true
  end
  
  if self.renderer.cameras.viewport_camera ~= nil then
    self.renderer.cameras.viewport_camera:resize(width, height)
  end
end

function GLWidget:paint()
  if not self.initialized then return end

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
    local object = self.renderer.active_scene[object_id]
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
    self.camera:move_eye(-0.3,0,0)
  elseif event.key == key.Right then
    self.camera:move_eye(0.3,0,0)
  elseif event.key == key.Up then
    self.camera:move_eye(0,0.3,0)
  elseif event.key == key.Down then
    self.camera:move_eye(0,-0.3,0)
  end
  self:update()
end

function GLWidget:fly_to(goal_position)
  local VIEW_DISTANCE = 8
  local VIEW_HEIGHT = 2.25
  local eye_offset_direction = geom.direction(goal_position, self.renderer.cameras.viewport_camera.eye)
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

return GLWidget
