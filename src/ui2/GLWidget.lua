local libui = require 'libui2'
local gl = require 'ui2.gl'
local key = require 'ui2.key'
local geom = require 'util.geom'

local Shader = require 'ui2.Shader'
local MatrixStack = require 'ui2.MatrixStack'

local GLWidget = torch.class('GLWidget')

local NAV_MODE = 1
local FOCUS_MODE = 2

function GLWidget:__init()
  self.initialized = false
  libui.attach_qt(self)

  self.camera = require('ui2.Camera').new()
  self.camera:set_eye(2,3,5)
  self.camera:set_center(0,0,1)
  
  self.camera_raycaster = require('ui2.Camera').new()
  self.camera_raycaster:set_eye(0,0,0)
  self.camera_raycaster:set_center(0,0,1)
  self.camera_raycaster:update()

  self.mode = NAV_MODE

  self.objects = {}

  self.context = MatrixStack.new()
  
  self.animationManager = require('ui2.AnimationManager').new()
end

function GLWidget:init(qt_widget)
  log.info("OpenGL "..gl.GetString(gl.VERSION).." GLSL "..gl.GetString(gl.SHADING_LANGUAGE_VERSION))

  self.qt_widget = qt_widget

  self.textured_shader = Shader.new('textured')
end

function GLWidget:update()
  libui.update_gl(self.qt_widget)
end

function GLWidget:resize(width, height)
  log.trace('resize', width, height)
  gl.Viewport(0, 0, width, height)
  self.camera:update_projection_matrix(self.context)
  self.camera_raycaster:update_projection_matrix(self.context)

  self.initialized = true
end

function GLWidget:paint()
  if not self.initialized then return end

  self.camera.frame_buffer:use()

  gl.Clear(gl.COLOR_BUFFER_BIT + gl.DEPTH_BUFFER_BIT)
  gl.ClearColor(0, 0, 0, 0)
  gl.check_errors()

  gl.Enable(gl.BLEND)
  gl.BlendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA)
  gl.check_errors()

  gl.Enable(gl.DEPTH_TEST)
  gl.check_errors()

  gl.CullFace(gl.BACK)
  gl.Enable(gl.CULL_FACE)
  gl.check_errors()

  gl.Enable(gl.MULTISAMPLE)
  gl.check_errors()

  -- setup camera
  self.camera:update_matrix(self.context)

  -- show objects
  for i, object in ipairs(self.objects) do
    self.context.object_id = i
    object:paint(self.context)
  end

  self.camera.frame_buffer:display()

  gl.Disable(gl.BLEND)
  gl.Disable(gl.DEPTH_TEST)
  gl.check_errors()

  self.camera.frame_buffer:unbind()
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
  if event.right_button then
    self.mode = NAV_MODE
    rotateMode = false
    -- FlyTo Behavior
    local clicked_pos_world = self.camera:pixel_to_world(event.x, event.y)
    if clicked_pos_world ~= nil then
      self:fly_to(clicked_pos_world)
    end
    
  elseif event.left_button then
    self.mode = FOCUS_MODE
    local object_id, triangle_index = self.camera.frame_buffer:read_pick_pixel(event.x, event.y)
    local object = self.objects[object_id]
    local verts, center, normal = object:get_triangle(triangle_index)

    -- todo: animate
    self.camera.center[{{1,3}}] = center
    self.camera.eye[{{1,3}}] = center + (normal * 5)
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
      self.camera:rotate_center_around_eye(dx*rotation_speed, dy*rotation_speed)
    else
      self.camera:rotate_eye_around_center(dx*rotation_speed, dy*rotation_speed)
    end

    self.drag_last_x = event.global_x
    self.drag_last_y = event.global_y
    self:update()
  end

end

function GLWidget:mouse_wheel(event)
  -- p('mouse_wheel', event)
  local move_units = event.delta / 120
  self.camera:move_eye(0, 0, move_units)
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

function GLWidget:add_object(data_obj)
  table.insert(self.objects, require('ui2.Object').new(data_obj))
  self:update()
end

function GLWidget:raycast(start, direction)
  self.camera_raycaster.eye[{{1,3}}] = start
  torch.add(self.camera_raycaster.center, start, direction)
  self.camera_raycaster:update()
  
  log.trace(self.camera_raycaster.eye)
  log.trace(self.camera_raycaster.center)
  
  self.camera_raycaster.frame_buffer:use()

  gl.Clear(gl.DEPTH_BUFFER_BIT)
  gl.ClearColor(0, 0, 0, 0)
  gl.check_errors()

  gl.Enable(gl.DEPTH_TEST)
  gl.check_errors()

  gl.CullFace(gl.BACK)
  gl.Enable(gl.CULL_FACE)
  gl.check_errors()

  -- setup camera
  self.camera_raycaster:update_matrix(self.context)

  -- show objects
  for i, object in ipairs(self.objects) do
    self.context.object_id = i
    object:paint(self.context)
  end

  gl.Disable(gl.DEPTH_TEST)
  gl.check_errors()

  self.camera_raycaster.frame_buffer:unbind()
  
  local hit_location = self.camera_raycaster:pixel_to_world(self.camera_raycaster.width*0.5, self.camera_raycaster.height*0.5)
  log.trace(hit_location)
  return hit_location
end


function GLWidget:fly_to(goal_position)
  local VIEW_DISTANCE = 8
  local VIEW_HEIGHT = 2.25
  local eye_offset_direction = geom.direction(goal_position, self.camera.eye)
  local eye_position = goal_position - eye_offset_direction * VIEW_DISTANCE
  
  --TODO: {0,-0.01, -1} idealy would be {0,0,-1} to raycast straight down. 
  -- Without this slight tilt raycast always returns nil. An issue with the up vector? camera matrix?
  local hit_location = self:raycast(eye_position, torch.Tensor({0,0.01,-1}))
  log.trace(hit_location)
  if hit_location ~= nil then
    eye_position[3] = hit_location[3] + VIEW_HEIGHT
    log.trace(eye_position)
  end
  
  self.camera.center[{{1,3}}] = goal_position
  self.camera.eye[{{1,3}}] = eye_position
  
  --self.animationManager:updateState()
end

return GLWidget
