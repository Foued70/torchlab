local libui = require 'libui2'
local gl = require 'ui2.gl'
local key = require 'ui2.key'

local Shader = require 'ui2.Shader'
local MatrixStack = require 'ui2.MatrixStack'

local GLWidget = torch.class('GLWidget')

function GLWidget:__init()
  libui.attach_qt(self)

  self.camera = require('ui2.Camera').new()
  self.camera:set_eye(2,3,5)
  self.camera:set_center(0,0,1)

  self.objects = {}

  self.context = MatrixStack.new()
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
end

function GLWidget:paint()
  gl.Clear(gl.COLOR_BUFFER_BIT)
  gl.Clear(gl.DEPTH_BUFFER_BIT)
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
    object:paint(self.context)
  end
  
  gl.Disable(gl.BLEND)
  gl.Disable(gl.DEPTH_TEST)
  gl.check_errors()
end

function GLWidget:mouse_press(event)
  -- p('mouse_press', event)
end

function GLWidget:mouse_release(event)
  -- p('mouse_release', event)
end

function GLWidget:mouse_move(event)
  -- p('mouse_move', event)
end

function GLWidget:mouse_wheel(event)
  -- p('mouse_wheel', event)
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

return GLWidget
