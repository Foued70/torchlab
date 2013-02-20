local ffi = require 'ffi'
local libui = require 'libui2'
local gl = require 'ui2.gl'

local Shader = require 'ui2.Shader'

local GLWidget = torch.class('GLWidget')

function GLWidget:__init()
  libui.attach_qt(self)

  self.camera = require('ui2.Camera').new()
  self.camera:set_eye(-20, 0, 7)
  self.camera:set_center(0, 0, 7)

  self.objects = {}

  self.context = {}
  self.context.model_view_projection_matrix = torch.FloatTensor(4,4)
end

function GLWidget:init(qt_widget)
  log.info("OpenGL "..ffi.string(gl.GetString(gl.VERSION)).." GLSL "..ffi.string(gl.GetString(gl.SHADING_LANGUAGE_VERSION)))

  self.qt_widget = qt_widget

  -- self.textured_shader = Shader.new('textured') 
  -- self.shadow_shader = Shader.new('shadow') 
  self.identity_shader = Shader.new('identity') 

end

function GLWidget:update()
  libui.update_gl(self.qt_widget)
end

function GLWidget:resize(width, height)
  log.trace('resize', width, height)
  gl.Viewport(0, 0, width, height)
  self.camera:update_projection_matrix()
end

function GLWidget:paint()
  log.trace()
  gl.Clear(gl.COLOR_BUFFER_BIT)
  gl.Clear(gl.DEPTH_BUFFER_BIT)
  gl.ClearColor(0, 0, 0, 0)
  gl.check_errors()

  gl.Enable(gl.BLEND)
  gl.BlendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA)
  gl.check_errors()
  
  gl.Enable(gl.DEPTH_TEST)
  gl.check_errors()
  
  -- gl.CullFace(gl.BACK)
  -- gl.Enable(gl.CULL_FACE)
  -- gl.check_errors()
  
  gl.Enable(gl.MULTISAMPLE)
  gl.check_errors()

  local context = {}
  -- setup camera
  self.camera:update_matrices()
  context.projection_matrix = self.camera.projection_matrix
  context.model_view_matrix = self.camera.model_view_matrix
  context.normal_matrix = self.camera.normal_matrix
  context.model_view_projection_matrix = torch.FloatTensor(4,4)

  -- show objects
  for i, object in ipairs(self.objects) do
    object:paint(context)
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

return GLWidget
