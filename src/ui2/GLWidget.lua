local ffi = require 'ffi'
local libui = require 'libui2'
local gl = require 'ui2.gl'

local Shader = require 'ui2.Shader'

local GLWidget = torch.class('GLWidget')

function GLWidget:__init()
  libui.attach_qt(self)

  self.camera = require('ui2.Camera').new()
end

function GLWidget:init()
  print("OpenGL "..ffi.string(gl.GetString(gl.VERSION)).." GLSL "..ffi.string(gl.GetString(gl.SHADING_LANGUAGE_VERSION)))

  self.textured_shader = Shader.new('textured') 
  self.shadow_shader = Shader.new('shadow') 
  self.identity_shader = Shader.new('identity') 

end

function GLWidget:resize(width, height)
  print('resize', width, height)
  gl.Viewport(0, 0, width, height)
  self.camera:update_projection_matrix()
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

  -- show objects

  
  gl.Disable(gl.BLEND)
  gl.Disable(gl.DEPTH_TEST)
  gl.check_errors()
end

function GLWidget:mouse_press(event)
  p('mouse_press', event)
end

function GLWidget:mouse_release(event)
  p('mouse_release', event)
end

function GLWidget:mouse_move(event)
  p('mouse_move', event)
end

function GLWidget:mouse_wheel(event)
  p('mouse_wheel', event)
end

return GLWidget
