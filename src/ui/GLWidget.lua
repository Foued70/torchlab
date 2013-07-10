local gl = require './gl'
local glfw = require './glfw'
local timer = require 'timer'
local uv = require 'uv_native'

local GLWidget = Class()

function GLWidget:__init(width, height, title)
  self.window = glfw.CreateWindow(width, height, title, nil, nil)
  self:make_current()
  
  log.info("OpenGL "..gl.GetString(gl.VERSION).." GLSL "..gl.GetString(gl.SHADING_LANGUAGE_VERSION))

  self:setup_callbacks()
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

  glfw.SetScrollCallback(self.window, function(window, xoffset, yoffset)
    self:mouse_scroll(xoffset, yoffset)
  end)
end

function GLWidget:make_current()
  glfw.MakeContextCurrent(self.window)
end

function GLWidget:__gc()
  glfw.DestroyWindow(self.window)
end

function GLWidget:update()
  timer.setTimeout(1, function()
    self:paint()
  end)
end

function GLWidget:resize(width, height)
  glfw.SetWindowSize(self.window, width, height)
  if self.renderer.cameras.viewport_camera == nil then
    self.renderer:init(width, height)
  else
    self.renderer.cameras.viewport_camera:resize(width, height)
  end
end

function GLWidget:paint()
  self.renderer:render()
  glfw.SwapBuffers(self.window)
end

function GLWidget:mouse_press(button, mods)
end

function GLWidget:mouse_release(button, mods)
end

function GLWidget:mouse_click(button, mods)
end

function GLWidget:mouse_move(x, y)
end

function GLWidget:mouse_scroll(xoffset, yoffset)
end

function GLWidget:key_press(key, scancode, action, mods)
end

