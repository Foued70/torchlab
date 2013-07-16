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
  glfw.SetWindowCloseCallback(self.window, function(window)
    self:__gc()
  end)

  glfw.SetFramebufferSizeCallback(self.window, function(window, width, height) 
    self:framebuffer_resized(width, height) 
  end)

  glfw.SetWindowSizeCallback (self.window, function(window, width, height) 
    self:resized(width, height) 
  end)


  glfw.SetKeyCallback(self.window, function(window, key, scancode, action, mods) 
    self:key_press(key, scancode, action, mods) 
  end)

  glfw.SetCursorPosCallback(self.window, function(window, x, y)
    self:mouse_move(x,y)

    if self.mouse_button then
      self:mouse_drag(self.mouse_button, x, y)
    end

    self.drag_last_x = x
    self.drag_last_y = y

  end)

  glfw.SetMouseButtonCallback(self.window, function(window, button, action, mods)
    local x, y = glfw.GetCursorPos(self.window)
    if action == glfw.PRESS then
      self.drag_start_x = x
      self.drag_start_y = y
      self.drag_last_x = x
      self.drag_last_y = y
      self.mouse_button = button

      self:mouse_press(button, mods, x, y) 
    else
      self:mouse_release(button, mods, x, y)
      -- redirect simple clicks
      if self.drag_start_x == x and self.drag_start_y == y then 
        self:mouse_click(button, mods, x, y)
      end

      self.mouse_button = nil
    end
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

function GLWidget:framebuffer_resized(width, height)
  -- log.trace(width, height)

  if self.renderer.cameras.viewport_camera then
    self.renderer.cameras.viewport_camera:resize_framebuffer(width, height)
  end
  
end

function GLWidget:resized(width, height)
  -- log.trace(width, height)
  self.window_width = width
  self.window_height = height

  if self.renderer.cameras.viewport_camera then
    self.renderer.cameras.viewport_camera:resize(width, height)
  end

  self:update()
end

function GLWidget:resize(width, height)
  glfw.SetWindowSize(self.window, width, height)
end

function GLWidget:paint()
  self.renderer:render()
  glfw.SwapBuffers(self.window)
end

function GLWidget:mouse_press(button, mods, x, y)
end

function GLWidget:mouse_release(button, mods, x, y)
end

function GLWidget:mouse_click(button, mods, x, y)
end

function GLWidget:mouse_move(x, y)
end

function GLWidget:mouse_drag(button, x, y)
end

function GLWidget:mouse_scroll(xoffset, yoffset)
end

function GLWidget:key_press(key, scancode, action, mods)
end

