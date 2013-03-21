local libui = require 'libui2'
local gl = require 'ui2.gl'
local key = require 'ui2.key'
local geom = require 'util.geom'

local Shader = require 'ui2.Shader'
local MatrixStack = require 'ui2.MatrixStack'

local Renderer = torch.class('Renderer')

function Renderer:__init(parent, width, height)
  self.parent = parent
  self.cameras = {}
  self.active_camera = nil
  self.scenes = {}
  self.active_scene = nil

  self.materials = {}
  self.shaders = {}
  self.context = MatrixStack.new()
  self.texture_manager = require('ui2.TextureManager').new()
  self.animation_manager = require('ui2.AnimationManager').new()

  log.trace("Renderer Constructed")
end

function Renderer:init(viewport_width, viewport_height)
  self:create_camera('viewport_camera', viewport_width, viewport_height, (math.pi/4), torch.Tensor({2,3,5}), torch.Tensor({0,1,0}))
  self:activate_camera('viewport_camera')

  self:create_camera('raycast_camera', viewport_width, viewport_height)  
  self:create_shaders()

  self:create_scene('viewport_scene')
  self:activate_scene('viewport_scene')
end

function Renderer:render()
  if (self.active_camera == nil) or (self.active_scene == nil) then
    return
  end

  self.active_camera.frame_buffer:use()
  --log.trace("Rendering with camera", self.active_camera.name)

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
  self.active_camera:update_matrix(self.context)

  -- show objects
  for i, object in ipairs(self.active_scene) do
    self.context.object_id = i
    self.context.screen_width = self.active_camera.width
    self.context.screen_height = self.active_camera.height
    object:paint(self.context)
  end

  gl.Disable(gl.BLEND)
  gl.Disable(gl.DEPTH_TEST)
  gl.check_errors()

  self.active_camera.frame_buffer:unbind()

  if self.active_camera == self.cameras.viewport_camera then
    self.active_camera.frame_buffer:display()

    if self.animation_manager:needsAnimating() == true then
      self.animation_manager:tick_all()
      self.parent:update()
    end
  end
end

function Renderer:create_camera(name, width, height, fov_y, eye, center)
  local camera = require('ui2.Camera').new(self, name)
  if fov_y then camera.fov_y = fov_y end
  local eye_position = eye or torch.Tensor({0,0,0})
  local center_position = center or torch.Tensor({0,1,0})

  camera:set_eye(eye_position[1], eye_position[2], eye_position[3])
  camera:set_center(center_position[1], center_position[2], center_position[3])

  camera:resize(width, height)
  camera:update()
  self.cameras[camera.name] = camera
  log.trace("Created camera", name)
end

function Renderer:activate_camera(camera_name)
  if self.cameras[camera_name] == nil then
    log.trace(camera_name, " does not exist! " .. camera_name .. " not activated.")
    return
  end
  self.active_camera = self.cameras[camera_name]
  gl.Viewport(0,0, self.active_camera.width, self.active_camera.height)
end

function Renderer:create_scene(scene_name)
  if self.scenes[scene_name] ~= nil then
    log.trace(scene_name .. " already exists! No scene created.")
    return
  end
  local objects = {}
  self.scenes[scene_name] = objects
end

function Renderer:activate_scene(scene_name)
  if self.scenes[scene_name] == nil then
    log.trace(scene_name .. " does not exist! " .. scene_name .. " not activated.")
    return
  end
  self.active_scene = self.scenes[scene_name]
end

function Renderer:create_shader(name)
  log.trace("Creating shader:", name)
  self.shaders[name] = Shader.new(name)
  log.trace("Shader", name, "created")
end

function Renderer:create_shaders()
  --TODO: look at all shaders in directory(or table holding list of shaders) and create them
  self:create_shader('textured')
  self:create_shader('vertex_highlight')
  self:create_shader('wireframe')
end

function Renderer:add_object(data_obj)
  local object = require('ui2.Object').new(self, data_obj)
  table.insert(self.active_scene, object)
  return object
end

function Renderer:add_material(material)
  table.insert(self.materials, material)
end

function Renderer:create_material(data_mat, shader, textures)
  local material = require('ui2.Material').new(self, data_mat)
  material:set_shader(shader)
  for t = 1, #textures do
    material:attach_texture(textures[t], t-1)
  end
  self:add_material(material)
  return material
end

function Renderer:raycast(start, direction)
  if self.active_scene == nil then
    return false
  end

  self.cameras.raycast_camera.eye[{{1,3}}] = start
  torch.add(self.cameras.raycast_camera.center, start, direction)
  self.cameras.raycast_camera:update()
  
  self.cameras.raycast_camera.frame_buffer:use()

  gl.Clear(gl.DEPTH_BUFFER_BIT)
  gl.ClearColor(0, 0, 0, 0)
  gl.check_errors()

  gl.Enable(gl.DEPTH_TEST)
  gl.check_errors()

  gl.CullFace(gl.BACK)
  gl.Enable(gl.CULL_FACE)
  gl.check_errors()

  -- setup camera
  self.cameras.raycast_camera:update_matrix(self.context)

  -- show objects
  for i, object in ipairs(self.active_scene) do
    self.context.object_id = i
    object:paint(self.context)
  end

  gl.Disable(gl.DEPTH_TEST)
  gl.check_errors()

  self.cameras.raycast_camera.frame_buffer:unbind()
  
  local hit_location = self.cameras.raycast_camera:pixel_to_world(self.cameras.raycast_camera.width*0.5, self.cameras.raycast_camera.height*0.5)
  return hit_location
end

function Renderer:pick_vertex(screen_position, selection_radius)
  local pixel_coord_x = math.floor(self.cameras.viewport_camera.frame_buffer.width * ((screen_position[1]+1)*0.5))
  local pixel_coord_y = math.floor(self.cameras.viewport_camera.frame_buffer.height * ((screen_position[2]+1)*0.5))
  log.trace("screen_position")
  log.trace(screen_position)
  log.trace("pixel_coord_x")
  log.trace(pixel_coord_x)
  log.trace("pixel_coord_y")
  log.trace(pixel_coord_y)
  local object_id, triangle_index = self.cameras.viewport_camera.frame_buffer:read_pick_pixel(pixel_coord_x, pixel_coord_y)
  log.trace()
  local object = self.scenes.viewport_scene[object_id]
  local verts, center, normal = object:get_triangle(triangle_index)

  local selection = nil
  for i, vertex in ipairs(verts) do
    local vertex_screen_space = self.cameras.viewport_camera:world_to_screen(vertex)
    local offset = vertex_screen_space - screen_position
    if math.sqrt((offset[1]*offset[1])+(offset[2]*offset[2])) <= selection_radius then
      selection = vertex
      break
    end
  end
  return selection
end

return Renderer
