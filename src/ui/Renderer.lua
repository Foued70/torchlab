local gl = require './gl'

local Renderer = Class()

function Renderer:__init(parent, width, height)
  self.parent = parent
  self.cameras = {}
  self.active_camera = nil
  self.scenes = {}
  self.active_scene = nil

  self.materials = {}
  self.shaders = {}
  self.context = ui.MatrixStack.new()
  self.texture_manager = ui.TextureManager.new()
  self.animation_manager = ui.AnimationManager.new()

  self:create_scene('viewport_scene')
  self:activate_scene('viewport_scene')

  self:create_shaders()
  self:create_raycast_camera()

  -- log.trace("Renderer Constructed")
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

function Renderer:create_camera(name, camera_class)
  local camera = camera_class.new(name)
  camera.widget = self
  self.cameras[name] = camera
  return camera
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
  self.scenes[scene_name] = {}
end

function Renderer:activate_scene(scene_name)
  if self.scenes[scene_name] == nil then
    log.trace(scene_name .. " does not exist! " .. scene_name .. " not activated.")
    return
  end
  self.active_scene = self.scenes[scene_name]
  -- log.trace("Scene "..scene_name.." activated.")
end

function Renderer:create_shader(name)
  -- log.trace("Creating shader:", name)
  self.shaders[name] = ui.Shader.new(name)
  -- log.trace("Shader", name, "created")
end

function Renderer:create_shaders()
  --TODO: look at all shaders in directory(or table holding list of shaders) and create them
  self:create_shader('textured')
  self:create_shader('vertex_highlight')
  self:create_shader('wireframe')
end

function Renderer:add_object(data_obj)
  local object = ui.Object.new(self)
  if data_obj then object.from_data(data_obj) end
  table.insert(self.active_scene, object)
  return object
end

function Renderer:add_material(material)
  table.insert(self.materials, material)
end

function Renderer:create_material(data_mat, shader, textures)
  local material = ui.Material.new(self, data_mat)
  material:set_shader(shader)
  for t, texture in ipairs(textures) do
    material:attach_texture(texture, t-1)
  end
  self:add_material(material)
  return material
end

function Renderer:create_raycast_camera()
  local raycast_camera = self:create_camera('raycast_camera', ui.Camera)
  raycast_camera.vfov = 0
  raycast_camera:resize(1, 1)
end

function Renderer:raycast(start, direction)
  if self.active_scene == nil then
    return false
  end

  local cam = self.cameras.raycast_camera
  cam.eye[{{1,3}}] = start
  local rot = geom.quaternion.angle_between(cam.look_dir, direction)
  cam:rotate(rot)
  
  cam.frame_buffer:use()

  gl.Clear(gl.DEPTH_BUFFER_BIT)
  gl.ClearColor(0, 0, 0, 0)
  gl.check_errors()

  gl.Enable(gl.DEPTH_TEST)
  gl.check_errors()

  gl.CullFace(gl.BACK)
  gl.Enable(gl.CULL_FACE)
  gl.check_errors()

  -- setup camera
  cam:update_matrix(self.context)

  -- show objects
  for i, object in ipairs(self.active_scene) do
    self.context.object_id = i
    object:paint(self.context)
  end

  gl.Disable(gl.DEPTH_TEST)
  gl.check_errors()

  cam.frame_buffer:unbind()
  
  local hit_location = cam:pixel_to_world(cam.width*0.5, cam.height*0.5)
  return hit_location
end

