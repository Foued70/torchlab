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

  self:create_camera('raycast_camera', viewport_width, viewport_height, (math.pi/4))  
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
  local pixels = self.active_camera:screen_to_pixel(screen_position)
  pixels[1] = math.floor(pixels[1])
  pixels[2] = math.floor(pixels[2])
  
  local object_id, triangle_index = self.active_camera.frame_buffer:read_pick_pixel(pixels[1], pixels[2])
  
  local object = self.active_scene[object_id]
  local verts, center, normal = object:get_triangle(triangle_index)
  local vertex_positions = verts:narrow(2,1,3)

  local selection = nil
  local selection_distance = nil
  for i=1, 3 do
    local vertex_screen_space = self.active_camera:world_to_screen(vertex_positions[i])[{{1,2}}]
    if vertex_screen_space ~= nil then
      log.trace(vertex_screen_space)
      local offset = vertex_screen_space - screen_position
      local distance = math.sqrt((offset[1]*offset[1])+(offset[2]*offset[2]))
      if distance <= selection_radius then
        if selection ~= nil then
          if distance < selection_distance then
            selection = vertex_positions[i]
            selection_distance = distance
          end
        else
          selection = vertex_positions[i]
          selection_distance = distance
        end
      end
    end
  end

  log.trace(selection)
  return selection
end

function Renderer:pick_vertices(screen_position_min, screen_position_max)
  local pixels_min = self.active_camera:screen_to_pixel(screen_position_min)
  local pixels_max = self.active_camera:screen_to_pixel(screen_position_max)

  --Because Y is flipped in pixel space, we need to reverse min y and max y here
  local temp = pixels_min[2]
  pixels_min[2] = pixels_max[2]
  pixels_max[2] = temp

  local pick_image = self.active_camera.frame_buffer:read_pick_pixels(pixels_min, pixels_max)
  local pick_values = pick_image:resize(pick_image:size(1)*pick_image:size(2),2):double()

  --Use hashing to create a unique list of object, triangle pairs
  local hash_function = torch.Tensor({1, 10e3})
  local hashed_tensor = pick_values * hash_function

  local hash = {}
  local num_elements = 0
  for i = 1, hashed_tensor:size(1) do 
    if not hash[hashed_tensor[i]] then 
      hash[hashed_tensor[i]] = i 
      num_elements = num_elements + 1
    end 
  end

  local unique_picks = torch.Tensor(num_elements, pick_values:size(2))
  local j = 1
  for _,i in pairs(hash) do
      unique_picks[j] = pick_values[i]
      j = j + 1
  end

  local selected_vertices = {}
  for t=1, unique_picks:size(1) do
    --log.trace("index", t, "object", unique_picks[t][1], "triangle", unique_picks[t][2])
    local object_index = unique_picks[t][1]
    if object_index > 0 then
      local triangle_index = unique_picks[t][2]
      local object = self.active_scene[object_index]
      local verts, center, normal = object:get_triangle(triangle_index)
      local vertex_positions = verts:narrow(2,1,3)

      local selection = nil
      local selection_distance = nil
      for i=1, 3 do
        local vertex_screen_space = self.active_camera:world_to_screen(vertex_positions[i])
        if vertex_screen_space ~= nil then
          --Is vertex inside selection?
          if  (vertex_screen_space[1] >= screen_position_min[1]) and 
              (vertex_screen_space[1] <= screen_position_max[1]) and
              (vertex_screen_space[2] >= screen_position_min[2]) and 
              (vertex_screen_space[2] <= screen_position_max[2]) then
            table.insert(selected_vertices, vertex_positions[i])
          end
        end
      end
    end
  end

  if #selected_vertices > 0 then
    return selected_vertices
  end
  return nil
end

return Renderer
