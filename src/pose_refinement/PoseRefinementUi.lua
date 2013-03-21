require 'torch'
require 'image'
require 'paths'
require 'qtuiloader'
require 'qtwidget'
require 'qt'

local geom = require 'util.geom'
local config = require 'ui2/pp_config'
local Scan = require 'Scan'

local PoseRefinementUi = torch.class('PoseRefinementUi')

function PoseRefinementUi:__init()
  self.gl_viewport = nil
  self.viewport_width = nil
  self.viewport_height = nil

  self.scan_folder = nil
  self.pose_file = nil 
  self.scan = nil
  self.current_sweep = nil
  self.current_camera = nil

  self.description = paths.thisfile('pose_refinement.ui')

  self.widget = qtuiloader.load(self.description)
  self.painter = qt.QtLuaPainter(self.widget.calibration_view)

  self.layers = {}
  self.magnifier = nil

  self.widget.mouseTracking = false
  self.mouse_left_down = false
  
  self.listener = qt.QtLuaListener(self.widget)
  self:init_event_handling()
  
  self.widget:show()
end

function PoseRefinementUi:init_event_handling()
  qt.connect(self.listener,'sigClose()',
    function()
      self.widget:deleteLater()
    end )

  qt.connect(self.listener, 'sigPaint()',
    function()
      self:paint()
    end )

  qt.connect(self.listener, 'sigMousePress(int,int,QByteArray,QByteArray,QByteArray)',
    function(x, y, mouse_button, keyboard_modifier, keys)
      self.mouse_left_down = true
      self.widget.mouseTracking = true
    end )

  qt.connect(self.listener, 'sigMouseRelease(int,int,QByteArray,QByteArray,QByteArray)',
    function(x, y, mouse_button, keyboard_modifier, keys)
      self.mouse_left_down = false
      self.widget.mouseTracking = false
      self.magnifier = nil
      p(self.gl_viewport.renderer:pick_vertex(torch.Tensor({x/self.widget.calibration_view.x,y/self.widget.calibration_view.y}), (1/400)))
    end )

  qt.connect(self.listener, 'sigMouseMove(int,int,QByteArray,QByteArray)',
    function(x, y, keyboard_modifier, keys)
      if self.mouse_left_down == true then
        self:update_magnifier(1.0, 1, x-self.widget.calibration_view.x, y-self.widget.calibration_view.y, 100, 100)
      end
    end )

  qt.connect(self.widget.button_scan_folder, 'clicked()', function() self:set_scan_folder() end)
  qt.connect(self.widget.button_pose_file, 'clicked()', function() self:set_pose_file() end)
  qt.connect(self.widget.button_init_calibration, 'clicked()', function() self:init_calibration() end)
  qt.connect(self.widget.button_previous, 'clicked()', function() self:previous_camera() end)
  qt.connect(self.widget.button_next, 'clicked()', function() self:next_camera() end)
end

function PoseRefinementUi:set_scan_folder()
  self.scan_folder = qt.QFileDialog.getExistingDirectory(self.widget, "Select Scan Directory"):tostring()
  
  if not self.scan_folder or string.len(self.scan_folder) == 0 or not paths.dirp(self.scan_folder) then 
    log.trace("Error loading scan folder. No scan folder set.")
    self.scan_folder = nil
    return false
  end

  return true
end

function PoseRefinementUi:set_pose_file()
  self.pose_file = qt.QFileDialog.getOpenFileName(self.widget, "Select Pose File", '', "Text Files (*.txt)"):tostring()
  
  if not self.pose_file or string.len(self.pose_file) == 0 or not paths.filep(self.pose_file) then 
    log.trace("Error loading pose file. No pose file set.")
    self.pose_file = nil
    return false
  end
  return true
end

function PoseRefinementUi:init_calibration()
  if (self.scan_folder == nil) then
    log.trace("Failed to start calibration. No scan folder set.")
    return false
  end

  if (self.pose_file == nil) then
    log.trace("Failed to start calibration. No pose file set.")
    return false
  end

  log.trace("Loading scan at", sys.clock())
  self.scan = Scan.new(self.scan_folder, self.pose_file)
  log.trace("Completed scan load at", sys.clock())

  log.trace("Loading model data at", sys.clock())
  self.scan:load_model_data()
  log.trace("Completed model load at", sys.clock())

  collectgarbage()
  self.gl_viewport = require('ui2.GLWidget').new()
  while self.gl_viewport.initialized ~= true do
    os.execute("sleep " .. tonumber(1))
  end
  collectgarbage()

  require('libui2').make_current(self.gl_viewport.qt_widget)
  require('libui2').hide_widget(self.gl_viewport.qt_widget)

  self.current_sweep = 1
  self.current_camera = 1
  self:update_photo_pass()

  self:calculate_viewport_size(self.scan.sweeps[self.current_sweep].cameras[self.current_camera].image_data)
  local fov_y = self.scan.sweeps[self.current_sweep].cameras[self.current_camera].lens.vfov
  log.trace("Field of view y=", fov_y) 
  self.model_object = self.gl_viewport.renderer:add_object(self.scan.model_data)

  self.gl_viewport.renderer:create_camera('pose_camera', self.viewport_width, self.viewport_height, fov_y)
  self.gl_viewport.renderer:activate_camera('pose_camera')

  self.gl_viewport.renderer:create_scene('wireframe_scene')
  self.gl_viewport.renderer:activate_scene('wireframe_scene')
  self.gl_viewport.renderer:create_camera('wireframe_camera', self.viewport_width, self.viewport_height, fov_y)
  self.gl_viewport.renderer:activate_camera('wireframe_camera')

  local billboard_data = require('util.obj2').new('../ui2/objs/planeNormalized.obj')
  local billboard_object = self.gl_viewport.renderer:add_object(billboard_data)
  local wireframe_mat_data = {name='wireframe_mat', ambient={0,0,0,1}, diffuse={0,0,0,1}, specular={0,0,0,1}, shininess={0,0,0,1}, emission={0,0,0,1}}
  local wireframe_mat = self.gl_viewport.renderer:create_material(wireframe_mat_data, self.gl_viewport.renderer.shaders.wireframe, {'pose_camera_frame_buffer_pass_picking'})
  billboard_object.mesh:override_materials(wireframe_mat)

  self.vertex_highlight_mat = self.gl_viewport.renderer:create_material(wireframe_mat_data, self.gl_viewport.renderer.shaders.vertex_highlight, {'pose_camera_frame_buffer_pass_depth'})

  self.gl_viewport.renderer:create_camera('vertex_highlight_camera', self.viewport_width, self.viewport_height, fov_y)

  self:update_viewport_passes()
end

function PoseRefinementUi:previous_camera()
  if self.current_camera > 1 then
    self.scan.sweeps[self.current_sweep].cameras[self.current_camera]:flush_image()
    self.current_camera = self.current_camera - 1 
    self:update_photo_pass()
    self:update_viewport_passes()
  elseif self.current_sweep > 1 then
    self.scan.sweeps[self.current_sweep].cameras[self.current_camera]:flush_image()
    self.current_sweep = self.current_sweep - 1
    self.current_camera = #self.scan.sweeps[self.current_sweep].cameras
    self:update_photo_pass()
    self:update_viewport_passes()
  end
end

function PoseRefinementUi:next_camera()
  if #self.scan.sweeps[self.current_sweep].cameras >= (self.current_camera+1) then
    self.scan.sweeps[self.current_sweep].cameras[self.current_camera]:flush_image()
    self.current_camera = self.current_camera + 1 
    self:update_photo_pass()
    self:update_viewport_passes()
  elseif #self.scan.sweeps > self.current_sweep then
    self.scan.sweeps[self.current_sweep].cameras[self.current_camera]:flush_image()
    self.current_sweep = self.current_sweep + 1
    self.current_camera = 1
    self:update_photo_pass()
    self:update_viewport_passes()
  end
end

function PoseRefinementUi:update_photo_pass()
  collectgarbage()
  if self.scan.sweeps[self.current_sweep].cameras[self.current_camera]:image_loaded() == false then
    self.scan.sweeps[self.current_sweep].cameras[self.current_camera]:load_image()
  end

  local photo_qt = qt.QImage.fromTensor(self.scan.sweeps[self.current_sweep].cameras[self.current_camera].image_data)
  self:attach_image(1, 'dslr_photo', 'SourceOver', photo_qt)
  self.widget:update()
end

function PoseRefinementUi:update_viewport_passes()
  --Set GL camera to match current SweepCamera
  local camera_position = nil
  local camera_rotation = nil

  camera_position, camera_rotation = self.scan.sweeps[self.current_sweep]:calculate_camera_world(self.current_camera)

  local forward_vector = torch.Tensor({0,1,0})
  local look_direction = geom.rotate_by_quat(forward_vector, camera_rotation)
  local camera_eye = camera_position
  local camera_center = camera_eye + look_direction

  log.trace("camera_eye=", camera_eye)
  log.trace("camera_center=", camera_center)
  log.trace("look_direction=", look_direction)
  log.trace("look_direction magnitude=", geom.dist(look_direction, torch.Tensor(3):fill(0)))

  self.gl_viewport.renderer.cameras.pose_camera.eye:copy(camera_eye)
  self.gl_viewport.renderer.cameras.pose_camera.center:copy(camera_center)
  self.gl_viewport.renderer.cameras.vertex_highlight_camera.eye:copy(camera_eye)
  self.gl_viewport.renderer.cameras.vertex_highlight_camera.center:copy(camera_center)

  --Standard Render of Model
  self.model_object.mesh:restore_materials()
  self.gl_viewport.renderer:activate_scene('viewport_scene')
  self.gl_viewport.renderer:activate_camera('pose_camera')
  self.gl_viewport.renderer:render()

  --Wireframe Render of Model
  self.gl_viewport.renderer:activate_scene('wireframe_scene')
  self.gl_viewport.renderer:activate_camera('wireframe_camera')
  self.gl_viewport.renderer:render()

  --Vertex Highlighting Render of Model
  self.gl_viewport.renderer:activate_camera('vertex_highlight_camera')
  self.gl_viewport.renderer:activate_scene('viewport_scene')
  self.model_object.mesh:override_materials(self.vertex_highlight_mat)
  self.gl_viewport.renderer:render()

  --Convert Passes to QImages
  local wireframe_image_tensor = self.gl_viewport.renderer.cameras.wireframe_camera.frame_buffer:get_color_image()
  local wireframe_image_qt = qt.QImage.fromTensor(wireframe_image_tensor)

  local vertex_highlight_image_tensor = self.gl_viewport.renderer.cameras.vertex_highlight_camera.frame_buffer:get_color_image()
  local vertex_highlight_image_qt = qt.QImage.fromTensor(vertex_highlight_image_tensor)

  self:attach_image(2, 'wireframe', 'SourceOver', wireframe_image_qt)
  self:attach_image(3, 'vertices', 'SourceOver', vertex_highlight_image_qt)

  self.widget:update()
end

function PoseRefinementUi:calculate_viewport_size(image)
  -- TODO: get ui's window size
  p("self.widget")
  p(self.widget)
  p("self.widget.calibration_view")
  p(self.widget.calibration_view)
  local window_width = self.widget.calibration_view.width
  local window_height = self.widget.calibration_view.height
  local window_aspect = window_width / window_height

  local image_width   = image:size()[3]
  local image_height  = image:size()[2]
  local image_aspect = image_width / image_height

  local viewport_width = nil
  local viewport_height = nil

  if image_aspect > window_aspect then 
    viewport_height = image_height * (window_width/image_width)
    viewport_width = window_width
  else
    viewport_width = image_width * (window_height/image_height)
    viewport_height = window_height
  end

  self.viewport_width = math.floor(viewport_width)
  self.viewport_height = math.floor(viewport_height)
end

function PoseRefinementUi:update_magnifier(magnification, layer, x, y, width, height)
  if self.magnifier == nil then
    self.magnifier = {}
  end

  self.magnifier.layer = layer
  self.magnifier.width = width
  self.magnifier.height = height
  self.magnifier.start_x = x - (width*0.5)
  self.magnifier.start_y = y - (height*0.5)
  self.magnifier.source_start_x = ((x/self.painter.width) * self.layers[layer].image:rect():totable().width) - ((width*0.5)/magnification)
  self.magnifier.source_start_y = ((y/self.painter.height) * self.layers[layer].image:rect():totable().height) - ((height*0.5)/magnification)
  self.magnifier.source_width = width / magnification
  self.magnifier.source_height = height / magnification
end

function PoseRefinementUi:attach_image(layer_number, name, blend_mode, q_image)
  local layer_data = {}
  layer_data.name = name
  layer_data.blend_mode = blend_mode
  layer_data.image = q_image

  self.layers[layer_number] = layer_data
end

function PoseRefinementUi:print_layer_info()
  for layer_number,layer_data in ipairs(self.layers) do
    log.trace("Layer", layer_number, "=", "name:", layer_data.name, "blend mode:", layer_data.blend_mode, "image:", layer_data.image) 
  end
end

function PoseRefinementUi:draw_image_layers()
  for layer_number, layer_data in ipairs(self.layers) do
    self.painter:initmatrix()

    --Fit image inside painter
    local painter_width = self.painter.width
    local painter_height = self.painter.height
    local painter_aspect = painter_width / painter_height

    local layer_width = layer_data.image:rect():totable().width
    local layer_height = layer_data.image:rect():totable().height
    local layer_aspect = layer_width / layer_height

    local final_width = nil
    local final_height = nil

    if layer_aspect > painter_aspect then 
      final_height = math.floor(layer_height * (painter_width/layer_width))
      final_width = painter_width
    else
      final_width = math.floor(layer_width * (painter_height/layer_height))
      final_height = painter_height
    end

    local scale_x = final_width / layer_width  
    local scale_y = final_height / layer_height

    self.painter:scale(scale_x, scale_y)

    self.painter:currentmode(layer_data.blend_mode)
    self.painter:image(0, 0, layer_data.image)
  end
end

function PoseRefinementUi:draw_magnifier()
  self.painter:initmatrix()
  self.painter:currentmode('SourceOver')
  
  self.painter:newpath()
  local magnifier_border = 2
  self.painter:rectangle( self.magnifier.start_x - magnifier_border,
                          self.magnifier.start_y - magnifier_border,
                          self.magnifier.width + (magnifier_border*2),
                          self.magnifier.height + (magnifier_border*2))
  self.painter:setcolor(1.0,1.0,1.0,0.6)
  self.painter:fill()

  self.painter:image( self.magnifier.start_x, 
                      self.magnifier.start_y, 
                      self.magnifier.width, 
                      self.magnifier.height, 
                      self.layers[self.magnifier.layer].image, 
                      self.magnifier.source_start_x,
                      self.magnifier.source_start_y,
                      self.magnifier.source_width,
                      self.magnifier.source_height)

  self.painter:newpath()
  self.painter:moveto(self.magnifier.start_x + (self.magnifier.width*0.5), self.magnifier.start_y)
  self.painter:rlineto( 0, self.magnifier.height)
  self.painter:closepath()
  self.painter:setcolor(1.0,1.0,1.0,0.2)
  self.painter:setlinewidth(1)
  self.painter:stroke(false)

  self.painter:newpath()
  self.painter:moveto(self.magnifier.start_x, self.magnifier.start_y + (self.magnifier.height*0.5))
  self.painter:rlineto( self.magnifier.width, 0)
  self.painter:closepath()
  self.painter:setcolor(1.0,1.0,1.0,0.2)
  self.painter:setlinewidth(1)
  self.painter:stroke(false)
end

function PoseRefinementUi:paint() 
  self.painter:gbegin()

  self:draw_image_layers()

  if self.magnifier ~= nil then self:draw_magnifier() end

  self.painter:gend()
end

return PoseRefinementUi