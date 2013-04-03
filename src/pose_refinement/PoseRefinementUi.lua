require 'torch'
require 'image'
require 'paths'
require 'qtuiloader'
require 'qtwidget'
require 'qt'

local geom = require 'util.geom'
local libui = require 'libui2'
local config = require 'ui2.pp_config'

local PoseRefinementUi = Class()

function PoseRefinementUi:__init()
  self.gl_viewport = nil
  self.viewport_size = torch.Tensor(2)
  self.viewport_vfov = nil

  self.highlighted_vertex = nil

  self.scan_folder = nil
  self.pose_file = nil 
  self.scan = nil
  self.current_sweep = nil
  self.current_photo = nil

  self.description = paths.thisfile('pose_refinement.ui')

  self.widget = qtuiloader.load(self.description)
  self.painter = qt.QtLuaPainter(self.widget.viewport_container.viewport)

  self.layers = {}
  self.magnifier = nil

  self.widget.mouseTracking = false
  self.mouse_left_down = false
  self.mouse_left_start = torch.Tensor(2)
  self.mouse_left_end = torch.Tensor(2)
  self.mouse_right_down = false
  
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
      if self.gl_viewport ~= nil then self:paint() end
    end )

  
  qt.connect(self.listener, 'sigMousePress(int,int,QByteArray,QByteArray,QByteArray)',
    function(x, y, mouse_button, keyboard_modifier, keys)
      self.widget.mouseTracking = true
      if mouse_button == 'LeftButton' then
        if self.gl_viewport ~= nil then
          self:mouse_to_viewport_screenspace(x, y, self.mouse_left_start)
          self.mouse_left_end:copy(self.mouse_left_start)
        end
        self.mouse_left_down = true
      elseif mouse_button == 'RightButton' then
        self.mouse_right_down = true
        self.widget.cursor = qt.QCursor.new("BlankCursor")

        local cursor_pos = qt.QPoint.totable(self.widget.cursor:pos())
        self.mouse_start_global = torch.Tensor({cursor_pos.x, cursor_pos.y})
        self.mouse_start_local = self:window_to_viewport_pixels(x,y)
        self.mouse_slowed_local = torch.Tensor(2):copy(self.mouse_start_local)
      end
    end )

  qt.connect(self.listener, 'sigMouseRelease(int,int,QByteArray,QByteArray,QByteArray)',
    function(x, y, mouse_button, keyboard_modifier, keys)
      self.widget.mouseTracking = false
      if mouse_button == 'LeftButton' then
        if self.gl_viewport ~= nil then
          self:mouse_to_viewport_screenspace(x, y, self.mouse_left_end)
          self:drag_select_vertex()
        end
        self.mouse_left_down = false
      elseif mouse_button == 'RightButton' then
        if self.gl_viewport ~= nil then
          local mouse_position = torch.Tensor(2)
          mouse_position[1] = self.widget.main_frame.x + self.widget.main_frame.viewport_container.x + self.widget.main_frame.viewport_container.viewport.x + self.mouse_slowed_local[1]
          mouse_position[2] = self.widget.main_frame.y + self.widget.main_frame.viewport_container.y + self.widget.main_frame.viewport_container.viewport.y + self.mouse_slowed_local[2]
          self:select_image_coordinate(mouse_position[1], mouse_position[2])

          --Oddly, mouse position must be set gobally, in pixel coordinates of the monitor, not the application. 
          --Because OS window decoration cannot be predicted, the offset is being used to calculate the end.
          local mouse_move_delta = self.mouse_slowed_local - self.mouse_start_local
          local mouse_end_global = self.mouse_start_global + mouse_move_delta
          self.widget.cursor.setPos(qt.QPoint({x=mouse_end_global[1], y=mouse_end_global[2]}))
          
          self.widget.cursor = qt.QCursor.new("CrossCursor")
          self.mouse_start_global = nil
          self.mouse_start_local = nil
          self.mouse_slowed_local = nil
        end
        self.highlighted_vertex = nil
        self.mouse_right_down = false
        self.magnifier = nil
        self.widget:update()
      end
    end )

  qt.connect(self.listener, 'sigMouseMove(int,int,QByteArray,QByteArray)',
    function(x, y, keyboard_modifier, keys)
      if self.mouse_left_down == true then
        self:mouse_to_viewport_screenspace(x, y, self.mouse_left_end)
      elseif self.mouse_right_down == true then
        local mouse_current = self:window_to_viewport_pixels(x,y)
        local mouse_delta = mouse_current - self.mouse_start_local
        torch.mul(mouse_delta, mouse_delta, 0.01)
        torch.add(self.mouse_slowed_local, self.mouse_slowed_local, mouse_delta)

        self:update_magnifier(2.0, 1, self.mouse_slowed_local[1], self.mouse_slowed_local[2], 200, 200)

        self.widget.cursor.setPos(qt.QPoint({x=self.mouse_start_global[1], y=self.mouse_start_global[2]}))
      end
    end )

  qt.connect(self.widget.button_scan_folder, 'clicked()', function() self:set_scan_folder() end)
  qt.connect(self.widget.button_pose_file, 'clicked()', function() self:set_pose_file() end)
  qt.connect(self.widget.button_init_calibration, 'clicked()', function() self:init_calibration() end)
  qt.connect(self.widget.button_save_calibration, 'clicked()', function() self:save_calibration() end)
  qt.connect(self.widget.button_previous, 'clicked()', function() self:previous_camera() end)
  qt.connect(self.widget.button_next, 'clicked()', function() self:next_camera() end)
  qt.connect(self.widget.check_box_white_wall, 'clicked()', function() self:set_white_wall_status(self.widget.check_box_white_wall.checked) end)
  qt.connect(self.widget.button_delete_calibration_pair, 'clicked()', function() self:delete_last_calibration_pair() end)
end

function PoseRefinementUi:delete_last_calibration_pair()
  self.scan.sweeps[self.current_sweep].photos[self.current_photo]:delete_calibration_pair(self.scan.sweeps[self.current_sweep].photos[self.current_photo].pairs_calibrated)
  self:update_ui_info()
end

function PoseRefinementUi:set_white_wall_status(status)
  self.scan.sweeps[self.current_sweep].photos[self.current_photo].white_wall = status
  self:update_ui_info()
end

function PoseRefinementUi:previous_camera()
  self.highlighted_vertex = nil
  if self.current_photo > 1 then
    self.scan.sweeps[self.current_sweep].photos[self.current_photo]:flush_image()
    self.current_photo = self.current_photo - 1 
    self:update_photo_pass()
    self:update_viewport_passes()
    self:update_ui_info()
  elseif self.current_sweep > 1 then
    self.scan.sweeps[self.current_sweep].photos[self.current_photo]:flush_image()
    self.current_sweep = self.current_sweep - 1
    self.current_photo = #self.scan.sweeps[self.current_sweep].photos
    self:update_photo_pass()
    self:update_viewport_passes()
    self:update_ui_info()
  end
end

function PoseRefinementUi:next_camera()
  self.highlighted_vertex = nil
  if #self.scan.sweeps[self.current_sweep].photos >= (self.current_photo+1) then
    self.scan.sweeps[self.current_sweep].photos[self.current_photo]:flush_image()
    self.current_photo = self.current_photo + 1 
    self:update_photo_pass()
    self:update_viewport_passes()
    self:update_ui_info()
  elseif #self.scan.sweeps > self.current_sweep then
    self.scan.sweeps[self.current_sweep].photos[self.current_photo]:flush_image()
    self.current_sweep = self.current_sweep + 1
    self.current_photo = 1
    self:update_photo_pass()
    self:update_viewport_passes()
    self:update_ui_info()
  end
end

function PoseRefinementUi:update_ui_info()
  self.widget.sweep_info:setText(string.format("%d/%d", self.current_sweep, #self.scan.sweeps)) 
  self.widget.photo_info:setText(string.format("%d/%d", self.current_photo, #self.scan.sweeps[self.current_sweep].photos)) 

  
  local pair_matrix = self.scan.sweeps[self.current_sweep].photos[self.current_photo].calibration_pairs

  local pair_matrix_string = "Calibration Pairs:\n"
  for i=1, self.scan.sweeps[self.current_sweep].photos[self.current_photo].pairs_calibrated do
    for j=1, pair_matrix:size(2) do
      pair_matrix_string = pair_matrix_string .. string.format("%2.2f", pair_matrix[i][j]) .. " "
    end
    pair_matrix_string = pair_matrix_string .. "\n"
  end
  self.widget.calibration_pair_info:setText(pair_matrix_string)
end

function PoseRefinementUi:select_vertex(mouse_x, mouse_y)
  local click_coordinates = self:mouse_to_viewport_screenspace(mouse_x, mouse_y)
  if click_coordinates ~= nil then
    self.gl_viewport.renderer:activate_scene('viewport_scene')
    self.gl_viewport.renderer:activate_camera('vertex_highlight_camera')

    local pixels = self.gl_viewport.renderer.active_camera:screen_to_pixel(click_coordinates)
    pixels[1] = math.floor(pixels[1])
    pixels[2] = math.floor(pixels[2])

    local object_id, triangle_index, submesh_start, primitive_id = self.gl_viewport.renderer.cameras.vertex_highlight_camera.frame_buffer:read_pick_pixel(pixels[1], pixels[2])
    triangle_index = math.floor(primitive_id/3) + submesh_start
    vertex_index = (primitive_id%3) + 1

    if object_id < 1 then
      self.highlighted_vertex = nil
      return nil 
    end
    local object = self.gl_viewport.renderer.scenes.viewport_scene[object_id]
    local verts, center, normal = object:get_triangle(triangle_index)
    local vertex_positions = verts:narrow(2,1,3)

    self.highlighted_vertex = vertex_positions[vertex_index]
    self.scan.sweeps[self.current_sweep].photos[self.current_photo]:add_vertex(self.highlighted_vertex)
    self:update_viewport_passes()
  end
end

function PoseRefinementUi:drag_select_vertex()
  if (self.mouse_left_start == nil) or (self.mouse_left_end == nil) then return end

  self.gl_viewport.renderer:activate_scene('viewport_scene')
  self.gl_viewport.renderer:activate_camera('vertex_highlight_camera')

  local selection_top_left = torch.Tensor(2)
  local selection_bottom_right = torch.Tensor(2)

  if (self.mouse_left_start[1] > self.mouse_left_end[1]) then
    selection_top_left[1] = self.mouse_left_end[1]
    selection_bottom_right[1] = self.mouse_left_start[1]
  else
    selection_top_left[1] = self.mouse_left_start[1]
    selection_bottom_right[1] = self.mouse_left_end[1]
  end

  if (self.mouse_left_start[2] > self.mouse_left_end[2]) then
    selection_bottom_right[2] = self.mouse_left_end[2]
    selection_top_left[2] = self.mouse_left_start[2]
  else
    selection_bottom_right[2] = self.mouse_left_start[2]
    selection_top_left[2] = self.mouse_left_end[2]
  end

  local selected_vertices = self.gl_viewport.renderer:pick_vertices(selection_top_left, selection_bottom_right)
  if selected_vertices ~= nil then
    self.highlighted_vertex = selected_vertices[1]
    local highlighted_vertex_distance = self.gl_viewport.renderer.active_camera:world_to_screen(self.highlighted_vertex)[3]
    --Find vertex closest to camera
    log.trace("Selected",#selected_vertices, "vertices")
    if #selected_vertices > 1 then
      for v = 2, #selected_vertices do
        local current_vertex_distance = self.gl_viewport.renderer.active_camera:world_to_screen(selected_vertices[v])[3]
        if highlighted_vertex_distance > current_vertex_distance then
          log.trace("Vertex", v, ":", selected_vertices[v], "is closer to camera at:", current_vertex_distance, "than", self.highlighted_vertex, "at:", highlighted_vertex_distance)
          self.highlighted_vertex = selected_vertices[v]
          highlighted_vertex_distance = current_vertex_distance
        end
      end
    end
    self.scan.sweeps[self.current_sweep].photos[self.current_photo]:add_vertex(self.highlighted_vertex)
  else
    self.highlighted_vertex = nil
  end
  self:update_viewport_passes()
  self:update_ui_info()
  return selected_vertices
end

function PoseRefinementUi:select_image_coordinate(mouse_x, mouse_y)
  local image_coordinate = self:mouse_to_viewport_screenspace(mouse_x, mouse_y)
  if image_coordinate ~= nil then
    log.trace("Selected image coordinate:", image_coordinate)
    self.scan.sweeps[self.current_sweep].photos[self.current_photo]:add_image_coordinate(image_coordinate)
    self:update_ui_info()
  end
end

function PoseRefinementUi:window_to_viewport_pixels(x, y, res)
  if res == nil then
    res = torch.Tensor(2)
  end

  res[1] = x - (self.widget.main_frame.x + self.widget.main_frame.viewport_container.x + self.widget.main_frame.viewport_container.viewport.x)
  res[2] = y - (self.widget.main_frame.y + self.widget.main_frame.viewport_container.y + self.widget.main_frame.viewport_container.viewport.y)
  return res
end

function PoseRefinementUi:mouse_to_viewport_screenspace(mouse_x, mouse_y, res)
  if not self.gl_viewport then return nil end

  local viewport_pixels = self:window_to_viewport_pixels(mouse_x, mouse_y)

  if res == nil then
    res = torch.Tensor(2)
  end

  res[1] = ((viewport_pixels[1] / self.viewport_size[1])-0.5)*2
  res[2] = ((1.0-(viewport_pixels[2] / self.viewport_size[2]))-0.5)*2 

  if (res[1] > 1) or (res[1] < -1) or (res[2] > 1) or (res[2] < -1) then
    return nil
  end
  return res
end

function PoseRefinementUi:viewport_screenspace_to_viewport_pixels(screen_position)
  local pixels = screen_position + 1
  torch.div(pixels, pixels, 2)
  pixels[2] = 1.0 - pixels[2] --flip y

  pixels[1] = pixels[1] * self.widget.main_frame.viewport_container.viewport.width
  pixels[2] = pixels[2] * self.widget.main_frame.viewport_container.viewport.height

  return pixels
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

function PoseRefinementUi:create_debug_camera_meshes(sweep_number)
  local debug_model_data = util.Obj.new('../ui2/objs/debug_forward_pointer.obj')

  for i = 1, #self.scan.sweeps[sweep_number].photos do
    local camera_position = nil
    local camera_rotation = nil

    local debug_camera = self.gl_viewport.renderer:add_object(debug_model_data)
    log.trace("Building debug camera number", i)
    camera_position, camera_rotation = self.scan.sweeps[sweep_number]:calculate_camera_world(i)
    log.trace("Camera",i,"position", camera_position)
    log.trace("Camera",i,"rotation", camera_rotation)

    debug_camera.position:copy(camera_position)
    debug_camera.rotation:copy(camera_rotation)
  end
end

function PoseRefinementUi:init_calibration()
  if (self.scan_folder == nil) then
    --log.trace("Failed to start calibration. No scan folder set.")
    --return false
    --For Testing, use a default for immidiate startup
    self.scan_folder = '/Users/NickBrancaccio/cloudlab/src/pose_refinement/96_spring_kitchen'
  end

  if (self.pose_file == nil) then
    --log.trace("Failed to start calibration. No pose file set.")
    --return false
    --For Testing, use a default for immidiate startup
    self.pose_file = '/Users/NickBrancaccio/cloudlab/src/pose_refinement/96_spring_kitchen/scanner371_job286000_texture_info.txt'
  end

  log.trace("Loading scan at", sys.clock())
  self.scan = pose_refinement.Scan.new(self.scan_folder, self.pose_file)
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

  libui.make_current(self.gl_viewport.qt_widget)
  libui.hide_widget(self.gl_viewport.qt_widget)

  self.gl_viewport.renderer:activate_scene('viewport_scene')
  self.model_object = self.gl_viewport.renderer:add_object(self.scan.model_data)

  self.gl_viewport.renderer:create_scene('wireframe_scene')
  self.gl_viewport.renderer:activate_scene('wireframe_scene')

  local billboard_data = util.Obj.new('../ui2/objs/planeNormalized.obj')
  local billboard_object = self.gl_viewport.renderer:add_object(billboard_data)

  local wireframe_mat_data = {name='wireframe_mat', ambient={0,0,0,1}, diffuse={0,0,0,1}, specular={0,0,0,1}, shininess={0,0,0,1}, emission={0,0,0,1}}
  local wireframe_mat = self.gl_viewport.renderer:create_material(wireframe_mat_data, self.gl_viewport.renderer.shaders.wireframe, {'pose_camera_frame_buffer_pass_picking'})
  billboard_object.mesh:override_materials(wireframe_mat)

  self.vertex_highlight_mat = self.gl_viewport.renderer:create_material(wireframe_mat_data, self.gl_viewport.renderer.shaders.vertex_highlight, {'pose_camera_frame_buffer_pass_depth'})

  --Creating cameras with default settings. These setting get overridden during resize_viewport_by_photo() 
  self.gl_viewport.renderer:create_camera('pose_camera')
  self.gl_viewport.renderer:create_camera('wireframe_camera')
  self.gl_viewport.renderer:create_camera('vertex_highlight_camera')

  self.current_sweep = 3
  self.current_photo = 8
  self:update_photo_pass()
  self:update_ui_info()
  self:resize_viewport_by_photo(self.scan.sweeps[self.current_sweep].photos[self.current_photo])
end

function PoseRefinementUi:update_photo_pass()
  collectgarbage()
  if self.scan.sweeps[self.current_sweep].photos[self.current_photo]:image_loaded() == false then
    self.scan.sweeps[self.current_sweep].photos[self.current_photo]:load_image()
  end

  local photo_raw_qt = qt.QImage.fromTensor(self.scan.sweeps[self.current_sweep].photos[self.current_photo].image_data_raw)
  local photo_rectilinear_qt = qt.QImage.fromTensor(self.scan.sweeps[self.current_sweep].photos[self.current_photo].image_data_rectilinear)
  self:attach_image(1, 'dslr_photo_raw', 'Hidden', photo_raw_qt)
  self:attach_image(2, 'dslr_photo_rectilinear', 'SourceOver', photo_rectilinear_qt)
  self.widget:update()
end

function PoseRefinementUi:update_viewport_passes()
  --Set GL camera to match current Photo
  local camera_position = nil
  local camera_rotation = nil

  camera_position, camera_rotation = self.scan.sweeps[self.current_sweep]:calculate_camera_world(self.current_photo)
  --camera_position = torch.Tensor({3.276560, -2.5, 1.78})
  --camera_rotation = torch.Tensor({0.031647, 0.023114, 0.731051, -0.681197})

  local forward_vector = torch.Tensor({0,1,0})
  local look_direction = geom.rotate_by_quat(forward_vector, camera_rotation)
  log.trace(look_direction:norm())

  local camera_eye = camera_position
  local camera_center = camera_eye + look_direction

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
  gl.UseProgram(self.gl_viewport.renderer.shaders.vertex_highlight.program_id)
  if self.highlighted_vertex ~= nil then
    self.gl_viewport.renderer.shaders.vertex_highlight:set_uniform_uint('selectedVertexCount', 1)
    self.gl_viewport.renderer.shaders.vertex_highlight:set_uniform_float('selectedVertexPosition', {self.highlighted_vertex[1], self.highlighted_vertex[2], self.highlighted_vertex[3]})
  else
    self.gl_viewport.renderer.shaders.vertex_highlight:set_uniform_uint('selectedVertexCount', 0)
    self.gl_viewport.renderer.shaders.vertex_highlight:set_uniform_float('selectedVertexPosition', {0,0,0})
  end
  gl.UseProgram(0)
  self.gl_viewport.renderer:activate_camera('vertex_highlight_camera')
  self.gl_viewport.renderer:activate_scene('viewport_scene')
  self.model_object.mesh:override_materials(self.vertex_highlight_mat)
  self.gl_viewport.renderer:render()

  --Convert Passes to QImages
  local wireframe_image_tensor = self.gl_viewport.renderer.cameras.wireframe_camera.frame_buffer:get_color_image()
  local wireframe_image_qt = qt.QImage.fromTensor(wireframe_image_tensor)

  local vertex_highlight_image_tensor = self.gl_viewport.renderer.cameras.vertex_highlight_camera.frame_buffer:get_color_image()
  local vertex_highlight_image_qt = qt.QImage.fromTensor(vertex_highlight_image_tensor)

  self:attach_image(3, 'wireframe', 'SourceOver', wireframe_image_qt)
  self:attach_image(4, 'vertices', 'SourceOver', vertex_highlight_image_qt)

  self.widget:update()
end

function PoseRefinementUi:resize_viewport_by_photo(photo)
  local window_width = self.widget.viewport_container.width
  local window_height = self.widget.viewport_container.height
  local window_aspect = window_width / window_height
  log.trace("window_aspect", window_aspect)

  log.trace("Calibration view size:", window_width, window_height)

  local image_width   = photo.image_data_rectilinear:size()[3]
  local image_height  = photo.image_data_rectilinear:size()[2]
  local image_aspect = image_width / image_height
  log.trace("image_aspect", image_aspect)

  local viewport_width = nil
  local viewport_height = nil

  if image_aspect > window_aspect then 
    viewport_height = image_height * (window_width/image_width)
    viewport_width = window_width
  else
    viewport_width = image_width * (window_height/image_height)
    viewport_height = window_height
  end

  viewport_width = math.floor(viewport_width)
  viewport_height = math.floor(viewport_height)

  self:resize_viewport(viewport_width, viewport_height, photo.lens.rectilinear.vfov)
end

function PoseRefinementUi:resize_viewport(width, height, vfov)
  self.viewport_size[1] = width
  self.viewport_size[2] = height
  self.viewport_vfov = vfov

  self.gl_viewport.renderer.cameras.pose_camera.vfov = self.viewport_vfov
  self.gl_viewport.renderer.cameras.wireframe_camera.vfov = self.viewport_vfov
  self.gl_viewport.renderer.cameras.vertex_highlight_camera.vfov = self.viewport_vfov

  self.gl_viewport.renderer.cameras.pose_camera:resize(self.viewport_size[1], self.viewport_size[2])
  self.gl_viewport.renderer.cameras.wireframe_camera:resize(self.viewport_size[1], self.viewport_size[2])
  self.gl_viewport.renderer.cameras.vertex_highlight_camera:resize(self.viewport_size[1], self.viewport_size[2])

  local viewport_position_x = (self.widget.viewport_container.width - width) / 2
  local viewport_position_y = (self.widget.viewport_container.height - height) / 2
  self.widget.viewport_container.viewport.pos = qt.QPoint.new({x=viewport_position_x, y=viewport_position_y})
  self.widget.viewport_container.viewport.size = qt.QSize.new({width=width, height=height})

  self:update_viewport_passes()
end

--Simple save function to get some calibration data to test with. 
function PoseRefinementUi:save_calibration()
  local save_path = qt.QFileDialog.getSaveFileName(self.widget, "Save As", '', "Lua Files (*.lua)"):tostring()
  local file = io.open(save_path, "w")

  file:write("require \'torch\'\n")

  local scan_table = "local scan = {"

  for sweep = 1, #self.scan.sweeps do
    file:write("local sweep_"..sweep.." = {\n")
    for photo = 1, #self.scan.sweeps[sweep].photos do
      file:write("  {\n")
      file:write("    image_path = \""..self.scan.sweeps[sweep].photos[photo].image_path.."\",\n")

      if self.scan.sweeps[sweep].photos[photo].lens ~= nil then
        file:write("    lens = { lens_type= \""..self.scan.sweeps[sweep].photos[photo].lens.sensor.lens_type.."\", projection= \"nil\"},\n")
      else
        file:write("    lens = { lens_type= \"nil\", projection= \"nil\"},\n")
      end
      file:write("    white_wall = ")
      if self.scan.sweeps[sweep].photos[photo].white_wall then
        file:write("true,\n")
      else
        file:write("false,\n")
      end
      file:write("    pairs_calibrated = "..self.scan.sweeps[sweep].photos[photo].pairs_calibrated..",\n")
      file:write("    calibration_pairs = torch.Tensor({\n")
      for j = 1, self.scan.sweeps[sweep].photos[photo].calibration_pairs:size(1) do
        file:write("      {")
        for k = 1, self.scan.sweeps[sweep].photos[photo].calibration_pairs:size(2) do
          file:write(self.scan.sweeps[sweep].photos[photo].calibration_pairs[j][k])
          if k ~= 5 then file:write(", ") end
        end
        if j ~= 4 then file:write("},\n") else file:write("}\n") end 
      end
      file:write("    }),\n")

      file:write("    pose_position = torch.Tensor({")
      file:write(self.scan.sweeps[sweep].position[1]..", ")
      file:write(self.scan.sweeps[sweep].position[2]..", ")
      file:write(self.scan.sweeps[sweep].position[3].."}),\n")

      file:write("    pose_rotation = torch.Tensor({")
      file:write(self.scan.sweeps[sweep].rotation[1]..", ")
      file:write(self.scan.sweeps[sweep].rotation[2]..", ")
      file:write(self.scan.sweeps[sweep].rotation[3]..", ")
      file:write(self.scan.sweeps[sweep].rotation[4].."}),\n")

      local photo_position, photo_rotation = self.scan.sweeps[sweep]:calculate_camera_world(photo)
      file:write("    photo_position = torch.Tensor({")
      file:write(photo_position[1]..", ")
      file:write(photo_position[2]..", ")
      file:write(photo_position[3].."}),\n")

      file:write("    photo_rotation = torch.Tensor({")
      file:write(photo_rotation[1]..", ")
      file:write(photo_rotation[2]..", ")
      file:write(photo_rotation[3]..", ")
      file:write(photo_rotation[4].."})\n")

      if photo ~= #self.scan.sweeps[sweep].photos then
        file:write("  },\n")
      else
        file:write("  }\n")
      end
    end
    file:write("}\n\n")

    if sweep ~= #self.scan.sweeps then
      scan_table = scan_table.."sweep_"..sweep..", "
    else
      scan_table = scan_table.."sweep_"..sweep
    end
  end

  scan_table = scan_table.."}"
  file:write(scan_table.."\n")
  file:write("return scan\n")
  file:close()
end

function PoseRefinementUi:update_magnifier(magnification, layer, x, y, width, height)
  if self.magnifier == nil then
    self.magnifier = {}
  end
  self.magnifier.layer = layer
  self.magnifier.start_x = x - (width*0.5)
  self.magnifier.start_y = y - (height*0.5)
  self.magnifier.width = width
  self.magnifier.height = height
  self.magnifier.source_start_x = ((x/self.widget.main_frame.viewport_container.viewport.width) * self.layers[layer].image:rect():totable().width) - ((width*0.5)/magnification)
  self.magnifier.source_start_y = ((y/self.widget.main_frame.viewport_container.viewport.height) * self.layers[layer].image:rect():totable().height) - ((height*0.5)/magnification)
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
    if layer_data.blend_mode ~= 'Hidden' then
      self.painter:initmatrix()

      local scale_x = self.widget.main_frame.viewport_container.viewport.width / layer_data.image:rect():totable().width  
      local scale_y = self.widget.main_frame.viewport_container.viewport.height / layer_data.image:rect():totable().height

      self.painter:scale(scale_x, scale_y)

      self.painter:currentmode(layer_data.blend_mode)
      self.painter:image(0, 0, layer_data.image)
    end
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

function PoseRefinementUi:draw_selection_box(start_corner, end_corner)
  self.painter:initmatrix()
  self.painter:currentmode('SourceOver')
  local start_corner_pixels = self:viewport_screenspace_to_viewport_pixels(start_corner)
  local end_corner_pixels = self:viewport_screenspace_to_viewport_pixels(end_corner)

  local start = torch.Tensor(2)
  local size = torch.Tensor(2)

  if start_corner_pixels[1] < end_corner_pixels[1] then
    start[1] = start_corner_pixels[1]
    size[1] = end_corner_pixels[1] - start_corner_pixels[1]
  else
    start[1] = end_corner_pixels[1]
    size[1] = start_corner_pixels[1] - end_corner_pixels[1]
  end

  if start_corner_pixels[2] < end_corner_pixels[2] then
    start[2] = start_corner_pixels[2]
    size[2] = end_corner_pixels[2] - start_corner_pixels[2]
  else
    start[2] = end_corner_pixels[2]
    size[2] = start_corner_pixels[2] - end_corner_pixels[2]
  end

  self.painter:rectangle(start[1], start[2], size[1], size[2])
  self.painter:setcolor(1.0, 0.96, 0.82, 0.8)
  self.painter:fill()
end

function PoseRefinementUi:paint() 
  self.painter:gbegin()

  self:draw_image_layers()

  if self.magnifier ~= nil then self:draw_magnifier() end

  if self.mouse_left_down == true then self:draw_selection_box(self.mouse_left_start, self.mouse_left_end) end

  self.painter:gend()
end

return PoseRefinementUi