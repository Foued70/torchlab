local Snapper = Class()

function Snapper:__init(obj_data)
    self.gl_viewport = ui.GLWidget.new(function(widget)
      widget.renderer:create_camera('viewport_camera', nil, nil, nil, nil, nil, ui.Camera)
    end)

    log.trace()
    self.gl_viewport.renderer:add_object(obj_data)

    log.trace()
    self.camera = self.gl_viewport.renderer.cameras.viewport_camera
    -- self.gl_viewport.renderer:activate_camera('viewport_camera')
    log.trace()
end



function Snapper:config_camera(width, height, vfov)
  self.camera.vfov = vfov
  self.gl_viewport:resize_widget(width, height)
end

function Snapper:snap(position, rotation_quat)
  self.camera.eye:copy(position)
  
  -- right handed coordinate system looking down +y with +z up
  self.camera:set_look_dir(0,1,0)
  self.camera:set_up_dir(0,0,1)
  self.camera:set_right_dir(1,0,0)
  self.camera:rotate(rotation_quat)

  self.gl_viewport:paint()
  return self.camera.frame_buffer:get_color_image()
end