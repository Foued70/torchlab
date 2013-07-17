local gl = require './gl'
local glfw = require './glfw'

local ImgWidget = Class(ui.GLWidget)

local WIN_MAX_W = 1500
local WIN_MAX_H = 1200

function ImgWidget:__init()
  ui.GLWidget.__init(self, 1, 1, 'Image Viewer')
  self.renderer = ui.Renderer.new(self)

  local viewport_camera = self.renderer:create_camera('viewport_camera', ui.Camera)
  viewport_camera.vfov = 0
  self.renderer:activate_camera('viewport_camera')

  self:setup_geom()
end

function ImgWidget:setup_geom()
  local verts = torch.Tensor({
    {0,0,0,1,  0,0,  0,0,0},
    {1,0,0,1,  1,0,  0,0,0},
    {0,1,0,1,  0,1,  0,0,0},
    {1,1,0,1,  1,1,  0,0,0}
  })

  local faces = torch.IntTensor({ {1,2,3}, {4,3,2} })

  local material = self.renderer:create_material(
    {diffuse_tex_path = 'ImgWidget_image'},
    ui.Shader.shaders.textured,
    {}
  )

  local submeshes = {{start=1, length=2, material=material}}

  self.object = self.renderer:add_object()
  self.object.mesh = ui.Mesh.new(verts, faces, submeshes)
end

function ImgWidget:display(image)
  local typename = torch.typename(image)
  if typename == 'torch.FloatTensor' or typename == 'torch.DoubleTensor' then
    local max = image:max()
    if max < 1 then max = 1 end
    image = image * (255/max)
  end

  if typename ~= 'torch.ByteTensor' then
    image = image:byte()
  end

  self.img_width  = image:size()[3]
  self.img_height = image:size()[2]

  local w_mult = self.img_width / WIN_MAX_W
  local h_mult = self.img_height / WIN_MAX_H
  self.zoom = math.max(w_mult, h_mult)
  if self.zoom < 1 then self.zoom = 1 end
  self.default_zoom = self.zoom
  
  -- focus in pixel coords
  self.focus_x = self.img_width / 2
  self.focus_y = self.img_height / 2

  self:set_scale_and_position()

  self.renderer.texture_manager:delete_texture('ImgWidget_image')
  self.renderer.texture_manager:create_from_image('ImgWidget_image', image)

  -- set initial window size
  local width  = math.ceil(self.img_width / self.zoom)
  local height = math.ceil(self.img_height / self.zoom)
  self:resize(width, height)

  self:update()
  glfw.SetWindowPos(self.window, 0, 0)
end


function ImgWidget:set_scale_and_position()
  local cam = self.renderer.cameras.viewport_camera
  cam:set_eye(self.focus_x/self.zoom, self.focus_y/self.zoom, 1)

  self.object.scale = torch.Tensor({self.img_width / self.zoom, self.img_height / self.zoom, 1})
end


function ImgWidget:mouse_drag(button, x, y)
  local dx = self.drag_last_x - x
  local dy = -(self.drag_last_y - y)

  self.focus_x = self.focus_x + self.zoom * dx
  self.focus_y = self.focus_y + self.zoom * dy

  self:set_scale_and_position()
  self:update()
end

function ImgWidget:img_coords_from_click(x, y)
  --convert to gl coords
  y = self.window_height - y

  --convert to img coords
  local center_x = self.zoom * self.window_width/2
  local center_y = self.zoom * self.window_height/2
  local img_x = self.focus_x - (center_x - self.zoom * x)
  local img_y = self.focus_y - (center_y - self.zoom * y)

  return img_x, img_y
end

function ImgWidget:mouse_click(button, mods, x, y)
  local img_x, img_y = self:img_coords_from_click(x, y)

  if self.zoom > 1 then
    -- focus in pixel coords
    self.focus_x = img_x
    self.focus_y = img_y

    -- zoom in to 100%
    self.zoom = 1
  elseif self.zoom == 1 then
    -- center
    self.focus_x = self.img_width / 2
    self.focus_y = self.img_height / 2

    -- zoom out
    self.zoom = self.default_zoom
  end

    self:set_scale_and_position()
  self:update()
end
