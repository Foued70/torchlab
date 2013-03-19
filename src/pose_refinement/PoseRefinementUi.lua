require 'torch'
require 'image'
require 'paths'
require 'qtuiloader'
require 'qtwidget'
require 'qt'

local PoseRefinementUi = torch.class('PoseRefinementUi')

function PoseRefinementUi:__init(description_file)
  self.description = paths.thisfile(description_file)
  log.trace(self.description)
  p(self.description)

  self.widget = qtuiloader.load(self.description)
  self.painter = qt.QtLuaPainter(self.widget.frame)

  self.images = {}
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
    end )

  qt.connect(self.listener, 'sigMouseMove(int,int,QByteArray,QByteArray)',
    function(x, y, keyboard_modifier, keys)
      if self.mouse_left_down == true then
        self:update_magnifier(3.0, 1, x-6, y-6, 200, 200)
      end
    end )
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
  self.magnifier.source_start_x = ((x/self.painter.width) * self.images[layer].image:rect():totable().width) - ((width*0.5)/magnification)
  self.magnifier.source_start_y = ((y/self.painter.height) * self.images[layer].image:rect():totable().height) - ((height*0.5)/magnification)
  self.magnifier.source_width = width / magnification
  self.magnifier.source_height = height / magnification
end

function PoseRefinementUi:attach_image(layer_number, name, blend_mode, q_image)
  if self.images[layer_number] ~= nil then
    self.images[layer_number] = nil
  end

  local layer_data = {}
  layer_data.name = name
  layer_data.blend_mode = blend_mode
  layer_data.image = q_image
  table.insert(self.images, layer_number, layer_data)
end

function PoseRefinementUi:print_layer_info()
  for layer_number,layer_data in ipairs(self.images) do
    log.trace("Layer", layer_number, "=", "name:", layer_data.name, "blend mode:", layer_data.blend_mode, "image:", layer_data.image) 
  end
end

function PoseRefinementUi:draw_image_layers()
  for layer_number, layer_data in ipairs(self.images) do
    self.painter:initmatrix()

    --Fit image inside painter
    local width = layer_data.image:rect():totable().width
    local height = layer_data.image:rect():totable().height
    local desired_width = self.painter.width
    local desired_height = self.painter.height
    local scale_x = desired_width / width  
    local scale_y = desired_height / height

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
                      self.images[self.magnifier.layer].image, 
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