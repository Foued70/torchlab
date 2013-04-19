local libui = require 'libui'
local gl = require 'ui.gl'

require 'image' -- fucking image global

local TextureManager = Class()

local function format_image(img)
  if not img or img:size()[1] ~= 3 then return img end
  
  -- img is color,y,x
  img = img:transpose(1,3)
  img = img:transpose(1,2)
  -- now it's y,x,color which is what gl needs

  return img:contiguous()
end

function TextureManager:__init()
  log.trace('TextureManager constructed')
  self.textures = {}
end

function TextureManager:__gc()
  for key,value in pairs(self.textures) do
    self:delete_texture(key)
  end
end

function TextureManager:delete_texture(texture_name)
  if self.textures[texture_name] then
    gl.DeleteTexture(self.textures[texture_name])
    self.textures[texture_name] = nil
  end
end

function TextureManager:create_blank(texture_name, pixel_format_internal, width, height, pixel_format, value_type)
  if self.textures[texture_name] then
    return self.textures[texture_name]
  end

  local id = gl.GenTexture()
  gl.check_errors()

  gl.BindTexture(gl.TEXTURE_2D, id)
  gl.check_errors()

  gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST)
  gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST)
  gl.check_errors()

  gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE)
  gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE)
  gl.check_errors()
  
  gl.TexImage2D(
    gl.TEXTURE_2D, 
    0, -- level
    pixel_format_internal, -- internalFormat
    width, 
    height, 
    0, -- border
    pixel_format, -- format 
    value_type, -- type
    nil
  )
  gl.check_errors()

  gl.BindTexture(gl.TEXTURE_2D, 0)
  gl.check_errors()

  self.textures[texture_name] = id;
  return self.textures[texture_name]
end

function TextureManager:create_from_image(image_path)
  if self.textures[image_path] then
    return self.textures[image_path]
  end

  local tex_img = image.load(image_path, nil, 'byte')
  tex_img = format_image(tex_img)
  local img_data_ptr, img_data_size = libui.byte_storage_info(tex_img)
  local width = tex_img:size()[2]
  local height = tex_img:size()[1]

  local id = gl.GenTexture()
  gl.check_errors()

  gl.BindTexture(gl.TEXTURE_2D, id)
  gl.check_errors()

  gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
  gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
  gl.check_errors()

  gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_BORDER)
  gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_BORDER)
  gl.check_errors()

  gl.TexImage2D(
    gl.TEXTURE_2D, 
    0, -- level
    gl.RGB, -- internalFormat
    width, height, 
    0, -- border
    gl.RGB, -- format 
    gl.UNSIGNED_BYTE, -- type
    img_data_ptr
  )
  gl.check_errors()

  gl.BindTexture(gl.TEXTURE_2D, 0)
  gl.check_errors()

  self.textures[image_path] = id;
  return self.textures[image_path]
end

