local gl = require 'ui2.gl'
local libui = require 'libui2'

local Material = torch.class('Material')

local function format_image(img)
  if not img or img:size()[1] ~= 3 then return img end
  
  -- img is color,y,x
  img = img:transpose(1,3)
  img = img:transpose(1,2)
  -- now it's y,x,color which is what gl needs

  return img:contiguous()
end

function Material:__init(mtl_data)
  
  self.name = mtl_data.name
  self.ambient = mtl_data.ambient
  self.diffuse = mtl_data.diffuse
  self.specular = mtl_data.specular
  self.shininess = mtl_data.shininess
  self.emission = {0,0,0,1} --mtl_data.emission

  self.diffuse_tex_img = format_image(mtl_data.diffuse_tex_img)
  self.texture_loaded = not self.diffuse_tex_img -- texture_loaded = false is we have a texture to load

  self.shader = require('ui2.Shader').shaders.textured
  self.tex_id = nil
end

function Material:load_texture()
  local img_data_ptr, img_data_size = libui.byte_storage_info(self.diffuse_tex_img)
  local width = self.diffuse_tex_img:size()[2]
  local height = self.diffuse_tex_img:size()[1]

  self.tex_id = gl.GenTexture()
  gl.check_errors()

  gl.BindTexture(gl.TEXTURE_2D, self.tex_id)
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

  gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
  gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
  gl.check_errors()

  gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_BORDER)
  gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_BORDER)
  gl.check_errors()

  self.texture_loaded = true
end

function Material:use(context)
  self.shader:use(context)
  self.shader:set_uniform_float('sFrontMaterial.ambient', self.ambient)
  self.shader:set_uniform_float('sFrontMaterial.diffuse', self.diffuse)
  self.shader:set_uniform_float('sFrontMaterial.specular', self.specular)
  self.shader:set_uniform_float('sFrontMaterial.shininess', self.shininess)
  self.shader:set_uniform_float('sFrontMaterial.emission', self.emission)

  self:set_texture()
end

function Material:set_texture()
  if not self.texture_loaded then self:load_texture() end
  if not self.tex_id then return end

  gl.ActiveTexture(gl.TEXTURE0)
  gl.BindTexture(gl.TEXTURE_2D, self.tex_id)
  gl.check_errors()

  self.shader:set_uniform_int('textureUnit', 0)
end

return Material

