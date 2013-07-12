local gl = require './gl'

local Material = Class()

local function format_image(img)
  if not img or img:size()[1] ~= 3 then return img end
  
  -- img is color,y,x
  img = img:transpose(1,3)
  img = img:transpose(1,2)
  -- now it's y,x,color which is what gl needs

  return img:contiguous()
end

function Material:__init(widget, mtl_data)
  self.widget = widget
  self.name = mtl_data.name
  self.ambient = mtl_data.ambient
  self.diffuse = mtl_data.diffuse
  self.specular = mtl_data.specular
  self.shininess = mtl_data.shininess
  self.emission = {0,0,0,1} --mtl_data.emission

  self.shader = ui.Shader.shaders.textured
  self.textures = {}

  if mtl_data.diffuse_tex_path then
    self:attach_texture(mtl_data.diffuse_tex_path, 0)
  end
end

function Material:set_shader(shader)
  self.shader = shader
end

function Material:attach_texture(texture_name, texture_slot)
  self.textures[texture_slot] = texture_name
end

function Material:use(context)
  self.shader:use(context)
  self.shader:set_uniform_float('sFrontMaterial.ambient', self.ambient or 0)
  self.shader:set_uniform_float('sFrontMaterial.diffuse', self.diffuse or 0)
  self.shader:set_uniform_float('sFrontMaterial.specular', self.specular or 0)
  self.shader:set_uniform_float('sFrontMaterial.shininess', self.shininess or 0)
  self.shader:set_uniform_float('sFrontMaterial.emission', self.emission or 0)

  self:set_textures()
end

function Material:set_textures()
  for slot,name in pairs(self.textures) do
    gl.ActiveTexture(gl.TEXTURE0 + slot)
    if self.widget.texture_manager.textures[name] == nil then
      if name:sub(name:len()-3, name:len()-3) == '.' then -- name is a path as it ends in a file extension
        self.widget.texture_manager:create_from_image_path(name)
      else
        log.trace('Texture '..name..' does not exist! Cannot bind nil texture to slot '..slot)
        return false
      end
    end
    gl.BindTexture(gl.TEXTURE_2D, self.widget.texture_manager.textures[name])
    gl.check_errors()

    self.shader:set_uniform_int('textureUnit'..slot, slot)
  end
  return true
end

