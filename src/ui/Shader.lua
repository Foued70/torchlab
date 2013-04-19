local gl = require 'ui.gl'
local ffi = require 'ffi'
local libui = require 'libui'

local Shader = Class()

local function read(filename)
  io.input(CLOUDLAB_SRC..'/ui/shaders/'..filename)
  return io.read("*all")
end

local function file_exists(filename)
  local file = io.open(CLOUDLAB_SRC..'/ui/shaders/'..filename)
  if file ~= nil then
    io.close(file)
    return true
  else
    return false
  end
end

local function setShaderSource(shader_id, source)
  local source_ptr = gl.char(#source + 1)
  ffi.copy(source_ptr, source)
  local source_pp = gl.const_char_pp(source_ptr)
  gl.ShaderSource(shader_id, 1, source_pp, NULL)
  gl.check_errors()

  gl.CompileShader(shader_id)
  gl.check_errors()

  local result = gl.GetShaderiv(shader_id, gl.COMPILE_STATUS)
  gl.check_errors()

  if result == 0 then log.error('Error compiling shader: '..gl.GetShaderInfoLog(shader_id)) end
end


Shader.shaders = {}

function Shader:__init(name)
  self.name = name
  self.vert_shader_id = gl.CreateShader(gl.VERTEX_SHADER)
  gl.check_errors()

  self.use_geom_shader = file_exists(self.name..'.geom')

  if self.use_geom_shader then
    self.geom_shader_id = gl.CreateShader(gl.GEOMETRY_SHADER)
    gl.check_errors()
  end

  self.frag_shader_id = gl.CreateShader(gl.FRAGMENT_SHADER)
  gl.check_errors()
  self.program_id = 0

  self:load()

  Shader.shaders[name] = self
end

-- this doesn't work in Lua 5.1, but should in 5.2
function Shader:__gc()
  log.trace('Destroying shader: ', self.name)

  gl.DetachShader(self.program_id, self.vert_shader_id)

  if self.use_geom_shader then
    gl.DetachShader(self.program_id, self.geom_shader_id)
  end

  gl.DetachShader(self.program_id, self.frag_shader_id)

  gl.DeleteProgram(self.program_id)

  gl.DeleteShader(self.vert_shader_id)
  if self.type == SHADER_TYPE.GEOMETRY then
    gl.DeleteShader(self.geom_shader_id)
  end
  gl.DeleteShader(self.frag_shader_id)
  gl.check_errors()
end

function Shader:load()
  local header_common = read('_header_common.shader')
  local header_vert = read('_header_vert.shader')
  local header_frag = read('_header_frag.shader')

  local vert_code = nil
  local frag_code = nil

  if self.use_geom_shader then 
  -- Can't use the header files in a geometry shader. vertex out params, cant go directly to frag in. geom must pass vars through
    vert_code = header_common..read(self.name..'.vert')
    frag_code = header_common..read(self.name..'.frag')
  else
    vert_code = header_common..header_vert..read(self.name..'.vert')
    frag_code = header_common..header_frag..read(self.name..'.frag')
  end
  
  setShaderSource(self.vert_shader_id, vert_code)
  setShaderSource(self.frag_shader_id, frag_code)

  self.program_id = gl.CreateProgram()
  gl.check_errors()
  if self.program_id == 0 then log.error('Error creating shader program!') end

  gl.AttachShader(self.program_id, self.vert_shader_id)
  gl.check_errors()
  gl.AttachShader(self.program_id, self.frag_shader_id)
  gl.check_errors()

  if self.use_geom_shader then
    local header_geom = read('_header_geom.shader')
    local geom_code = header_common..header_geom..read(self.name..'.geom')

    setShaderSource(self.geom_shader_id, geom_code)
    gl.check_errors()

    gl.AttachShader(self.program_id, self.geom_shader_id)
    gl.check_errors()
  end

  gl.BindAttribLocation(self.program_id, 0, 'sVertex')
  gl.check_errors()
  gl.BindAttribLocation(self.program_id, 1, 'sTexCoords')
  gl.check_errors()
  gl.BindAttribLocation(self.program_id, 2, 'sNormal')
  gl.check_errors()

  gl.BindFragDataLocation(self.program_id, 0, 'sFragColor')
  gl.check_errors()
  gl.BindFragDataLocation(self.program_id, 1, 'sPickingData')
  gl.check_errors()

  gl.LinkProgram(self.program_id)
  gl.check_errors()

  local result = gl.GetProgramiv(self.program_id, gl.LINK_STATUS)
  if result == 0 then log.error('Error linking shader program: '..gl.GetProgramInfoLog(self.program_id)) end

  if not gl.IsProgram(self.program_id) then
    log.error('Error creating shader program:', self.name)
  else
    log.trace('Shader created:', self.name)
  end
end


function Shader:use(context)
  gl.UseProgram(self.program_id)
  gl.check_errors()

  local projection_matrix, model_view_matrix, normal_matrix, model_view_projection_matrix = context:for_gl()

  self:set_uniform_matrix('sProjectionMatrix', projection_matrix)
  self:set_uniform_matrix('sModelViewMatrix', model_view_matrix)
  self:set_uniform_matrix('sNormalMatrix',normal_matrix)
  self:set_uniform_matrix('sModelViewProjectionMatrix', model_view_projection_matrix)

  self:set_uniform_uint('objectID', context.object_id)
  self:set_uniform_uint('submeshStart', context.submesh_start)

  self:set_uniform_uint('screenWidth', context.screen_width)
  self:set_uniform_uint('screenHeight', context.screen_height)
end

function Shader:set_uniform_uint(name, value)
  local loc = gl.GetUniformLocation(self.program_id, name)
  
  if type(value) ~= 'table' then 
    gl.Uniform1ui(loc, value)
  else
    local v1, v2, v3, v4 = unpack(value)
    if     v4 then gl.Uniform4ui(loc, v1, v2, v3, v4)
    elseif v3 then gl.Uniform3ui(loc, v1, v2, v3)
    elseif v2 then gl.Uniform2ui(loc, v1, v2)
    else           gl.Uniform1ui(loc, v1)
    end
  end

  gl.check_errors()
end

function Shader:set_uniform_int(name, value)
  -- log.trace(name, value)
  if type(value) ~= 'table' then value = {value} end
  local loc = gl.GetUniformLocation(self.program_id, name)

  local v1, v2, v3, v4 = unpack(value)
  if     v4 then gl.Uniform4i(loc, v1, v2, v3, v4)
  elseif v3 then gl.Uniform3i(loc, v1, v2, v3)
  elseif v2 then gl.Uniform2i(loc, v1, v2)
  else           gl.Uniform1i(loc, v1)
  end

  gl.check_errors()
end

function Shader:set_uniform_float(name, value)
  -- log.trace(name, value)
  if type(value) ~= 'table' then value = {value} end
  local loc = gl.GetUniformLocation(self.program_id, name)

  local v1, v2, v3, v4 = unpack(value)
  if     v4 then gl.Uniform4f(loc, v1, v2, v3, v4)
  elseif v3 then gl.Uniform3f(loc, v1, v2, v3)
  elseif v2 then gl.Uniform2f(loc, v1, v2)
  else           gl.Uniform1f(loc, v1)
  end

  gl.check_errors()
end

function Shader:set_uniform_matrix(name, matrix)
  local loc = gl.GetUniformLocation(self.program_id, name)

  local matrix_ptr = libui.float_storage_info(matrix)
  if     matrix:size()[1] == 3 then gl.UniformMatrix3fv(loc, 1, gl.FALSE, matrix_ptr)
  elseif matrix:size()[1] == 4 then gl.UniformMatrix4fv(loc, 1, gl.FALSE, matrix_ptr)
  end

  gl.check_errors()
end


