local gl = require 'ui2.gl'
local ffi = require 'ffi'

local Shader = torch.class('Shader')

local function read(filename)
  io.input(CLOUDLAB_SRC..'/ui2/shaders/'..filename)
  return io.read("*all")
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



function Shader:__init(name)
  log.trace('Creating shader: ', name)

  self.name = name

  self.vert_shader_id = gl.CreateShader(gl.VERTEX_SHADER)
  self.frag_shader_id = gl.CreateShader(gl.FRAGMENT_SHADER)
  gl.check_errors()
  self.program_id = 0

  self:load()
end

-- this doesn't work in Lua 5.1, but should in 5.2
function Shader:__gc()
  log.trace('Destroying shader: ', self.name)

  gl.DetachShader(self.program_id, self.vert_shader_id)
  gl.DetachShader(self.program_id, self.frag_shader_id)

  gl.DeleteProgram(self.program_id)

  gl.DeleteShader(self.vert_shader_id)
  gl.DeleteShader(self.frag_shader_id)
  gl.check_errors()
end

function Shader:load()
  local header_common = read('_header_common.shader')
  local header_vert = read('_header_vert.shader')
  local header_frag = read('_header_frag.shader')

  local vert_code = header_common..header_vert..read(self.name..'.vert')
  local frag_code = header_common..header_frag..read(self.name..'.frag')

  setShaderSource(self.vert_shader_id, vert_code)
  setShaderSource(self.frag_shader_id, frag_code)

  self.program_id = gl.CreateProgram()
  gl.check_errors()
  if self.program_id == 0 then log.error('Error creating shader program!') end

  gl.AttachShader(self.program_id, self.vert_shader_id)
  gl.check_errors()
  gl.AttachShader(self.program_id, self.frag_shader_id)
  gl.check_errors()

  gl.BindFragDataLocation(self.program_id, 0, 'sFragColor')
  gl.BindFragDataLocation(self.program_id, 1, 'sPickingData')

  gl.LinkProgram(self.program_id)
  gl.check_errors()

  local result = gl.GetProgramiv(self.program_id, gl.LINK_STATUS)
  if result == 0 then log.error('Error linking shader program: '..gl.GetProgramInfoLog(self.program_id)) end

  if not gl.IsProgram(self.program_id) then 
    log.error('Error creating shader program!')
  else
    log.trace('Shader compiled. No errors reported.')
  end
end

function Shader:use()
  gl.UseProgram(self.program_id)
  gl.check_errors()
end


return Shader

