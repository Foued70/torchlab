local gl = require 'gl'
local libui = require 'libui2'

local Mesh = torch.class('Mesh')

function Mesh:__init(verts, face_indexes, submeshes)
  self.verts = verts
  self.face_indexes = face_indexes
  self.submeshes = submeshes
  self.vao_id = gl.uint(1)
  self.vert_buffer_id = gl.uint(1)
  self.element_buffer_id = gl.uint(1)
end


function Mesh:push_to_gl()
  gl.GenVertexArrays(1, self.vao_id)
  gl.check_errors()
  glGenBuffers(1, self.vert_buffer_id)
  gl.check_errors()
  gl.GenBuffers(1, self.element_buffer_id)
  gl.check_errors()

  gl.BindVertexArray(self.vao_id[0])
  gl.check_errors()
  gl.BindBuffer(gl.ELEMENT_ARRAY_BUFFER, self.element_buffer_id[0])
  gl.check_errors()
  gl.BindBuffer(gl.ARRAY_BUFFER, self.vert_buffer_id[0])
  gl.check_errors()

  local ptr, size = libui.int_storage_info(self.face_indexes)
  gl.BufferData(
      gl.ELEMENT_ARRAY_BUFFER,
      size,
      ptr,
      gl.STATIC_DRAW
  )

  ptr, size = libui.double_storage_info(self.verts)
  gl.BufferData(
      GL_ARRAY_BUFFER,
      size,
      ptr,
      gl.STATIC_DRAW
  )

  gl.check_errors()

  -- unbind
  gl.BindBuffer(gl.ARRAY_BUFFER, 0);
  gl.BindBuffer(gl.ELEMENT_ARRAY_BUFFER, 0);
  gl.BindVertexArray(0);
  gl.check_errors()
end

function Mesh:paint()
  gl.BindVertexArray(self.vao_id[0]);
  gl.check_errors()

  for i in 1,#self.submeshes do
    local submesh = self.submeshes[i]
    if (m.material)
      m.material -> setMaterial();

    glDrawElements(
        __mode,
        m.end,
        GL_UNSIGNED_INT,
        BUFFER_OFFSET(sizeof(IndicesType) * m.begin)
      );
    checkGLErrors(AT);

    if (m.material)
      m.material -> unsetTextures();

  }
  glBindVertexArray(0);
end


return Mesh


