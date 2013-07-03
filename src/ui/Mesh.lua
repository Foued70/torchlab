local gl = require './gl'

local Mesh = Class()

function Mesh:__init(verts, faces, submeshes)
  self.verts = verts
  self.faces = faces
  self.submeshes = submeshes

  self.vao_id = nil
  self.vert_buffer_id = nil
  self.element_buffer_id = nil
  self.pushed = false
end


function Mesh:push_to_gl()
  self.vao_id = gl.GenVertexArray()
  self.vert_buffer_id = gl.GenBuffer()
  self.element_buffer_id = gl.GenBuffer()
  gl.check_errors()

  gl.BindVertexArray(self.vao_id)
  gl.BindBuffer(gl.ELEMENT_ARRAY_BUFFER, self.element_buffer_id)
  gl.BindBuffer(gl.ARRAY_BUFFER, self.vert_buffer_id)
  gl.check_errors()

  local ptr, size = util.ctorch.storage_info(self.faces - 1)
  gl.BufferData(
      gl.ELEMENT_ARRAY_BUFFER,
      size,
      ptr,
      gl.STATIC_DRAW
  )
  gl.check_errors()


  ptr, size = util.ctorch.storage_info(self.verts)
  local vert_size = size / self.verts:size()[1] -- bytes per row
  gl.BufferData(
      gl.ARRAY_BUFFER,
      size,
      ptr,
      gl.STATIC_DRAW
  )
  gl.check_errors()

  start = gl.double_ptr(nil)
  gl.VertexAttribPointer(0, 4, gl.DOUBLE, gl.FALSE, vert_size, start+0) -- position
  gl.VertexAttribPointer(1, 2, gl.DOUBLE, gl.FALSE, vert_size, start+4) -- uv
  gl.VertexAttribPointer(2, 3, gl.DOUBLE, gl.FALSE, vert_size, start+6) -- normal
  gl.check_errors()

  gl.EnableVertexAttribArray(0);
  gl.EnableVertexAttribArray(1);
  gl.EnableVertexAttribArray(2);
  gl.check_errors()
  
  -- unbind TODO do we need this?
  gl.BindBuffer(gl.ARRAY_BUFFER, 0);
  gl.BindBuffer(gl.ELEMENT_ARRAY_BUFFER, 0);
  gl.BindVertexArray(0);
  gl.check_errors()

  self.pushed = true
end

function Mesh:override_materials(material)
  self.default_materials = {}
  for i = 1, #self.submeshes do
    self.default_materials[i] = self.submeshes[i].material
    self.submeshes[i].material = material
  end
end

function Mesh:restore_materials()
  if self.default_materials == nil then
    return
  end
  
  for i = 1, #self.submeshes do
    self.submeshes[i].material = self.default_materials[i]
  end
  self.default_materials = nil
end

function Mesh:apply_material_to_all_(material)
  log.trace(self.submeshes)
  for i = 1, #self.submeshes do
    log.trace('start= ' .. self.submeshes[i].start)
    log.trace('length= ' .. self.submeshes[i].length)
    log.trace('material= ' .. self.submeshes[i].material.name)
  end
end

function Mesh:paint(context)
  if not self.pushed then self:push_to_gl() end

  gl.BindVertexArray(self.vao_id)
  gl.BindBuffer(gl.ELEMENT_ARRAY_BUFFER, self.element_buffer_id)
  gl.BindBuffer(gl.ARRAY_BUFFER, self.vert_buffer_id)
  gl.check_errors()

  for i, submesh in ipairs(self.submeshes) do
    local submesh = self.submeshes[i]
    context.submesh_start = submesh.start
    if (submesh.material) then
      submesh.material:use(context)
    end

    gl.DrawElements(
        gl.TRIANGLES,
        submesh.length * 3,
        gl.UNSIGNED_INT,
        gl.uint_ptr(nil) + ((submesh.start - 1) * 3)
    )
    gl.check_errors()

  end

  gl.BindBuffer(gl.ARRAY_BUFFER, 0);
  gl.BindBuffer(gl.ELEMENT_ARRAY_BUFFER, 0);
  gl.BindVertexArray(0)
end

function Mesh:dump_buffer(buf_type, ptr_type, rows, columns)
  local buf = gl.MapBuffer(buf_type, gl.READ_ONLY)
  gl.check_errors()
  log.trace(buf)
  
  local arr = ptr_type(buf)
  for i = 0, rows-1 do
    local offset = i * columns
    local out = ''
    for j = 0, columns-1 do
      out = out..arr[offset + j]..'\t'
    end
    print(out)
  end

  gl.UnmapBuffer(buf_type)
  gl.check_errors()
end

function Mesh:dump_buffers()
  self:dump_buffer(gl.ARRAY_BUFFER, gl.double_ptr, self.verts:size()[1], self.verts:size()[2])
  self:dump_buffer(gl.ELEMENT_ARRAY_BUFFER, gl.uint_ptr, self.faces:size()[1], self.faces:size()[2])
end


return Mesh


