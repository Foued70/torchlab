local gl = require 'ui2.gl'

local Object = torch.class('Object')

function Object:__init(obj_data)
  self.obj_data = obj_data
  self.mesh = Object.create_mesh(obj_data)

  self.position = torch.Tensor(3):fill(0)
end

function Object.create_mesh(obj_data)
  local materials = {}
  for _, mtl_data in ipairs(obj_data.materials) do
    local material = require('ui2.Material').new(mtl_data)
    table.insert(materials, material)
  end

  local submeshes = {}
  for i = 1, obj_data.submeshes:size()[1] do
    local sub = obj_data.submeshes[i]
    table.insert(submeshes, {start = sub[1], length = sub[2] - sub[1] + 1, material = materials[sub[3]]})
  end

  return require('ui2.Mesh').new(obj_data.unified_verts, obj_data.faces, submeshes)
end

function Object:paint(context)
  context:push()

  context:translate(self.position)

  self.mesh:paint(context)
  
  context:pop()
end

function Object:get_triangle(triangle_id)
  local vert_indices = self.mesh.faces[triangle_id]

  -- todo: use the new tensor indexer that Marco wrote
  local verts = torch.Tensor(3, self.mesh.verts:size(2))
  for i = 1,3 do
    verts[i] = self.mesh.verts[vert_indices[i]]
  end

  local normal = self.obj_data.face_normals[triangle_id]
  local center = self.obj_data.face_centers[triangle_id]

  return verts, center, normal
end


return Object
