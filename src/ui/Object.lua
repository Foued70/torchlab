local gl = ui.gl

local Object = Class()

function Object:__init(widget, obj_data)
  self.widget = widget;
  self.obj_data = obj_data
  self.mesh = Object.create_mesh(widget, obj_data)

  self.position = torch.Tensor(3):fill(0)
  self.rotation = torch.Tensor({0,0,0,1})
end

function Object.create_mesh(widget, obj_data)
  local materials = {}
  for _, mtl_data in ipairs(obj_data.materials) do
    local material = require('ui.Material').new(widget, mtl_data)
    table.insert(materials, material)
  end

  local submeshes = {}
  for i = 1, obj_data.submeshes:size()[1] do
    local sub = obj_data.submeshes[i]    
    table.insert(submeshes, {start = sub[1], length = sub[2] - sub[1] + 1, material = materials[sub[3]]})
  end

  return ui.Mesh.new(obj_data.unified_verts, obj_data:get_tris(), submeshes)
end

function Object:paint(context)
  context:push()

  context:set_model(self.rotation, self.position)
  --context:rotate(self.rotation)
  --context:translate(self.position)

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
