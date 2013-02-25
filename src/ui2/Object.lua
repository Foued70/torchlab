local gl = require 'ui2.gl'

local Object = torch.class('Object')

function Object:__init(obj_data)
  self.materials = {}
  for _, mtl_data in ipairs(obj_data.materials) do
    local material = require('ui2.Material').new(mtl_data)
    table.insert(self.materials, material)
  end

  local submeshes = {}
  for i = 1, obj_data.submeshes:size()[1] do
    local sub = obj_data.submeshes[i]
    table.insert(submeshes, {start = sub[1], length = sub[2] - sub[1] + 1, material = self.materials[sub[3]]})
  end

  self.mesh = require('ui2.Mesh').new(obj_data.unified_verts, obj_data.faces, submeshes)

  self.position = torch.Tensor(3):fill(0)
end

function Object:paint(context)
  context:push()

  context:translate(self.position)
  self.mesh:paint(context)
  
  context:pop()
end


return Object
