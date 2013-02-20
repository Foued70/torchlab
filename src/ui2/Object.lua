local gl = require 'ui2.gl'

local Object = torch.class('Object')

function Object:__init(obj_data)
  self.materials = {}
  for _, mtl_data in ipairs(obj_data.materials) do
    local material = require('ui2.Material').new(mtl_data)
    table.insert(self.materials, material)
  end

  local submeshes = {}
    for _, sub in ipairs(obj_data.submeshes) do
    table.insert(submeshes, {start = sub[1], length = sub[2] - sub[1] + 1, material = self.materials[sub[3]]})
  end

  self.mesh = require('ui2.Mesh').new(obj_data.unified_verts, obj_data.faces, submeshes)
end

function Object:paint(context)
  self.mesh:paint(context)
end


return Object
