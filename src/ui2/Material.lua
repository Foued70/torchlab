local gl = require 'gl'

local Material = torch.class('Material')

function Material:__init()
  self.ambient = {0.2, 0.2, 0.2, 1}
  self.diffuse = {0.8, 0.8, 0.8, 1}
  self.specular = {0, 0, 0, 1}
  self.shininess = 0
  self.emission = {0, 0, 0, 1}
end


return Material

