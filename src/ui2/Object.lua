local gl = require 'gl'
local geom = require 'geom'

local Object = torch.class('Object')

function Object:__init()
  self.mesh = require('ui2.Mesh').new()
end

function Object:paint()

end
