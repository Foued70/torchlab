local libui = require 'libui2'
local gl = require 'ui2.gl'

local AnimationManager = torch.class('AnimationManager')

function AnimationManager:__init()
  self.var1 = 1
  self.var2 = 2
  self.stringVar = "Hello World!"
  self.loopCount = 10
  
  for i = 1, self.loopCount do
    local outString = self.stringVar .. " " .. i
    log.info(outString)
  end
  
end

return AnimationManager