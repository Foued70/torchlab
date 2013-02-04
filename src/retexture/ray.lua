require 'math'
local util = require 'util'
local geom = util.geom

-- Ray primitive
local Ray = torch.class('Ray')

function Ray:__init(o,d) 
   self.origin = o
   self.dir     = geom.normalize(d)
   self.invdir  = self.dir:clone():pow(-1)
   self.sign    = torch.gt(self.dir,0):mul(2):add(-1)
   self.mint    = 0
   self.maxt    = math.huge
end

function Ray:__call(t)
   return self.origin + (self.dir * t)
end

