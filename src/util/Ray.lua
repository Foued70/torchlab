require 'math'

local geom = require 'util/geom'

-- Ray primitive.  This class is intentionally light weight.
local Ray = torch.class('Ray')

function Ray:__init(o,d) 
   -- origin starting point of the ray
   if not o then
      o = torch.zeros(3)
   end
   self.origin = o

   -- the direction of the ray
   if not d then 
      d = torch.ones(3)
   end
   -- always normalized
   self.dir     = geom.normalized(d)

   -- inverse direction is stored as this is needed in any
   -- axis-aligned intersection
   self.invdir  = self.dir:clone():pow(-1)

   -- sign is also stored as this is in the inner loop of intersection
   -- tests
   self.sign    = torch.gt(self.dir,0):mul(2):add(-1)

   -- these mint and maxt values are mostly unused
   self.mint    = 0
   self.maxt    = math.huge
end

-- function returns the xyz value for the endpoint of a ray of length t 
function Ray:__call(t)
   return self.origin + (self.dir * t)
end

return Ray