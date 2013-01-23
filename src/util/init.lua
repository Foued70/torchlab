require 'torch'
require 'dok'
require 'paths'

util = {}

-- add files to the local utils table

torch.include('util','util.lua')
torch.include('util','geom.lua')
torch.include('util','pose.lua')
torch.include('util','obj.lua')

function util.run_tests()
   util.test = {}
   torch.include('util','geom-test.lua')
   util.test.geom.all()
   torch.include('util','pose-test.lua')
   util.test.pose.all()
end