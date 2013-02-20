require 'torch'

local balproblem16 = {}

balproblem16.ncam = 16
balproblem16.npts = 22106
balproblem16.nobs = 83718

local thisdir = paths.dirname(paths.thisfile())

-- had to split the tensors into separate files as torch or lua has a
-- limit (main function has more than 65536 constants)

balproblem16.cam_index    = dofile(thisdir.."/problem-16-22106-pre-cam_index.lua")
balproblem16.pt_index     = dofile(thisdir.."/problem-16-22106-pre-pt_index.lua")
balproblem16.parameters   = dofile(thisdir.."/problem-16-22106-pre-parameters.lua")
balproblem16.observations = dofile(thisdir.."/problem-16-22106-pre-observations.lua")

balproblem16.observations:resize(balproblem16.nobs,2)

return balproblem16
