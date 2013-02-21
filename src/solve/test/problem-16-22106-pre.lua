require 'torch'


local ncam = 16
local npts = 22106
local nobs = 83718

local thisdir = paths.dirname(paths.thisfile())

-- had to split the tensors into separate files as torch or lua has a
-- limit (main function has more than 65536 constants)

local cam_index    = dofile(thisdir.."/problem-16-22106-pre-cam_index.lua")
local pts_index    = dofile(thisdir.."/problem-16-22106-pre-pt_index.lua")
local parameters   = dofile(thisdir.."/problem-16-22106-pre-parameters.lua")
local observations = dofile(thisdir.."/problem-16-22106-pre-observations.lua")

observations:resize(nobs,2)

local cameras = parameters:narrow(1,1,9*ncam):resize(ncam,3,3)
local points  = parameters:narrow(1,9*ncam,npts*3):resize(npts,3)


return { ncam         = ncam,
         npts         = npts,
         nobs         = nobs,
         cam_index    = cam_index,
         pts_index    = pts_index,
         parameters   = parameters,
         observations = observations,
         cameras      = cameras,
         points       = points }
