require 'torch'
require 'torchffi'

local ffi = require 'ffi'

ffi.cdef[[ 
            void simplesba(int* points, int npts, int* cameras, int ncam, double *obs, int nobs);
            int loadBALfile(char* fname); 
         ]]

local solveC = ffi.load("/Users/marco/local/lib/torch/libsolve.dylib")

local solve = {}

function solve.simplesba(pts,cam,obs)
   local pts_cdata = torch.data(pts)
   local npts      = pts:size(1)
   local cam_cdata = torch.data(cam)
   local ncam      = cam:size(1)
   local obs_cdata = torch.data(obs)
   local nobs      = obs:size(1)
   solveC.simplesba(pts_cdata,npts,
                    cam_cdata,ncam,
                    obs_cdata,nobs)
end

function solve.loadbalfile(fname)
   local fcdata = ffi.new("char[?]",fname:len())
   ffi.copy(fcdata,fname)
   return solveC.loadBALfile(fcdata)
end

return solve

