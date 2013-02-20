require 'torch'
require 'torchffi'

local ffi = require 'ffi'

ffi.cdef[[ 
            void simplesba(int* cameras, int ncam,
                           int* points,  int npts,  
                           double *obs, int nobs,
                           double *params, int nparams);
            int loadBALfile(char* fname); 
         ]]

local solveC = ffi.load("/Users/marco/local/lib/torch/libsolve.dylib")

local solve = {}

-- torch interface to simple bundle adjust
function solve.simplesba(cam,pts,obs,prm)
   
   local cam_cdata = torch.data(cam)
   local ncam      = cam:max() + 1

   local pts_cdata = torch.data(pts)
   local npts      = pts:max() + 1

   local obs_cdata = torch.data(obs)
   local nobs      = obs:size(1)

   local prm_cdata = torch.data(prm)
   local nprm      = prm:size(1)

   solveC.simplesba(cam_cdata,ncam,
                    pts_cdata,npts,
                    obs_cdata,nobs,
                    prm_cdata,nprm)
end

-- load a sample bal file which is already stored as torch tensors, to
-- test the torch interface to simplesba
function solve.testsimplesba()
   local bal = dofile("test/problem-16-22106-pre.lua")
   solve.simplesba(bal.cam_index,bal.pt_index, 
                   bal.observations,bal.parameters)
end

-- load a bal file sample as simple_bundle_adjust in ceres examples
function solve.loadbalfile(fname)
   -- FIXME this string copy is annoyingly not working...
   local fcdata = ffi.new("char[?]",fname:len())
   ffi.copy(fcdata,fname,ffi.sizeof(fcdata))
   return solveC.loadBALfile(fcdata)
end

return solve

