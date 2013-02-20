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

-- FIXME make this loading cleaner for other ffi projects
-- ffi doesn't look in the right place by default
local ffidir = paths.install_lib .. "/torch/"
local solveC = ffi.load(ffidir .. "libsolve.dylib")

local solve = {}

-- torch interface to simple bundle adjust
function solve.simplesba(cam_index,pts_index,obs,prm)

   cam_index = cam_index:contiguous()
   local cam_index_cdata = torch.data(cam_index)
   local ncam      = cam_index:max() + 1

   pts_index = pts_index:contiguous()
   local pts_index_cdata = torch.data(pts_index)
   local npts      = pts_index:max() + 1

   obs = obs:contiguous()
   local obs_cdata = torch.data(obs)
   local nobs      = obs:size(1)

   prm = prm:contiguous()
   local prm_cdata = torch.data(prm)
   local nprm      = prm:size(1)

   if (cam_index:size(1) ~= pts_index:size(1)) then
      print("ERROR cam_index not same size as pts_index")
      return
   end

   if (cam_index:size(1) ~= nobs) then
      print("ERROR cam_index not same size as number of observations")
      return
   end

   if (pts_index:size(1) ~= nobs) then
      print("ERROR pts_index not same size as number of observations")
      return
   end
   
   solveC.simplesba(cam_index_cdata,ncam,
                    pts_index_cdata,npts,
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

