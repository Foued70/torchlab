require 'torchffi'

local ffi = require 'ffi'

ffi.cdef[[ 
            void simplesba(int* cameras,   int ncam,
                           int* points,    int npts,  
                           double *obs,    int nobs,
                           double *params, int nparams);
            int loadBALfile(char* fname); 
         ]]

local solveC = util.ffi.load("libsolve")

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
 
-- Test the torch interface to simplesba by loading a sample bal file
-- which is already stored as torch tensors
function solve.testsimplesba()
   local bal = require ("solve/test/problem-16-22106-pre")
   solve.simplesba(bal.cam_index,bal.pts_index, 
                   bal.observations,bal.parameters)
end

-- Load a bal file. Same as simple_bundle_adjust in ceres examples.
function solve.loadbalfile(fname)
   -- FIXME this string copy is annoyingly not working...
   local fcdata = ffi.new("char[?]",fname:len())
   ffi.copy(fcdata,fname,ffi.sizeof(fcdata))
   return solveC.loadBALfile(fcdata)
end

return solve

