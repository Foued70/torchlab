require 'torchffi'
local geom = require 'util.geom'

-- Reimplementation in torch/C of Laurent Kneip's code. Could also have
-- used Pierre Moulon's openMVG (https://github.com/openMVG/openMVG) but
-- trying to keep a limited number of dependencies. [MS]

--  Copyright (c) 2011, Laurent Kneip, ETH Zurich
--  All rights reserved.
--
--  Redistribution and use in source and binary forms, with or without
--  modification, are permitted provided that the following conditions are met:
--      * Redistributions of source code must retain the above copyright
--        notice, this list of conditions and the following disclaimer.
--      * Redistributions in binary form must reproduce the above copyright
--        notice, this list of conditions and the following disclaimer in the
--        documentation and/or other materials provided with the distribution.
--      * Neither the name of ETH Zurich nor the
--        names of its contributors may be used to endorse or promote products
--        derived from this software without specific prior written permission.
--
--  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
--  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
--  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
--  DISCLAIMED. IN NO EVENT SHALL ETH ZURICH BE LIABLE FOR ANY
--  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
--  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
--  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
--  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
--  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
--  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
-- /

--  P3p.cpp
--
--   Created on: Nov 2, 2010
--       Author: Laurent Kneip
--  Description: Compute the absolute pose of a camera using three 3D-to-2D correspondences
--    Reference: A Novel Parametrization of the P3P-Problem for a Direct Computation of
--               Absolute Camera Position and Orientation
--
--        Input: featureVectors: 3x3 matrix with UNITARY feature vectors (each column is a vector)
--               worldPoints: 3x3 matrix with corresponding 3D world points (each column is a point)
--               solutions: 3x16 matrix that will contain the solutions
--                          form: [ 3x1 position(solution1) 3x3 orientation(solution1) 3x1 position(solution2) 3x3 orientation(solution2) ... ]
--                          the obtained orientation matrices are defined as transforming points from the cam to the world frame
--       Output: int: 0 if correct execution
--                   -1 if world points aligned

-- Load the C versions of intermediate functions
ffi.cdef[[
            int solveQuartic( double *factors, double *realRoots);
            int computeFactors(double f_1, double f_2, 
                               double p_1, double p_2, 
                               double b, double d_12, 
                               double *factors);
            
         ]]

-- FIXME make this loading cleaner for other ffi projects
-- ffi doesn't look in the right place by default
local ffidir = paths.install_lib .. "/torch/lua/"
local p3pC  = ffi.load(ffidir .. "libp3p.dylib")

local p3p = {}

function p3p.compute_poses (camera)
   local xyz = camera.world
   local p1 = xyz[1]
   local p2 = xyz[2]
   local p3 = xyz[3]
 
   local p2p1 = p2 - p1
   local p3p1 = p3 - p1

   if (torch.cross(p2p1,p3p1) == 0) then
      print("Error malformed input world points are colinear")
      return
   end

   local uv   = camera.uv
   local f1  = uv[1]
   local f2  = uv[2]
   f3  = uv[3]

   -- Create intermediate camera frame
   camT = torch.Tensor(3,3)
   camT[1] = f1
   camT[3] = torch.cross(f1,f2)
   geom.normalize(camT[3])

   camT[2] = torch.cross(camT[3],camT[1])
   
   local pf3 = camT*f3 -- make sure > 0

   -- Create intermediate world frame
   local worldT = torch.Tensor(3,3)
   worldT[1] = p2p1
   geom.normalize(worldT[1])
   
   worldT[3] = torch.cross(worldT[1],p3p1)
   geom.normalize(worldT[3])
   
   worldT[2] = torch.cross(worldT[3],worldT[1])

   local pp3 = worldT*p3p1

   local d_12 = p2p1:norm()
   local f_1  = pf3[1]/pf3[3]
   local f_2  = pf3[2]/pf3[3]
   local p_1  = pp3[1]
   local p_2  = pp3[2]

   local cos_beta = f1 * f2
   local b = 1/(1-math.pow(cos_beta,2)) -1

   if (cos_beta < 0) then
      b = -math.sqrt(b)
   else
      b = math.sqrt(b)
   end
   local factors = torch.Tensor(5)
   p3pC.computeFactors(f_1,f_2,p_1,p_2,b,d_12,torch.data(factors))

   local real_roots = torch.Tensor(4)
   p3pC.solveQuartic(torch.data(factors),torch.data(real_roots))
   return real_roots 
end

return p3p