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
-- 

--  P3p.cpp
-- 
--   Created on: Nov 2, 2010
--       Author: Laurent Kneip
--  Description: Compute the absolute pose of a camera using three 3D-to-2D correspondences
--    Reference: A Novel Parametrization of the P3P-Problem for a Direct Computation of
--               Absolute Camera Position and Orientation
-- 
--        Input: 
-- 
--               featureVectors: 3x3 matrix with UNITARY feature
--                 vectors (each column is a vector)
-- 
--               worldPoints: 3x3 matrix with corresponding 3D world
--                 points (each column is a point)
-- 
--               solutions: 3x16 matrix that will contain the
--                 solutions in the form:
-- 
--                 [ 3x1 position(1) 3x3 orientation(1) 
--                   3x1 position(2) 3x3 orientation(2) ... ] 
-- 
--                the obtained orientation matrices are defined as
--                transforming points from the cam to the world frame
-- 
--       Output: int: 0 if correct execution
--                   -1 if world points aligned

-- Load the C versions of intermediate functions
ffi.cdef[[
            int solveQuartic(double *factors, double *realRoots);
            int computeFactors(double *factors, double *pointData);
            int backSubstitute(double *solutions, double *pointData, double *realRoots);
            
         ]]

-- FIXME make this loading cleaner for other ffi projects
-- ffi doesn't look in the right place by default
local ffidir = paths.install_lib .. "/torch/lua/"
local p3pC  = ffi.load(ffidir .. "libp3p.dylib")

local p3p = {}

function p3p.compute_poses (world_pts,camera_angles)
   local xyz = world_pts
   local p1 = xyz[1]
   local p2 = xyz[2]
   local p3 = xyz[3]
 
   local p2p1 = p2 - p1
   local p3p1 = p3 - p1

   if (torch.cross(p2p1,p3p1) == 0) then
      print("Error malformed input world points are colinear")
      return
   end

   -- Carefull input is angles derived from the image
   -- coordinates and camera calibration.  
   -- 
   -- FIXME current code expects this data on Unit Cartesian Sphere
   -- (Directions on unit sphere in camera coordinates with optical
   -- axis at (0,0,1) ??) update to handle our internal spherical
   -- coordinates.  For now: convert azimuth and elevation to
   -- cartesian unit sphere

   local unit_vec = 
      geom.spherical_coords_to_unit_cartesian(camera_angles)
   local f1 = unit_vec[1] 
   local f2 = unit_vec[2]
   local f3 = unit_vec[3]

   -- Create intermediate camera frame
   camT = torch.Tensor(3,3)
   camT[1] = f1
   camT[3] = torch.cross(f1,f2)
   geom.normalize(camT[3])
   camT[2] = torch.cross(camT[3],camT[1])

   local pf3 = camT*f3 

   -- Swap first 2 vectors and worldpoints to keep theta between 0 and
   -- pi. (See paper p.2972 and Figure 4 for explanation).
   if (pf3[3] > 0) then
      print("Swapping") 
      local tmp = f1
      f1 = f2
      f2 = tmp
      camT[1] = f1
      camT[3] = torch.cross(f1,f2)
      geom.normalize(camT[3])
      camT[2] = torch.cross(camT[3],camT[1])

      pf3 = camT*f3 

      p1 = xyz[2]
      p2 = xyz[1]
      p2p1 = p2 - p1 
      p3p1 = p3 - p1
   end
   
   -- Create intermediate world frame
   worldT = torch.Tensor(3,3)
   worldT[1] = p2p1
   geom.normalize(worldT[1])
   
   worldT[3] = torch.cross(worldT[1],p3p1)
   geom.normalize(worldT[3])
   
   worldT[2] = torch.cross(worldT[3],worldT[1])
   local pp3 = worldT*p3p1

   local pointData = torch.Tensor(6)

   pointData[1] = p2p1:norm()   -- d_12
   pointData[2] = pf3[1]/pf3[3] -- f_1
   pointData[3] = pf3[2]/pf3[3] -- f_2
   pointData[4] = pp3[1]        -- p_1
   pointData[5] = pp3[2]        -- p_2

   local cos_beta = f1 * f2
   if cos_beta > 1 then 
      error("cos_beta is >1. some error in data processing")
   end
   local b = (1/(1-math.pow(cos_beta,2))) -1 
   if (cos_beta < 0) then
      b = -math.sqrt(b)
   else
      b = math.sqrt(b)
   end
   pointData[6] = b

   local factors = torch.Tensor(5)
   p3pC.computeFactors(torch.data(factors),
                       torch.data(pointData))

   local real_roots = torch.Tensor(4)
   p3pC.solveQuartic(torch.data(real_roots),
                     torch.data(factors))
                     
   local solutions = torch.Tensor(4,4,3)
   p3pC.backSubstitute(torch.data(solutions),
                       torch.data(pointData),
                       torch.data(real_roots))
                     
   -- return solutions to world coordinate frame
   for si = 1,4 do
      C = solutions[si][1]
      R = solutions[si]:narrow(1,2,3)
      -- C = P1 + N.T()*C
      C:copy(p1 + (worldT:t()*C)) 
      -- R = N.T()*R.T()*T 
      local temp = worldT:t()*R:t()*camT
      R:copy(temp:t())
   end

   -- FIXME (perhaps?) return best solution
   return solutions
end

return p3p