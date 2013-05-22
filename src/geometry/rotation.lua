require 'torch'
require 'dok'

local gutil = geometry.util
Class()

-- Below is a test for loading C backends using ffi
local ffi_utils = require 'util.ffi'

local ffi = ffi_utils.ffi

local geomlib = ffi_utils.lib_path("geom")

if paths.filep(geomlib) then
   -- load low level c functions
   ffi.cdef[[
               void rotate_by_quat(THDoubleTensor *result,
                                   THDoubleTensor *vectors,
                                   THDoubleTensor *quat);

               void rotate_translate(THDoubleTensor *result,
                                     THDoubleTensor *vectors,
                                     THDoubleTensor *trans,
                                     THDoubleTensor *quat);

               void translate_rotate(THDoubleTensor *result,
                                     THDoubleTensor *vectors,
                                     THDoubleTensor *trans,
                                     THDoubleTensor *quat);
            ]]

   -- don't want to call C functions directly
   local C  = ffi.load(geomlib)

   -- either there is a way to stick function on torch.DoubleTensor
   -- for automatic type cast or we just do everything in doubles.
   function rotate_by_quatC (out, vec, quat)
      C.rotate_by_quat(torch.cdata(out), torch.cdata(vec),
                       torch.cdata(quat))
   end
   function rotate_translateC (out, vec, trans, quat)
      C.rotate_translate(torch.cdata(out), torch.cdata(vec),
                         torch.cdata(trans), torch.cdata(quat))
   end
   function translate_rotateC (out, vec, trans, quat)
      C.translate_rotate(torch.cdata(out), torch.cdata(vec),
                         torch.cdata(trans), torch.cdata(quat))
   end
end

-- end of loading C backend

local axes   = torch.eye(3)

x_axis = axes[1]
y_axis = axes[2]
z_axis = axes[3]

local neg_axes   = torch.eye(3):mul(-1)

function axis_rotation(normal,d)
   local n   = gutil.normalized(normal:narrow(1,1,3))
   return quaternion_from_to(n,d)
end

function x_rotation(normal)
   return axis_rotation(normal,z_axis)
end

function y_rotation(normal)
   return axis_rotation(normal,z_axis)
end

function z_rotation(normal)
   return axis_rotation(normal,z_axis)
end

function largest_rotation (normal)
   local n   = normal:narrow(1,1,3)
   local p   = n:clone():abs()
   local v,i = p:sort()
   local d   = i[-1]
   local a   = axes[d]
   if (n[d] < 0) then
      a = neg_axes[d]
   end
   return axis_rotation(n,a),d
end

function quaternion_to_rotation_matrix(quaternion, res)
   if (not res) then
      res   = torch.Tensor(3,3)
   end
   res:fill(0)
   local qq  = torch.addr(torch.zeros(4,4),quaternion,quaternion)
   qq:mul(2)

   res[1][1] = 1 - qq[2][2] - qq[3][3]
   res[1][2] =     qq[1][2] - qq[4][3]
   res[1][3] =     qq[1][3] + qq[4][2]

   res[2][1] =     qq[1][2] + qq[4][3]
   res[2][2] = 1 - qq[1][1] - qq[3][3]
   res[2][3] =     qq[2][3] - qq[4][1]

   res[3][1] =     qq[1][3] - qq[4][2]
   res[3][2] =     qq[2][3] + qq[4][1]
   res[3][3] = 1 - qq[1][1] - qq[2][2]

   return res
end

-- returns nil on a badly formed rotation matrix
function rotation_matrix_to_quaternion (rmat, quat, debug)
   if (not quat) then
      quat = torch.Tensor(4)
   end

   quat[1] =  rmat[1][1] - rmat[2][2] - rmat[3][3] + 1
   quat[2] = -rmat[1][1] + rmat[2][2] - rmat[3][3] + 1
   quat[3] = -rmat[1][1] - rmat[2][2] + rmat[3][3] + 1
   quat[4] =  rmat[1][1] + rmat[2][2] + rmat[3][3] + 1
   quat:mul(0.25)

   -- set everything less than 0 to 0
   quat[quat:lt(0)] = 0
   -- take square root
   quat:sqrt()

   local qsign = torch.Tensor(4)

   if ((quat[4] >= quat[1]) and (quat[4] >= quat[2]) and (quat[4] >= quat[3])) then

      qsign[1] = rmat[3][2] - rmat[2][3]
      qsign[2] = rmat[1][3] - rmat[3][1]
      qsign[3] = rmat[2][1] - rmat[1][2]
      qsign[4] = 1

   elseif ((quat[1] >= quat[4]) and (quat[1] >= quat[2]) and (quat[1] >= quat[3])) then

      qsign[1] = 1
      qsign[2] = rmat[2][1] + rmat[1][2]
      qsign[3] = rmat[1][3] + rmat[3][1]
      qsign[4] = rmat[3][2] - rmat[2][3]

   elseif ((quat[2] >= quat[4]) and (quat[2] >= quat[1]) and (quat[2] >= quat[3])) then

      qsign[1] = rmat[2][1] + rmat[1][2]
      qsign[2] = 1
      qsign[3] = rmat[3][2] + rmat[2][3]
      qsign[4] = rmat[1][3] - rmat[3][1]

   elseif ((quat[3] >= quat[4]) and (quat[3] >= quat[1]) and (quat[3] >= quat[2])) then

      qsign[1] = rmat[3][1] + rmat[1][3]
      qsign[2] = rmat[3][2] + rmat[2][3]
      qsign[3] = 1
      qsign[4] = rmat[2][1] - rmat[1][2]

   else
      if debug then
         print("Bad input perhaps not a rotation matrix?")
      end
      return nil
   end

   qsign:sign() -- convert to just +1,-1
   -- put signs into quaternion
   quat:cmul(qsign)

   gutil.normalize(quat)

   return quat
end


-- if two quaternions are equal they will rotate a vector the same distance
function quaternion_dist(quat1, quat2)
   local vec1x = rotate_by_quat(x_axis, quat1)
   local vec2x = rotate_by_quat(x_axis, quat2)
   local vec1y = rotate_by_quat(y_axis, quat1)
   local vec2y = rotate_by_quat(y_axis, quat2)
   local vec1z = rotate_by_quat(z_axis, quat1)
   local vec2z = rotate_by_quat(z_axis, quat2)
   return vec1x:dist(vec2x) + vec1y:dist(vec2y) + vec1z:dist(vec2z)
end

-- rotate a vector around axis by angle radians
function rotate_axis_angle(vec, rot_axis, rot_angle)
   local quat = quaternion_from_axis_angle(rot_axis, rot_angle)
   return rotate_by_quat(vec, vec, quat)
end

-- rotate vector by rotation matrix
function rotate_by_mat(...)
   local res,vec,mat
   local args = {...}
   local nargs = #args
   if nargs == 3 then
      res = args[1]
      vec   = args[2]
      mat   = args[3]
   elseif nargs == 2 then
      vec   = args[1]
      mat   = args[2]
      res = torch.Tensor(3)
   else
      print(dok.usage('rotate_by_mat',
                      'rotate a vector by rotation matrix',
                      '> returns: rotated vector',
                      {type='torch.Tensor', help='result'},
                      {type='torch.Tensor', help='vector', req=true},
                      {type='torch.Tensor', help='matrix', req=true}))
      dok.error('incorrect arguements', 'rotate_by_mat')
   end
   res:addmv(0,1,mat,vec)
   return res:narrow(1,1,3)
end

-- rotate vector by quaternion
-- this is an optimized version of 30 ops which we will move C
-- from http://physicsforgames.blogspot.com/2010/03/quaternion-tricks.html
function rotate_by_quat(...)
   local res,v,q
   local args = {...}
   local nargs = #args
   if nargs == 3 then
      res = args[1]
      v   = args[2]
      q   = args[3]
   elseif nargs == 2 then
      v   = args[1]
      q   = args[2]
      res = torch.Tensor(3)
   else
      print(dok.usage('rotate_by_quat',
                      'rotate a vector by quaternion',
                      '> returns: rotated vector',
                      {type='torch.Tensor', help='result'},
                      {type='torch.Tensor', help='vector', req=true},
                      {type='torch.Tensor', help='quaternion',   req=true}))
      dok.error('incorrect arguements', 'rotate_by_quat')
   end
   -- FIXME make this lowlevel C and add possibility to rotate a set of vectors
   local x1 = q[2]*v[3] - q[3]*v[2]
   local y1 = q[3]*v[1] - q[1]*v[3]
   local z1 = q[1]*v[2] - q[2]*v[1]

   res[1] = v[1] + 2 * (q[4]*x1 + q[2]*z1 - q[3]*y1)
   res[2] = v[2] + 2 * (q[4]*y1 + q[3]*x1 - q[1]*z1)
   res[3] = v[3] + 2 * (q[4]*z1 + q[1]*y1 - q[2]*x1)
   return res
end

function quat_conjugate(quat,res)
   if (not res) then
      res = quat:clone()
   end
   res:narrow(1,1,3):mul(-1)
   return res
end

function quat_equals(quat1, quat2)
   if (  (math.abs(quat2[1]-quat1[1])<1e-8) and
         (math.abs(quat2[2]-quat1[2])<1e-8) and
         (math.abs(quat2[3]-quat1[3])<1e-8) and
         (math.abs(quat2[4]-quat1[4])<1e-8) ) then
      return true
   else
      return false
   end
end

-- use to concatenate 2 rotations (careful: quat2 then quat1 non-cummutative)
-- FIXME test cases
function quat_product(quat1,quat2,res)
   local zero_quat = torch.Tensor({0,0,0,1})

   if quat_equals(quat1, zero_quat) and quat_equals(quat1, zero_quat) then
      if (not res) then
         return zero_quat
      else
         return res:copy(zero_quat)
      end
   end

   if (not res) then
      res = torch.Tensor(4)
   end

   local v1 = quat1:narrow(1,1,3)
   local v2 = quat2:narrow(1,1,3)
   local vres = res:narrow(1,1,3)
   vres:copy(torch.cross(v1,v2) + v1*quat2[4] + v2*quat1[4])
   res[4] = quat1[4]*quat2[4] - v1:dot(v2)
   return res
end

-- returns quaternion represnting angle between two vectors
function quaternion_from_to(from_vector, to_vector, quat)
   from = from_vector:narrow(1,1,3)
   to   = to_vector:narrow(1,1,3)

   local rot_axis = torch.cross(from, to)
   local rot_angle = 0

   -- avoid the degenerate case when from_vector is very close to to_vector
   local m = torch.norm(rot_axis)
   if(m > 1e-8) then
      rot_axis  = rot_axis/m
      rot_angle = torch.acos(torch.dot(from, to))
   end

   return quaternion_from_axis_angle(rot_axis, rot_angle, quat)
end

function quaternion_from_axis_angle(rot_axis, rot_angle, quat)
   if not quat then
      quat = torch.Tensor(4)
   end
   quat[{{1,3}}] = rot_axis * torch.sin(rot_angle / 2)
   quat[4] = torch.cos(rot_angle / 2)

   return quat
end

-- input:  Nx3 tensor of unit cartesian vectors
-- output: Nx2 tensor of azimuth and elevation for a set of points
-- these are in camera coords
--    x : right
--    y : forward
--    z : up
function unit_cartesian_to_spherical_coords(uc)

   local x = uc[{{},1}]
   local y = uc[{{},2}]
   local z = uc[{{},3}]

   local angles = torch.Tensor(uc:size(1),2)

   angles[{{},1}] = torch.atan2(x,y) -- azimuth
   angles[{{},2}] = torch.asin(z)   -- elevation

   return angles

end

-- input:  Nx2 tensor of azimuth and elevation for a set of points
-- output: Nx3 tensor of unit cartesian vectors
-- these are in camera coords
--    x : right
--    y : forward
--    z : up
function spherical_coords_to_unit_cartesian(angles)

   local azimuth   = angles[{{},1}]
   local elevation = angles[{{},2}]

   local unit_vec = torch.Tensor(angles:size(1),3)
   local cos_elevation = torch.cos(elevation)

   -- right = sin(azimuth) * cos(elevation)
   unit_vec[{{},1}]  = torch.sin(azimuth):cmul(cos_elevation)
   -- forward = cos(azimuth) * cos(elevation)
   unit_vec[{{},2}]  = torch.cos(azimuth):cmul(cos_elevation)
   -- up = sin(elevation)  or cos(pi/2 - elevation) == cos(polar)
   unit_vec[{{},3}]  = torch.sin(elevation)

   return unit_vec
end

