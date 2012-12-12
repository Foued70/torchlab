require 'torch'
require 'dok'

utils = {}


local axes = torch.Tensor(3,3)
for i = 1,3 do
   axes[i][i] = 1
end

local x_axis = axes[1]
local y_axis = axes[2]
local z_axis = axes[3]

function utils.normalize(...)
   local v,res
   local args = {...}
   local nargs = #args
   if nargs == 2 then
      res = args[1]
      v   = args[2]
   elseif nargs == 1 then
      v   = args[1]
      res = torch.Tensor(v:size())
   else
      print(dok.usage('utils.normalize',
                      'normalize a vector (L2)', nil,
                      {type='torch.Tensor', help='output'},
                      {type='torch.Tensor', help='input', req=true}))
      dok.error('incorrect arguements', 'utils.normalize')
   end

   res:copy(v)
   local n = res:norm()
   if (n > 1e-8) then
      res:mul(1/n)
   end
   return res
end

function utils.axis_rotation(normal,d)
   local n   = utils.normalize(normal:narrow(1,1,3))
   return utils.quaternion_angle(n,d)
end

function utils.x_rotation(normal)
   return utils.axis_rotation(normal,z_axis)
end

function utils.y_rotation(normal)
   return utils.axis_rotation(normal,z_axis)
end

function utils.z_rotation(normal)
   return utils.axis_rotation(normal,z_axis)
end

function utils.largest_rotation (normal)
   local n   = normal:narrow(1,1,3)
   local p   = torch.Tensor(3):copy(n):abs()
   local v,i = p:sort()
   local d   = i[#i]
   local a   = axes[d]
   if (n[d] < 0) then
      a = a * -1
   end
   return utils.axis_rotation(n,a),d
end

function utils.quat_conjugate(quat,res)
   if (not res) then
      res = torch.Tensor(quat:size()):copy(quat)
   end
   res:narrow(1,1,3):mul(-1)
   return res
end

function utils.rotation_matrix(quaternion, res)
   if (not res) then
      res   = torch.Tensor(4,4)
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

   res[4][4] = 1

   return res
end

-- rotate vector by quaternion 
-- this is an optimized version of 30 ops which we will move C
-- from http://physicsforgames.blogspot.com/2010/03/quaternion-tricks.html
function utils.rotate_by_quat(v,q,res)
   if (not res) then
      res = torch.Tensor(4)
   end
   local x1 = q[2]*v[3] - q[3]*v[2]
   local y1 = q[3]*v[1] - q[1]*v[3]
   local z1 = q[1]*v[2] - q[2]*v[1]

   res[1] = v[1] + 2 * (q[4]*x1 + q[2]*z1 - q[3]*y1)
   res[2] = v[2] + 2 * (q[4]*y1 + q[3]*x1 - q[1]*z1)
   res[3] = v[3] + 2 * (q[4]*z1 + q[1]*y1 - q[2]*x1)
   res[4] = 0
   return res
end

-- rotate vector by rotation matrix
function utils.rotate_by_mat(vector,mat,res)
   if (not res) then
      res = torch.Tensor(4)
   end
   res:addmv(0,1,mat,vector)
   return res
end

-- returns quaternion represnting angle between two vectors
function utils.quaternion_angle(from_vector, to_vector, quat)
   from_vector = from_vector:narrow(1,1,3)
   to_vector = to_vector:narrow(1,1,3)
   if (not quat) then
      quat = torch.Tensor(4):fill(0)
      quat[4] = 1
   end
   local rot_axis = torch.cross(from_vector, to_vector)
   local m        = torch.norm(rot_axis)
   --avoid the degenerate case when from_vector is very close to to_vector
   if(m > 1e-8) then
      rot_axis  = rot_axis/m
      rot_angle = torch.acos(torch.dot(from_vector, to_vector))
      quat:narrow(1,1,3):copy(rot_axis * torch.sin(rot_angle / 2))
      quat[4]   = torch.cos(rot_angle / 2)
   end
   return quat
end

