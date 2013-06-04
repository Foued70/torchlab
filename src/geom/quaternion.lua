Class()

local axes   = torch.eye(3)

x = axes[1]
y = axes[2]
z = axes[3]

function conjugate(quat,res)
   if (not res) then
      res = quat:clone()
   end
   res:narrow(1,1,3):mul(-1)
   return res
end

function equals(quat1, quat2)
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
function product(quat1,quat2,res)
   local zero_quat = torch.Tensor({0,0,0,1})

   if equals(quat1, zero_quat) and equals(quat1, zero_quat) then
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
function angle_between(from_vector, to_vector, quat)
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

   return from_axis_angle(rot_axis, rot_angle, quat)
end

function from_axis_angle(rot_axis, rot_angle, quat)
   if not quat then
      quat = torch.Tensor(4)
   end
   quat[{{1,3}}] = rot_axis * torch.sin(rot_angle / 2)
   quat[4] = torch.cos(rot_angle / 2)

   return quat
end

function to_rotation_matrix(quat, res)
   if (not res) then
      res   = torch.Tensor(3,3)
   end
   res:fill(0)
   local qq  = torch.addr(torch.zeros(4,4),quat,quat)
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
function from_rotation_matrix(rmat, quat, debug)
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

   geom.util.normalize(quat)

   return quat
end


-- if two quaternions are equal they will rotate a vector the same distance
function distance(quat1, quat2)
   local vec1x = rotate(x, quat1)
   local vec2x = rotate(x, quat2)
   local vec1y = rotate(y, quat1)
   local vec2y = rotate(y, quat2)
   local vec1z = rotate(z, quat1)
   local vec2z = rotate(z, quat2)
   return vec1x:dist(vec2x) + vec1y:dist(vec2y) + vec1z:dist(vec2z)
end


-- rotate vector by quaternion
-- this is an optimized version of 30 ops which we will move C
-- from http://physicsforgames.blogspot.com/2010/03/quaternion-tricks.html
function rotate(...)
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

   local x1 = q[2]*v[3] - q[3]*v[2]
   local y1 = q[3]*v[1] - q[1]*v[3]
   local z1 = q[1]*v[2] - q[2]*v[1]

   res[1] = v[1] + 2 * (q[4]*x1 + q[2]*z1 - q[3]*y1)
   res[2] = v[2] + 2 * (q[4]*y1 + q[3]*x1 - q[1]*z1)
   res[3] = v[3] + 2 * (q[4]*z1 + q[1]*y1 - q[2]*x1)
   return res
end