Class()
-- TODO: vectorize all these like the new spherical angles 
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
   local diff = quat1 - quat2
   diff:abs()
   return (diff:lt(1e-8):sum() == 4)
end

-- use to concatenate 2 rotations (careful: quat2 then quat1 non-cummutative)
function product(quat1,quat2,res)

   if (not res) then
      res = torch.Tensor(quat2:size())
   end

   if quat2:dim() == 1 then
      local v1 = quat1:narrow(1,1,3)
      local v2 = quat2:narrow(1,1,3)
      local vres = res:narrow(1,1,3)
      vres:copy(torch.cross(v1,v2) + v1*quat2[4] + v2*quat1[4])
      res[4] = quat1[4]*quat2[4] - v1:dot(v2)
   else
      quat1 = quat1:resize(1,4):expandAs(quat2)
      local v1 = quat1:narrow(2,1,3)
      local v2 = quat2:narrow(2,1,3)
      local w1 = quat1[{{},{4}}]:expandAs(v1)
      local w2 = quat2[{{},{4}}]:expandAs(v2)
      local vres = res:narrow(2,1,3)
      local wres = res[{{},4}]
      vres:copy(torch.cross(v1,v2,2)):add(torch.cmul(v1,w2)):add(torch.cmul(v2,w1))
      
      for i = 1,wres:size(1) do 
         wres[i] = quat1[i][4]*quat2[i][4] - v1[i]:dot(v2[i])
      end
   end    
   
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


-- euler angle    = Nx2 or 3 : pitch, yaw and optional roll
-- our spherical coordinates are called phi (pitch) and lambda (yaw)
function from_euler_angle(euler_angle)
   if euler_angle:dim() == 1 then 
      euler_angle:resize(euler_angle:size(1),1)
   end

   local pitch    = euler_angle[1]
   local yaw      = euler_angle[2]
   local roll
 
   if (euler_angle:size(1) == 3) then
      roll = euler_angle[3]
   else
      roll = torch.zeros(pitch:size())
   end

   quat = quat or torch.Tensor(4,pitch:nElement())
   quat:resize(4,pitch:nElement())

   local cpitch      = pitch:clone():mul(0.5):cos()
   local cyaw        = yaw:clone():mul(0.5):cos()
   local croll       = roll:clone():mul(0.5):cos()

   local spitch      = pitch:clone():mul(0.5):sin()
   local syaw        = yaw:clone():mul(0.5):sin()
   local sroll       = roll:clone():mul(0.5):sin()

   local cyaw_cpitch = torch.cmul(cyaw,cpitch)
   local syaw_spitch = torch.cmul(syaw,spitch)
   local cyaw_spitch = torch.cmul(cyaw,spitch)
   local syaw_cpitch = torch.cmul(syaw,cpitch)


   quat[1]:copy(cyaw_cpitch):cmul(sroll):add(-1,torch.cmul(syaw_spitch,croll))
   quat[2]:copy(cyaw_spitch):cmul(croll):add(torch.cmul(syaw_cpitch,sroll))
   quat[3]:copy(syaw_cpitch):cmul(croll):add(-1,torch.cmul(cyaw_spitch,sroll))
   quat[4]:copy(cyaw_cpitch):cmul(croll):add(torch.cmul(syaw_spitch,sroll))

   -- unlike projections, quaternions used in 3D are Nx4
   return quat:t():contiguous()
end

-- <quat> is Nx4
-- <euler_angles> is 3xN
function to_euler_angle(quat,euler_angle)
   local n_elem = 1
   if (quat:dim() == 1) then 
      quat:resize(1,quat:size(1)) 
   else
      n_elem = quat:size(1)
   end

   euler_angle = euler_angle or torch.Tensor(3,n_elem)
   euler_angle:resize(3,n_elem)

   -- q_sqr is 4xN
   local q     = quat:t():clone()
   local q_sqr = torch.cmul(q,q)

   q00 = q_sqr[4]  
   q11 = q_sqr[1]  
   q22 = q_sqr[2]  
   q33 = q_sqr[3]
  
   r11 = q00:clone():add(q11):add(-1,q22):add(-1,q33)
   r21 = torch.cmul(q[1],q[2]):add(torch.cmul(q[4],q[3])):mul(2)

   -- absorbs the negative applied to all future uses
   local r31 = torch.cmul(q[1],q[3]):add(-1,torch.cmul(q[4],q[2])):mul(-2)
   local r32 = torch.cmul(q[2],q[3]):add(torch.cmul(q[4],q[1])):mul(2)
   local r33 = q00:clone():add(-1,q11):add(-1,q22):add(q33)

   tmp = torch.abs(r31)
   gimbal = tmp:gt(0.999999)
   not_gimbal = gimbal:ne(1)
   
   if (not_gimbal:sum() > 0) then
      euler_angle[3][not_gimbal] = torch.atan2(r32[not_gimbal],r33[not_gimbal]) -- roll
      euler_angle[1][not_gimbal] = torch.asin(r31[not_gimbal])         -- pitch
      euler_angle[2][not_gimbal] = torch.atan2(r21[not_gimbal],r11[not_gimbal]) -- yaw
   end

   if (gimbal:sum() > 0) then 
      local r12 = r21
      r12:copy(q[1]):cmul(q[2]):add(-1,torch.cmul(q[4],q[3])):mul(-2)
      local r13 = r32
      r13:copy(q[1]):cmul(q[3]):add(torch.cmul(q[4],q[2])):mul(2)
      r13:cmul(r31)
      
      euler_angle[3][gimbal] = 0                                    -- roll
      tmp[gimbal] = tmp[gimbal]:mul(math.pi/2)
      euler_angle[1][gimbal] = torch.cdiv(r31[gimbal],tmp[gimbal])  -- pitch
      euler_angle[2][gimbal] = torch.atan2(r12[gimbal],r13[gimbal]) -- yaw
   end

   return euler_angle
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