Class()

-- TODO: change all these functions to apply for 
-- 
-- + single quats   dims: (4) or (1,4) or (4,1) and 
-- + single vectors dims: (3),(4),(1,3),(3,1),(1,4),(4,1)
-- 
-- to muliple tensors of above dimensions plus  
-- 
-- + multiple quats   dims: (Nx4)
-- + mulitple vectors dims: (Nx3),(Nx4)
local axes   = torch.eye(3)

x = axes[1]
y = axes[2]
z = axes[3]

-- DONE: multiple
function conjugate(quat,res)
   if (not res) then
      res = quat:clone()
   elseif (quat ~= res) then
      res:resizeAs(quat):copy(quat)
   end
   res:narrow(quat:nDimension(),1,3):mul(-1)
   return res
end

function equals(quat1, quat2)
   local diff = quat1 - quat2
   diff:abs()
   return (diff:lt(1e-8):sum() == 4)
end

function inverse(quat)
   local dis = quat:norm()
   local quat_new = quat:clone()*-1
   quat_new[4]= -quat_new[4]
   return quat_new/dis
end
-- use to concatenate 2 rotations (careful: quat2 then quat1
-- non-cummutative)
-- DONE: multiple
function product(quat1,quat2,res)

   local q1_nelem = quat1:nElement()
   local q2_nelem = quat2:nElement()

   if (not res) then
      res = torch.Tensor(q1_nelem * q2_nelem / 4)
   end

   if ((q1_nelem == 4) and (q2_nelem == 4)) then
      
      res:resize(4)
      quat1 = quat1:squeeze()
      quat2 = quat2:squeeze()
      local v1   = quat1:narrow(1,1,3)
      local v2   = quat2:narrow(1,1,3)
      local vres = res:narrow(1,1,3)
      vres:copy(torch.cross(v1,v2) + v1*quat2[4] + v2*quat1[4])
      res[4] = quat1[4]*quat2[4] - v1:dot(v2)
   else
      if (q1_nelem == 4) then 
         quat1 = quat1:reshape(1,4):expandAs(quat2)
      elseif (q2_nelem == 4) then 
         quat2 = quat2:reshape(1,4):expandAs(quat1)
      else
         error("currently product() only accepts 1x4,Nx4 or Nx4,1x4 pairs")
      end
      res:resize(quat1:size(1),4)
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

-- returns quaternion representing angle between two vectors
-- DONE: multiple
function angle_between(from_vector, to_vector, quat)
   local from_nElem = from_vector:nElement()
   local   to_nElem =   to_vector:nElement()
   local from_nDim  = from_vector:nDimension()
   local   to_nDim  =   to_vector:nDimension()
   local from_nVec  = from_nElem/from_vector:size(from_nDim)
   local   to_nVec  =   to_nElem/to_vector:size(to_nDim)

   quat = quat or torch.Tensor(from_nVec * to_nVec , 4)

   local from = from_vector:narrow(from_nDim,1,3):reshape(from_nVec,3)
   local to   = to_vector:narrow(to_nDim,1,3):reshape(to_nVec,3)
   
   from_nElem = from:nElement()
   to_nElem   = to:nElement()

   if (quat:size(1) == 1) then 
      local rot_axis = torch.cross(from,to)
      local rot_angle = 0

      -- avoid the degenerate case when from_vector is very close to to_vector
      local m = torch.norm(rot_axis)
      if(m > 1e-8) then
         rot_axis  = rot_axis/m
         rot_angle = torch.acos(torch.dot(from, to))
      end

      return from_axis_angle(rot_axis, rot_angle, quat)
   else
      
      
      if (from_nElem == 3) then 
         from = from:reshape(1,3):expand(to_nVec,3)
      elseif (to_nElem == 3) then 
         to = to:reshape(1,3):expand(from_nVec,3)
      else
         error("currently angle_between only accepts 1x(3or4),Nx(3or4) or Nx(3or4),1x(3or4) pairs")
      end

      local rot_axis  = torch.cross(from,to,2)
      
      -- avoid the degenerate case when from_vector is very close to to_vector
      local rot_angle = torch.cmul(from, to):sum(from:nDimension()):acos()
      local m        = rot_axis:norm(2,2)
      local invalid  = m:lt(1e-8)
      rot_angle[invalid] = 0

      m[m:lt(1e-8)]  = 1 

      m = m:resize(m:size(1),1):expandAs(rot_axis)
      rot_axis:cdiv(m)

      return from_axis_angle(rot_axis, rot_angle, quat)

   end
end

-- DONE: multiple
-- TODO: make axis_angle a single Nx4 tensor like quat
function from_axis_angle(rot_axis, rot_angle, quat)
   if (type(rot_angle) == "number") then
      quat = quat or torch.Tensor(4)
      quat:resize(4)
      quat[{{1,3}}] = rot_axis * torch.sin(rot_angle / 2)
      quat[4]       = torch.cos(rot_angle / 2)
   else
      local n_vec = rot_angle:nElement()

      quat = quat or torch.Tensor(n_vec,4)
      quat:resize(n_vec,4)

      rot_axis = rot_axis:narrow(rot_axis:nDimension(),1,3)
      local a = torch.mul(rot_angle,0.5):sin():resize(n_vec,1)
      a = a:expandAs(rot_axis)
      
      quat[{{},{1,3}}]:copy(rot_axis):cmul(a)
      quat[{{},4}] = rot_angle:clone():mul(0.5):cos()
   end
   
   return quat
end

-- DONE: multiple
-- DONE: make axis_angle a single Nx4 tensor like quat
function to_axis_angle(quat, axis_angle)

   axis_angle = axis_angle or quat:clone()
   axis_angle:resize(quat:size()):copy(quat)

   if (quat:nDimension() == 1) then 
      axis_angle[4] = torch.acos(quat[4]) * 2
      local d = math.sqrt(1 - quat[4]*quat[4])
      
      axis_angle[{{1,3}}]:mul(1/d)
   else
   
      axis_angle[{{},4}]:acos():mul(2)
      local rot_axis = axis_angle[{{},{1,3}}]
      
      local d = quat[{{},4}]:clone()
      d:cmul(d):mul(-1):add(1):sqrt()
      d = d:resize(d:size(1),1):expandAs(rot_axis)
      rot_axis:cdiv(d) 

   end
   return axis_angle
end

-- TODO: multiple 
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

-- TODO: multiple
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


-- Input: euler angle    = Nx2 or 3 : pitch, yaw and optional roll
-- Output:               = Nx4 quaternion
-- 
-- Local coordinates in our 2D image projections in x,y or phi
-- (vertical), lambda (horizontal) have 0,0 in the middle. Looking at
-- the 2D image in screen space: -,- is in the upper left hand corner,
-- and +,+ in the lower right hand corner.  This is the same way that
-- OpenGL presents the UV coordinates with -1,-1 in the upper left and
-- +1,+1 in the lower right.
-- 
-- In 3D coordinates we have settled on Z up, Y forward and X right.

-- To translate the euler angles of the spherical image projections to
-- angles in 3D coordinates it is useful to keep these bearings in mind:
-- 
-- Rotations : about (X,Y,Z axis)
-- 
-- + X : pitch (phi, elevation, latitude) pitch is rotation around the X
--   (right hand) axis (a quaternion has a large component in the x location)
-- + Y : roll rotation around the forward facing axis
-- + Z : yaw (lambda,azimuth, longitude) is a rotation around the Z upward axis
--   (quaternion is large in z location)

-- Direction of rotations :
-- 
-- (to stay consitent with image space projections)

-- + Pitch (south-ing): positive angle moves south 
--   (+y -> -z -> -y),(+z -> +y -> -z)

-- + Roll (clockwise) : positive angle moves clockwise around forward
--   facing direction (+x -> -z -> -x),(+z -> +x -> -z) 

-- + Yaw (east-ing): positive angle moves east 
--   (+y -> +x -> -y) (+x -> -y -> -x)
-- 
-- See: Appendix of O'Reilly Physics for Game Developers
-- 
-- DONE: multiple
function from_euler_angle(euler_angle, quat)
   if euler_angle:dim() == 1 then 
      euler_angle:resize(euler_angle:size(1),1)
   end

   local pitch   = euler_angle[1]  -- rotate about X axis
   local yaw     = euler_angle[2]  -- rotate about Z axis
   local roll                      -- rotate about Y axis
 
   if (euler_angle:size(1) == 3) then
      roll = euler_angle[3]
   else
      roll = torch.zeros(pitch:size())
   end

   -- for 3D manipulations quaternions are stored Nx4 
   quat = quat or torch.Tensor(pitch:nElement(),4)
   quat:resize(pitch:nElement(),4)

   local croll      =  roll:clone():mul(0.5):cos()
   local cyaw       =   yaw:clone():mul(0.5):cos()
   local cpitch     = pitch:clone():mul(0.5):cos()

   local sroll      =  roll:clone():mul(0.5):sin()
   local syaw       =   yaw:clone():mul(0.5):sin()
   local spitch     = pitch:clone():mul(0.5):sin()

   local cyaw_croll = torch.cmul(cyaw,croll)
   local syaw_sroll = torch.cmul(syaw,sroll)
   local cyaw_sroll = torch.cmul(cyaw,sroll)
   local syaw_croll = torch.cmul(syaw,croll)


   quat[{{},1}]:copy(cyaw_croll):cmul(spitch):add(-1,torch.cmul(syaw_sroll,cpitch))
   quat[{{},2}]:copy(cyaw_sroll):cmul(cpitch):add(torch.cmul(syaw_croll,spitch))
   quat[{{},3}]:copy(syaw_croll):cmul(cpitch):add(-1,torch.cmul(cyaw_sroll,spitch))
   quat[{{},4}]:copy(cyaw_croll):cmul(cpitch):add(torch.cmul(syaw_sroll,spitch))
   
   return quat
end

-- <quat> is Nx4
-- <euler_angles> is 3xN
-- See: help for from_euler_angle above
-- DONE: multiple
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

   local q00 = q_sqr[4]  
   local q11 = q_sqr[1]  
   local q22 = q_sqr[2]
   local q33 = q_sqr[3]
  
   local r11 = q00:clone():add(q11):add(-1,q22):add(-1,q33)
   local r21 = torch.cmul(q[1],q[2]):add(torch.cmul(q[4],q[3])):mul(2)

   -- absorbs the negative applied to all future uses
   local r31 = torch.cmul(q[1],q[3]):add(-1,torch.cmul(q[4],q[2])):mul(-2)
   local r32 = torch.cmul(q[2],q[3]):add(torch.cmul(q[4],q[1])):mul(2)
   local r33 = q00:clone():add(-1,q11):add(-1,q22):add(q33)

   local tmp = torch.abs(r31)
   local gimbal = tmp:gt(0.999999)
   local not_gimbal = gimbal:ne(1)
   
   if (not_gimbal:sum() > 0) then
      euler_angle[1][not_gimbal] = torch.atan2(r32[not_gimbal],r33[not_gimbal]) -- pitch
      euler_angle[2][not_gimbal] = torch.atan2(r21[not_gimbal],r11[not_gimbal]) -- yaw
      euler_angle[3][not_gimbal] = torch.asin(r31[not_gimbal])                  -- roll
   end

   if (gimbal:sum() > 0) then 
      local r12 = r21
      r12:copy(q[1]):cmul(q[2]):add(-1,torch.cmul(q[4],q[3])):mul(-2)
      local r13 = r32
      r13:copy(q[1]):cmul(q[3]):add(torch.cmul(q[4],q[2])):mul(2)
      r13:cmul(r31)
      
      euler_angle[1][gimbal] = 0                                    -- pitch
      euler_angle[2][gimbal] = torch.atan2(r12[gimbal],r13[gimbal]) -- yaw
      tmp[gimbal] = tmp[gimbal]:mul(math.pi/2)
      euler_angle[3][gimbal] = torch.cdiv(r31[gimbal],tmp[gimbal])  -- roll
   end

   -- make euler angles correspond to image projections (south-ing and east-ing)
   -- euler_angle[1]:mul(-1)
   -- euler_angle[2]:mul(-1)

   return euler_angle
end

-- TODO: multiple
-- if two quaternions are equal they will rotate a vector the same distance
function distance(quat1, quat2)
   local vec1 = rotate(quat1,axes)
   local vec2 = rotate(quat2,axes)
   return vec1:dist(vec2)
end


-- rotate vector by quaternion
-- this is an optimized version of 30 ops which we will move C
-- from http://physicsforgames.blogspot.com/2010/03/quaternion-tricks.html
-- DONE: multiple
function rotate(...)
   local res,q,v
   local args = {...}
   local nargs = #args
   if nargs == 3 then
      res = args[1]
      q   = args[2]
      v   = args[3] 
   elseif nargs == 2 then
      q   = args[1]
      v   = args[2]
      local n_elem = v:nElement() * (q:nElement() / 4)
      res = torch.Tensor(n_elem)
   else
      print(dok.usage('rotate_by_quat',
                      'rotate a vector by quaternion',
                      '> returns: rotated vector',
                      {type='torch.Tensor', help='result'},
                      {type='torch.Tensor', help='quaternion', req=true},
                      {type='torch.Tensor', help='vector',     req=true}))
      dok.error('incorrect arguments', 'rotate_by_quat')
   end
   -- call C function
   geom.rotation.by_quaternion(res,q,v)

   return res:squeeze()
end

-- rotate vector by quaternion and translate
-- this is an optimized version of 30 ops which we will move C
-- from http://physicsforgames.blogspot.com/2010/03/quaternion-tricks.html
-- DONE: multiple
function rotate_translate(...)
   local res,q,t,v
   local args = {...}
   local nargs = #args
   if nargs == 4 then
      res = args[1]
      q   = args[2]
      t   = args[3] 
      v   = args[4] 
   elseif nargs == 3 then
      q   = args[1]
      t   = args[2]
      v   = args[3]
      local n_elem = v:nElement() * (q:nElement() / 4)
      res = torch.Tensor(n_elem)
   else
      print(dok.usage('rotate_translate',
                      'rotate a vector by quaternion then translate',
                      '> returns: rotated vector',
                      {type='torch.Tensor', help='result'},
                      {type='torch.Tensor', help='quaternion', req=true},
                      {type='torch.Tensor', help='translate',  req=true},
                      {type='torch.Tensor', help='vector',     req=true}))
      dok.error('incorrect arguments', 'rotate_translate')
   end
   -- call C function
   geom.rotation.rotate_translate(res,q,t,v)

   return res:squeeze()
end

-- translate a vector then rotate by quaternion
-- this is an optimized version of 30 ops which we will move C
-- from http://physicsforgames.blogspot.com/2010/03/quaternion-tricks.html
-- DONE: multiple
function translate_rotate(...)
   local res,t,q,v
   local args = {...}
   local nargs = #args
   if nargs == 4 then
      res = args[1]
      t   = args[2]
      q   = args[3] 
      v   = args[4] 
   elseif nargs == 3 then
      t   = args[1]
      q   = args[2]
      v   = args[3]
      local n_elem = v:nElement() * (q:nElement() / 4)
      res = torch.Tensor(n_elem)
   else
      print(dok.usage('rotate_translate',
                      'rotate a vector by quaternion then translate',
                      '> returns: rotated vector',
                      {type='torch.Tensor', help='result'},
                      {type='torch.Tensor', help='translate',  req=true},
                      {type='torch.Tensor', help='quaternion', req=true},
                      {type='torch.Tensor', help='vector',     req=true}))
      dok.error('incorrect arguments', 'rotate_translate')
   end
   -- call C function
   geom.rotation.translate_rotate(res,t,q,v)

   return res:squeeze()
end
