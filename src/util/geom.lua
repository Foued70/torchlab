setfenv(1, setmetatable({}, {__index = _G}))

require 'torch'
require 'dok'

local axes   = torch.eye(3)

x_axis = axes[1]
y_axis = axes[2]
z_axis = axes[3]

local neg_axes   = torch.eye(3):mul(-1)

function eq(vec1, vec2)
   return torch.min(torch.eq(vec1,vec2)) == 1
end

function dist(vec1, vec2)
   return (vec1 - vec2):norm()
end

function direction(vec1, vec2)
   return normalize(vec1 - vec2)
end

function normalize(vec)
   return torch.div(vec, vec, vec:norm())
end

function normalized(...)
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
      print(dok.usage('normalize',
                      'normalize a vector (L2)',
                      '> returns: normalized vector',
                      {type='torch.Tensor', help='output'},
                      {type='torch.Tensor', help='input', req=true}))
      dok.error('incorrect arguments', 'normalize')
   end

   res:copy(v)
   local n = res:norm()
   if (n > 1e-8) then
      res:mul(1/n)
   end
   return res
end

-- compute normal from first three vertices in a face
function compute_normal(v)
   return normalize(torch.cross(v[3] - v[2], v[1] - v[2]))
end

function angle_between(vec1, vec2)
   return torch.acos(torch.dot(vec1, vec2) / (vec1:norm()*vec2:norm()))
end

function axis_rotation(normal,d)
   local n   = normalized(normal:narrow(1,1,3))
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

function rotation_matrix_to_quaternion (rmat, quat)
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
      error("Bad input perhaps not a rotation matrix?")
   end

   qsign:sign() -- convert to just +1,-1
   -- put signs into quaternion
   quat:cmul(qsign)

   normalize(quat)

   return quat
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

   --avoid the degenerate case when from_vector is very close to to_vector
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

-- FIXME make tests as this function seems to invert results for
-- simple axis-aligned plane intersections

function ray_plane_intersection(...)
   local pt,dir,plane_norm,plane_d,debug
   local args  = {...}
   local nargs = #args
   if nargs == 5 then
      debug    = args[5]
   end
   if nargs < 4 then
      print(dok.usage('ray_plane_intersection',
                      'does ray intersect the plane',
                      '> returns: intersection,distance or nil,errno',
                      {type='torch.Tensor', help='point (start of ray)'},
                      {type='torch.Tensor', help='direction (of ray)', req=true},
                      {type='torch.Tensor', help='normal (of plane)', req=true},
                      {type='torch.Tensor', help='offset (of plane)', req=true},
                      {type='torch.Tensor', help='debug', default=False}))

      dok.error('incorrect arguements', 'ray_plane_intersection')
   else
      pt         = args[1]
      dir        = args[2]
      plane_norm = args[3]
      plane_d    = args[4]
   end
   local a = torch.dot(plane_norm,dir)
   if torch.abs(a) < 1e-8 then
      if debug then print(" - angle parallel") end
      return nil,1
   end
   local t = -(torch.dot(plane_norm,pt) + plane_d)/a
   if debug then print(string.format(" - t: %f", t)) end
   if t < 0 then
      if debug then print(" - plane behind") end
      return nil,2
   end
   local i = pt + dir * t
   if debug then print(string.format(" - int: [%f, %f, %f]",
                                     i[1],i[2],i[3])) end
   return i,t
end


function ray_face_intersection(...)
   local pt,dir,plane_norm,plane_d,face_verts,debug
   local args  = {...}
   local nargs = #args
   if nargs == 6 then
      debug    = args[6]
   end
   if nargs < 4 then
      print(dok.usage('ray_face_intersection',
                      'does ray intersect the face',
                      '> returns: intersection point,distance, or nil,errno',
                      {type='torch.Tensor', help='point (start of ray)'},
                      {type='torch.Tensor', help='direction (of ray)', req=true},
                      {type='torch.Tensor', help='normal (of plane)', req=true},
                      {type='torch.Tensor', help='offset (of plane)', req=true},
                      {type='torch.Tensor', help='face vertices', req=true},
                      {type='torch.Tensor', help='debug', default=False}))

      dok.error('incorrect arguements', 'ray_plane_intersection')
   else
      pt         = args[1]
      dir        = args[2]
      plane_norm = args[3]
      plane_d    = args[4]
      face_verts = args[5]
   end
   -- First find planar intersection
   local intersection,t =
      ray_plane_intersection(pt,dir,plane_norm,plane_d,debug)
   if not intersection then
      return nil,t
   end
   -- pick two most planar dimensions of the face throw away
   -- coordinate with greatest magnitude (if normal is mostly z
   -- then we want x and y)
   local nverts = face_verts:size(1)
   local _,ds = torch.sort(torch.abs(plane_norm))
   local ri = torch.Tensor(2)
   local verts = torch.Tensor(nverts,2)
   ri[1] = intersection[ds[1]]
   ri[2] = intersection[ds[2]]
   for i = 1,nverts do
      verts[i][1] = face_verts[i][ds[1]] - ri[1]
      verts[i][2] = face_verts[i][ds[2]] - ri[2]
   end
   -- count crossings along 'y' axis : b in slope intercept line equation
   local pvert = verts[nverts]
   local count = 0
   for vi = 1,nverts do
      local cvert = verts[vi]
      --  compute y axis crossing (b = y - mx)
      local run  =  cvert[1] - pvert[1]
      local b    = -math.huge
      local cpos = 1
      local ppos = 1
      if math.abs(run) < 1e-8 then
         if (math.abs(cvert[1]) < 1e-8) then
            count = count + 1
         end
      else
         b = cvert[2] - ((cvert[2] - pvert[2])/run) * cvert[1]
         if (cvert[1] < 0) then cpos = -1 end
         if (pvert[1] < 0) then ppos = -1 end
         if (b >= 0) and ((cpos + ppos) == 0) then
            count = count + 1
         end
      end
      pvert = cvert
   end
   if ((count > 0) and (count % 2)) then
      return intersection,t
   else
      return nil,3
   end
end


return (getfenv())

