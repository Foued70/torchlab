require 'torch'
require 'dok'

util.geom = {}
local geom = util.geom

local axes = torch.Tensor(3,3)
for i = 1,3 do
   axes[i][i] = 1
end

local x_axis = axes[1]
local y_axis = axes[2]
local z_axis = axes[3]

function geom.normalize(...)
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
function geom.compute_normal(v)
   return geom.normalize(
      torch.cross(v[3] - v[2], v[1] - v[2]))
end

function geom.axis_rotation(normal,d)
   local n   = geom.normalize(normal:narrow(1,1,3))
   return geom.quaternion_angle(n,d)
end

function geom.x_rotation(normal)
   return geom.axis_rotation(normal,z_axis)
end

function geom.y_rotation(normal)
   return geom.axis_rotation(normal,z_axis)
end

function geom.z_rotation(normal)
   return geom.axis_rotation(normal,z_axis)
end

function geom.largest_rotation (normal)
   local n   = normal:narrow(1,1,3)
   local p   = torch.Tensor(3):copy(n):abs()
   local v,i = p:sort()
   local d   = i[#i]
   local a   = axes[d]
   if (n[d] < 0) then
      a = a * -1
   end
   return geom.axis_rotation(n,a),d
end

function geom.quat_conjugate(quat,res)
   if (not res) then
      res = torch.Tensor(quat:size()):copy(quat)
   end
   res:narrow(1,1,3):mul(-1)
   return res
end

function geom.rotation_matrix(quaternion, res)
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
function geom.rotate_by_quat(...)
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
      res = torch.Tensor(4)
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
   res[4] = 0
   return res
end

-- rotate vector by rotation matrix
function geom.rotate_by_mat(...)
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
      res = torch.Tensor(4)
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
   return res
end

-- returns quaternion represnting angle between two vectors
function geom.quaternion_angle(...)
   local quat, from_vector, to_vector
   local args = {...}
   local nargs = #args
   if nargs == 3 then
      quat = args[1]
      from_vector = args[2]:narrow(1,1,3)
      to_vector   = args[3]:narrow(1,1,3)
   elseif nargs == 2 then
      from_vector = args[1]:narrow(1,1,3)
      to_vector   = args[2]:narrow(1,1,3)
      quat = torch.Tensor(4):fill(0)
      quat[4] = 1
   else
      print(dok.usage('quaternion_angle',
                      'compute rotation quaternion from <from_vector> to <to_vector>', 
                      '> returns: quaternion',
                      {type='torch.Tensor', help='quaternion'},
                      {type='torch.Tensor', help='from_vector', req=true},
                      {type='torch.Tensor', help='to_vector',   req=true}))
      dok.error('incorrect arguements', 'quaternion_angle')
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

function geom.ray_plane_intersection(...)
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
      

function geom.ray_face_intersection(...)
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
      geom.ray_plane_intersection(pt,dir,plane_norm,plane_d,debug)
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

