Class()

local quat = geom.quaternion
local geom = geom.util


local data = require "geom.test.data"

function quaternion_from_to()
   print("Testing quaternion angle btw. 2 vectors")
   local e = 0
   local p = geom.normalized(data.vec[1])

   for i = 2,data.vec:size(1) do
      local v = geom.normalized(data.vec[i])
      local q = quat.angle_between(p,v)
      local d = q - data.result_vec_quat[i-1]
      if (d:abs():max() > 1e-6) then
         print(d,q,data.result_vec_quat[i-1])
         e = e + 1
      end
      p = v
   end
   print(string.format(" - Found %d/%d errors",e,data.vec:size(1)-1))
end

function quat2rot()
   print("Testing quaternion to rotation matrix")
   local e = 0
   for i = 1,data.quat:size(1) do
      local m = quat.to_rotation_matrix(data.quat[i])
      local d = m - data.result_rot_mat[i]
      if (d:abs():max() > 1e-6) then
         print(d,m,data.result_rot_mat[i])
         e = e + 1
      end
   end
   print(string.format(" - Found %d/%d errors",e,data.quat:size(1)))
end

function rot2quat()
   print("Testing rotation matrix to quaternion")
   local e = 0
   local maxerr = 0
   local res = torch.Tensor(data.vec:size(1)*data.quat:size(1),3)
   local i = 1
   for j = 1,data.quat:size(1) do
      local q = quat.from_rotation_matrix(data.result_rot_mat[j])
      -- cant test quat to quat so we test that they transform a
      -- vector in the same way.
      for k = 1,data.vec:size(1) do
         local v   = data.vec[k]
         quat.rotate(res[i],v,q)
         local d   = res[i] - data.result_rot_by_quat[i]
         local ce  = d:abs():max()
         if (ce > maxerr) then maxerr = ce end
         -- FIXME numerical error is getting really high.  Redo test
         -- suite with the highest precision possible.
         if (ce > 1e-2) then
            printf("error: %f",ce)
            printf("result (%f,%f,%f)", res[i][1],res[i][2],res[i][3])
            printf("should be: (%f,%f,%f)",
                   data.result_rot_by_quat[i][1],
                   data.result_rot_by_quat[i][2],
                   data.result_rot_by_quat[i][3])
            e = e + 1
         end
         i = i + 1
      
      end
   end
   print(string.format(" - Found %d/%d errors",e,res:size(1)))
end

function rotation_by_quat()
   print("Testing rotation with quaternion")
   local e = 0
   local maxerr = 0
   local res = torch.Tensor(data.vec:size(1)*data.quat:size(1),3)
   local i = 1
   sys.tic()
   for j = 1,data.quat:size(1) do
      local q = data.quat[j]
      for k = 1,data.vec:size(1) do
         local v   = data.vec[k]
         quat.rotate(res[i],v,q)
         local d   = res[i] - data.result_rot_by_quat[i]
         local ce  = d:abs():max()
         if (ce > maxerr) then maxerr = ce end
         if (ce > 1e-3) then
            printf("error: %f",ce)
            printf("result (%f,%f,%f)", res[i][1],res[i][2],res[i][3])
            printf("should be: (%f,%f,%f)",
                   data.result_rot_by_quat[i][1],
                   data.result_rot_by_quat[i][2],
                   data.result_rot_by_quat[i][3])
            e = e + 1
         end
         i = i + 1
      end
   end
   print(string.format(" - Found %d/%d errors (max: %e) in %2.4fs",
                       e,res:size(1),maxerr,sys.toc()))
end

function quat_product()
   print("Testing composition of rotations with quaternion")
   local e = 0
   local maxerr = 0
   sys.tic()
   local q1 = data.quat[1]
   for j = 2,data.quat:size(1) do
      local q2 = data.quat[j]
      for k = 1,data.vec:size(1) do
         local v    = data.vec[k]
         local vr1  = quat.rotate(v,q1)
         local vr2  = quat.rotate(vr1,q2)
         local q12  = quat.product(q2,q1)
         local vr12 = quat.rotate(v,q12)
         local ce = torch.max(torch.abs(vr12 - vr2))
         if (ce > maxerr) then maxerr = ce end
         if (ce > 5e-4) then
            print(ce,vr2,vr12)
            e = e + 1
         end
         q2 = q1
      end
   end
   print(string.format(" - Found %d/%d errors (max: %e) in %2.4fs",
                       e,data.quat:size(1)*data.vec:size(1),maxerr,sys.toc()))
end

function quat_from_to_euler()
   print("Testing conversion to euler (spherical angles) and back")
   local eps = 0.002 -- terrible numerical results with all the trig functions
   local e = 0
   local maxerr = 0
   sys.tic()
   local ea = quat.to_euler_angle(data.quat)
   local q  = quat.from_euler_angle(ea)
   for j = 1,data.quat:size(1) do
      local d = quat.distance(data.quat[j],q[j])
      if d > maxerr then maxerr = d end
      if d > eps then e = e + 1 end
   end
   -- test 2 edge cases
   local qe = torch.eye(4)
   ea = quat.to_euler_angle(qe)
   q  = quat.from_euler_angle(ea)
   for j = 1,4 do
      local d = quat.distance(qe[j],q[j])
      if d > maxerr then maxerr = d end
      if d > eps then e = e + 1 end
   end
   print(string.format(" - Found %d/%d errors (max: %e) in %2.4fs",
                       e,data.quat:size(1)+4,maxerr,sys.toc()))
end


function all()
   quaternion_from_to()
   quat2rot()
   rot2quat()
   rotation_by_quat()
   quat_product()
   quat_from_to_euler()
end
