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

function quaternion_to_rotation_matrix()
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

function rotation_matrix_to_quaternion()
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
         quat.rotate(res[i],q,v)
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
   log.tic()
   for j = 1,data.quat:size(1) do
      local q = data.quat[j]
      for k = 1,data.vec:size(1) do
         local v   = data.vec[k]
         quat.rotate(res[i],q,v)
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
                       e,res:size(1),maxerr,log.toc()))
end

function quat_product()
   print("Testing composition of rotations with quaternion product")
   local e = 0
   local maxerr = 0
   local n_test = 0
   log.tic()

   local dq = data.quat
   local dv = data.vec
   local vr1  = quat.rotate(dq,dv)
   local vr2  = quat.rotate(dq,vr1)
   for i = 1,dq:size(1) do
      local q1 = dq[i]
      for j = 1,dq:size(1) do
         local q2   = dq[j]
         local q12  = quat.product(q2,q1)

         local vr12 = quat.rotate(q12,dv)
         local ce = torch.abs(vr12 - vr2[j][i])
         local mx = torch.max(ce)
         if ( mx > maxerr) then maxerr = mx end
         e = e + ce:gt(7e-4):sum()
         n_test = n_test + dv:size(1)
      end
   end

   -- test with conjugate also
   local qr = torch.Tensor(data.quat:size())
   for i = 1,data.quat:size(1) do
      local q1 = data.quat[i]
      for j = 1,data.quat:size(1) do
         local q2 = data.quat[j]
         quat.product(q1,q2,qr[j])
         local qc = quat.product(qr[j],quat.conjugate(q2))
         local ce = torch.max(torch.abs(qc - q1))
         if (ce > maxerr) then maxerr = ce end
         if (ce > 5e-4) then
            print(ce,q1,qc)
            e = e + 1
         end
         q2 = q1
         n_test = n_test + 1
      end
      -- test matrix version
      local qmat = quat.product(q1,data.quat)
      for j = 1,data.quat:size(1) do
         local ce = torch.max(torch.abs(qr[j] - qmat[j]))
         if (ce > maxerr) then maxerr = ce end
         if (ce > 5e-4) then
            print(ce,qr[j],qmat[j])
            e = e + 1
         end
         n_test = n_test + 1
      end
   end
   print(string.format(" - Found %d/%d errors (max: %e) in %2.4fs",
                       e,n_test,maxerr,log.toc()))
end

function quat_from_to_euler()
   print("Testing conversion to euler (spherical angles) and back")
   local eps = 0.002 -- terrible numerical results with all the trig functions
   local e = 0
   local maxerr = 0
   log.tic()
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
                       e,data.quat:size(1)+4,maxerr,log.toc()))
end


function euler_to_quat_directions()
   print("Testing directions in euler to quat conversions")
   log.tic()
   -- step = 25
   -- angles = torch.linspace(-2*math.pi,2*math.pi,2*step+1)
   local angles = data.euler_angles
   local n_angles = data.euler_angles:size(1)
   local ea = torch.zeros(3,n_angles)

   local e = 0

   local mid = math.ceil(n_angles/2)

   local axes = torch.eye(3)
   print("   rotate about quaternion axis is equivalent to euler angle")
   print(" - quaternion X (Right)   <=> euler pitch (phi, elevation,latitude)")

   ea[1] = angles
   local q     = quat.from_euler_angle(ea)
   local v     = quat.rotate(q,axes)

   local x = v[{{},1,{}}]
   local y = v[{{},2,{}}]
   local z = v[{{},3,{}}]

   if not (x:sum() == n_angles) then e = e + 1 end
   if not (x[{{},1}]:sum() == n_angles) then e = e + 1 end
   if not (q[{{},2}]:sum() == 0) then e = e + 1 end
   if not (q[{{},3}]:sum() == 0) then e = e + 1 end
   if not (torch.abs(v[1]   - axes):max() < 1e-15) then e = e + 1 end
   if not (torch.abs(v[mid] - axes):max() < 1e-15) then e = e + 1 end
   if not (torch.abs(v[-1]  - axes):max() < 1e-15) then e = e + 1 end
   if not (torch.abs(v - data.result_axes_about_x):max() < 1e-15) then e = e + 1 end

   print(" - quaternion Y (Forward) <=> euler roll")
   ea[1]:fill(0)
   ea[3] = angles
   quat.from_euler_angle(ea,q)
   quat.rotate(v,q,axes)

   if not (y:sum() == n_angles) then e = e + 1 end
   if not (y[{{},2}]:sum() == n_angles) then e = e + 1 end
   if not (q[{{},1}]:sum() == 0) then e = e + 1 end
   if not (q[{{},3}]:sum() == 0) then e = e + 1 end
   if not (torch.abs(v[1]   - axes):max() < 1e-15) then e = e + 1 end
   if not (torch.abs(v[mid] - axes):max() < 1e-15) then e = e + 1 end
   if not (torch.abs(v[-1]  - axes):max() < 1e-15) then e = e + 1 end
   if not (torch.abs(v - data.result_axes_about_y):max() < 1e-15) then e = e + 1 end

   print(" - quaternion Z (Up)      <=> euler yaw (lambda,azimuth, longitude)")
   ea[3]:fill(0)
   ea[2] = angles
   quat.from_euler_angle(ea,q)
   quat.rotate(v,q,axes)

   if not (z:sum() == n_angles) then e = e + 1 end
   if not (z[{{},3}]:sum() == n_angles) then e = e + 1 end
   if not (q[{{},1}]:sum() == 0) then e = e + 1 end
   if not (q[{{},2}]:sum() == 0) then e = e + 1 end
   if not (torch.abs(v[1]   - axes):max() < 1e-15) then e = e + 1 end
   if not (torch.abs(v[mid] - axes):max() < 1e-15) then e = e + 1 end
   if not (torch.abs(v[-1]  - axes):max() < 1e-15) then e = e + 1 end
   if not (torch.abs(v - data.result_axes_about_z):max() < 1e-15) then e = e + 1 end

   printf(" - Passed %d/%d tests in %2.4fs", e, 8*3, log.toc())
end

function all()
   quaternion_from_to()
   quaternion_to_rotation_matrix()
   rotation_matrix_to_quaternion()
   rotation_by_quat()
   quat_product()
   quat_from_to_euler()
   euler_to_quat_directions()
end
