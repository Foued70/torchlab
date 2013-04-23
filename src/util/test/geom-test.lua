require 'torch'
require 'sys'

local geom = util.geom

local test = {}
test.data = require "util.test.data.geom-data"

function test.quaternion_from_to()
   print("Testing quaternion angle btw. 2 vectors")
   local e = 0
   local p = geom.normalized(test.data.vec[1])

   for i = 2,test.data.vec:size(1) do
      local v = geom.normalized(test.data.vec[i])
      local q = geom.quaternion_from_to(p,v)
      local d = q - test.data.result_vec_quat[i-1]
      if (d:abs():max() > 1e-6) then
         print(d,q,test.data.result_vec_quat[i-1])
         e = e + 1
      end
      p = v
   end
   print(string.format(" - Found %d/%d errors",e,test.data.vec:size(1)-1))
end

function test.quat2rot()
   print("Testing quaternion to rotation matrix")
   local e = 0
   for i = 1,test.data.quat:size(1) do
      local m = geom.quaternion_to_rotation_matrix(test.data.quat[i])
      local d = m - test.data.result_rot_mat[i]
      if (d:abs():max() > 1e-6) then
         print(d,m,test.data.result_rot_mat[i])
         e = e + 1
      end
   end
   print(string.format(" - Found %d/%d errors",e,test.data.quat:size(1)))
end

function test.rot2quat()
   print("Testing rotation matrix to quaternion")
   local e = 0
   local maxerr = 0
   local res = torch.Tensor(test.data.vec:size(1)*test.data.quat:size(1),3)
   local i = 1
   for j = 1,test.data.quat:size(1) do
      local q = geom.rotation_matrix_to_quaternion(test.data.result_rot_mat[j])
      -- cant test quat to quat so we test that they transform a
      -- vector in the same way.
      for k = 1,test.data.vec:size(1) do
         local v   = test.data.vec[k]
         geom.rotate_by_quat(res[i],v,q)
         local d   = res[i] - test.data.result_rot_by_quat[i]
         local ce  = d:abs():max()
         if (ce > maxerr) then maxerr = ce end
         -- FIXME numerical error is getting really high.  Redo test
         -- suite with the highest precision possible.
         if (ce > 1e-2) then
            printf("error: %f",ce)
            printf("result (%f,%f,%f)", res[i][1],res[i][2],res[i][3])
            printf("should be: (%f,%f,%f)",
                   test.data.result_rot_by_quat[i][1],
                   test.data.result_rot_by_quat[i][2],
                   test.data.result_rot_by_quat[i][3])
            e = e + 1
         end
         i = i + 1
      
      end
   end
   print(string.format(" - Found %d/%d errors",e,res:size(1)))
end

function test.rotation_by_quat()
   print("Testing rotation with quaternion")
   local e = 0
   local maxerr = 0
   local res = torch.Tensor(test.data.vec:size(1)*test.data.quat:size(1),3)
   local i = 1
   sys.tic()
   for j = 1,test.data.quat:size(1) do
      local q = test.data.quat[j]
      for k = 1,test.data.vec:size(1) do
         local v   = test.data.vec[k]
         geom.rotate_by_quat(res[i],v,q)
         local d   = res[i] - test.data.result_rot_by_quat[i]
         local ce  = d:abs():max()
         if (ce > maxerr) then maxerr = ce end
         if (ce > 1e-3) then
            printf("error: %f",ce)
            printf("result (%f,%f,%f)", res[i][1],res[i][2],res[i][3])
            printf("should be: (%f,%f,%f)",
                   test.data.result_rot_by_quat[i][1],
                   test.data.result_rot_by_quat[i][2],
                   test.data.result_rot_by_quat[i][3])
            e = e + 1
         end
         i = i + 1
      end
   end
   print(string.format(" - Found %d/%d errors (max: %e) in %2.4fs",
                       e,res:size(1),maxerr,sys.toc()))
end

function test.rotation_by_quatC()
   print("Testing rotation with quaternion (C version)")
   local e      = 0
   local maxerr = 0
   local vecs   = test.data.vec
   local nvecs  = vecs:size(1)
   local quats  = test.data.quat
   local nquats = quats:size(1)
   local res    = torch.Tensor(nvecs*nquats,3)
   local groundt = test.data.result_rot_by_quat
   local i      = 1
   local offset = 1
   sys.tic()
   for i = 1,nquats do
      local q   = quats[i]
      local out = res:narrow(1,offset,nvecs)
      local gt  = groundt:narrow(1,offset,nvecs) 
      offset = offset + nvecs

      util.geom.rotate_by_quatC(out,vecs,q)

      local d = out - gt
      local ce  = d:abs():max()
      if (ce > maxerr) then maxerr = ce end
      if (ce > 1e-3) then
         printf("error: %f",ce)
         printf("result (%f,%f,%f)", res[i][1],res[i][2],res[i][3])
         printf("should be: (%f,%f,%f)",
                test.data.result_rot_by_quat[i][1],
                test.data.result_rot_by_quat[i][2],
                test.data.result_rot_by_quat[i][3])
         e = e + 1
      end 
   end
   print(string.format(" - Found %d/%d errors (max: %e) in %2.4fs",
                       e,res:size(1),maxerr,sys.toc()))
end

function test.rotate_translateC()
   print("Testing rotation with quaternion then translation (C version)")
   local e      = 0
   local maxerr = 0
   local vecs   = test.data.vec
   local nvecs  = vecs:size(1)
   local quats  = test.data.quat
   local nquats = quats:size(1)
   local res    = torch.Tensor(nvecs*nquats,3)
   local groundt = res:clone()
   local i      = 1
   local offset = 1

   sys.tic()
   for j = 1,nquats do
      local q = quats[j]
      for k = 1,nvecs do
         groundt[i] = vecs[j] + geom.rotate_by_quat(vecs[k],q)
         i = i + 1
      end
   end
   print(string.format(" - built ground truth in %2.4fs", sys.toc()))

   sys.tic()
   for i = 1,nquats do
      local q   = quats[i]
      local out = res:narrow(1,offset,nvecs)
      local gt  = groundt:narrow(1,offset,nvecs) 
      offset = offset + nvecs

      util.geom.rotate_translateC(out,vecs,vecs[i],q)

      local d = out - gt
      local ce  = d:abs():max()
      if (ce > maxerr) then maxerr = ce end
      if (ce > 1e-3) then
         printf("error: %f",ce)
         printf("result (%f,%f,%f)", res[i][1],res[i][2],res[i][3])
         printf("should be: (%f,%f,%f)",
                test.data.result_rot_by_quat[i][1],
                test.data.result_rot_by_quat[i][2],
                test.data.result_rot_by_quat[i][3])
         e = e + 1
      end 
   end
   print(string.format(" - Found %d/%d errors (max: %e) in %2.4fs",
                       e,res:size(1),maxerr,sys.toc()))
end

function test.translate_rotateC()
   print("Testing translation then rotation with quaternion (C version)")
   local e      = 0
   local maxerr = 0
   local vecs   = test.data.vec
   local nvecs  = vecs:size(1)
   local quats  = test.data.quat
   local nquats = quats:size(1)
   local res    = torch.Tensor(nvecs*nquats,3)
   local i      = 1
   local offset = 1
   local groundt = res:clone()

   sys.tic()
   for j = 1,nquats do
      local q = quats[j]
      for k = 1,nvecs do
         local v   = vecs[k] + vecs[j]
         geom.rotate_by_quat(groundt[i],v,q)
         i = i + 1
      end
   end
   print(string.format(" - built ground truth in %2.4fs", sys.toc()))

   sys.tic()
   for i = 1,nquats do
      local q   = quats[i]
      local out = res:narrow(1,offset,nvecs)
      local gt  = groundt:narrow(1,offset,nvecs) 
      offset = offset + nvecs

      util.geom.translate_rotateC(out,vecs,vecs[i],q)

      local d = out - gt
      local ce  = d:abs():max()
      if (ce > maxerr) then maxerr = ce end
      if (ce > 1e-3) then
         printf("error: %f",ce)
         printf("result (%f,%f,%f)", res[i][1],res[i][2],res[i][3])
         printf("should be: (%f,%f,%f)",
                test.data.result_rot_by_quat[i][1],
                test.data.result_rot_by_quat[i][2],
                test.data.result_rot_by_quat[i][3])
         e = e + 1
      end 
   end
   print(string.format(" - Found %d/%d errors (max: %e) in %2.4fs",
                       e,res:size(1),maxerr,sys.toc()))
end

function test.quat_product()
   print("Testing composition of rotations with quaternion")
   local e = 0
   local maxerr = 0
   sys.tic()
   local q1 = test.data.quat[1]
   for j = 2,test.data.quat:size(1) do
      local q2 = test.data.quat[j]
      for k = 1,test.data.vec:size(1) do
         local v    = test.data.vec[k]
         local vr1  = geom.rotate_by_quat(v,q1)
         local vr2  = geom.rotate_by_quat(vr1,q2)
         local q12  = geom.quat_product(q2,q1)
         local vr12 = geom.rotate_by_quat(v,q12)
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
                       e,test.data.quat:size(1)*test.data.vec:size(1),maxerr,sys.toc()))
end

function test.rotation_by_mat()
   print("Testing rotation w/ rotation matrix")
   local e = 0
   local maxerr = 0
   local res = torch.Tensor(test.data.vec:size(1)*test.data.quat:size(1),3)
   local i = 1
   sys.tic()
   for j = 1,test.data.quat:size(1) do
      local mat = test.data.result_rot_mat[j]
      for k = 1,test.data.vec:size(1) do
         local v = test.data.vec[k]
         geom.rotate_by_mat(res[i],v,mat)
         local d = res[i] - test.data.result_rot_by_quat[i]
         local ce = d:abs():max()
         if (ce > maxerr) then maxerr = ce end
         if (ce > 1e-3) then
            print(ce,res[i],test.data.result_rot_by_quat[i])
            e = e + 1
         end
         i = i + 1
      end
   end
   print(string.format(" - Found %d/%d errors (max: %e) in %2.4fs",
                       e,res:size(1),maxerr,sys.toc()))
end

function test.z_rotation()
   print("Testing z-rotation")
   local e = 0
   local maxerr = 0
   sys.tic()
   for i = 1,test.data.vec:size(1) do
      local tvec = test.data.vec[i]
      local quat = geom.z_rotation(tvec)
      local rvec = geom.rotate_by_quat(tvec,quat)
      local ce = rvec[3] - rvec:sum()
      if (ce > maxerr) then maxerr = ce end
      if (ce > 1e-8) then
         print(tvec,rvec)
         e = e + 1
      end
   end
   print(string.format(" - Found %d/%d errors (max: %e) in %2.4fs",
                       e,test.data.vec:size(1),maxerr,sys.toc()))
end

function test.x_rotation()
   print("Testing x-rotation")
   local e = 0
   local maxerr = 0
   sys.tic()
   for i = 1,test.data.vec:size(1) do
      local tvec = test.data.vec[i]
      local quat = geom.z_rotation(tvec)
      local rvec = geom.rotate_by_quat(tvec,quat)
      local ce = rvec[1] - rvec:sum()
      if (ce > maxerr) then maxerr = ce end
      if (ce > 1e-8) then
         print(tvec,rvec)
         e = e + 1
      end
   end
   print(string.format(" - Found %d/%d errors (max: %e) in %2.4fs",
                       e,test.data.vec:size(1),maxerr,sys.toc()))
end

function test.y_rotation()
   print("Testing y-rotation")
   local e = 0
   local maxerr = 0
   sys.tic()
   for i = 1,test.data.vec:size(1) do
      local tvec = test.data.vec[i]
      local quat = geom.z_rotation(tvec)
      local rvec = geom.rotate_by_quat(tvec,quat)
      local ce = rvec[2] - rvec:sum()
      if (ce > maxerr) then maxerr = ce end
      if (ce > 1e-8) then
         print(tvec,rvec)
         e = e + 1
      end
   end
   print(string.format(" - Found %d/%d errors (max: %e) in %2.4fs",
                       e,test.data.vec:size(1),maxerr,sys.toc()))
end

function test.largest_rotation()
   print("Testing largest rotation")
   local e = 0
   local maxerr = 0
   sys.tic()
   for i = 1,test.data.vec:size(1) do
      local tvec = test.data.vec[i]
      local quat,d = geom.largest_rotation(tvec)
      local rvec   = geom.rotate_by_quat(tvec,quat)
      local ce = rvec[d] - rvec:sum()
      if (ce > maxerr) then maxerr = ce end
      if (ce > 1e-8) then
         print(tvec,rvec)
         e = e + 1
      end
   end
   print(string.format(" - Found %d/%d errors (max: %e) in %2.4fs",
                       e,test.data.vec:size(1),maxerr,sys.toc()))
end

function test.ray_plane_intersection()
   print("Testing ray plane intersection")
   local e = 0
   local cnt = 0
   local results = test.data.result_ray_plane
   local plane_norm = test.data.quat:narrow(2,1,3)
   local plane_d = test.data.quat:select(2,4)
   local maxterr = 0
   local maxierr = 0
   sys.tic()
   for i = 1,test.data.vec:size(1) do
      local tvec = test.data.vec[i]:narrow(1,1,3)
      for j = 1,plane_norm:size(1) do
         local n = geom.normalize(plane_norm[j])
         local d = plane_d[j]
         local intersection,t =
            geom.ray_plane_intersection(tvec,tvec,n,d,false)
         cnt = cnt + 1
         local gt = results[cnt]
         local terr = torch.abs(t - gt[1])
         if not intersection then
            if  terr > 1 then
               e = e + 1
            end
         else
            ierr = torch.abs(torch.sum(torch.add(intersection,-gt:narrow(1,2,3))))/3
            if terr > 1e-8 or ierr > 1e-8 then
               if terr > maxterr then maxterr = terr end
               if ierr > maxierr then maxierr = ierr end
               e = e + 1
               print(string.format("[v:%d][p:%d]", i, j))
               print(string.format("T: %f <-> %f err: %e I: (%f,%f,%f) <-> (%f,%f,%f) err: %e",
                                   t, gt[1], terr,
                                   intersection[1],intersection[2],intersection[3],
                                   gt[2],gt[3],gt[4], ierr))
            end
         end
      end
   end
   print(string.format(" - Found %d/%d errors (maxT: %e, maxI: %e) in %2.4fs",
                       e,cnt, maxterr,maxierr,sys.toc()))
end

function test.ray_face_intersection()
   print("Testing ray face intersection")
   local e          = 0
   local cnt        = 0
   local maxterr    = 0
   local maxierr    = 0
   local results    = test.data.result_ray_face
   local pts        = test.data.pt_dir:narrow(2,1,3)
   local dirs       = test.data.pt_dir:narrow(2,4,3)
   local face_plane = test.data.face_plane
   local face_verts = test.data.face_verts
   local plane_d    = test.data.quat:select(2,4)
   sys.tic()
   for i = 1,pts:size(1) do
      local pt  = pts[i]
      local dir = dirs[i]
      for j = 1,face_plane:size(1) do
         local n = face_plane[j]:narrow(1,1,3)
         local d = face_plane[j][4]
         local v = face_verts[j]
         local intersection,t =
            geom.ray_face_intersection(pt,dir,n,d,v,false)
         cnt = cnt + 1
         local gt = results[i][j]
         if intersection then
            if (gt:sum() == 0) then
               e = e + 1
            else
               local terr = torch.max(torch.abs(intersection - gt))
               if terr > 1e-7    then
                  e = e + 1
               end
               if terr > maxterr then maxterr = terr end
            end
         else
            if torch.sum(torch.abs(gt)) > 0 then
               e = e + 1
            end
         end
      end
   end
   print(string.format(" - Found %d/%d errors (max: %e) in %2.4fs",
                       e,cnt, maxterr,sys.toc()))
end

function test.compute_normals()
   print("Testing compute normals")
   local e          = 0
   local cnt        = 0
   local maxerr     = 0
   local normals    = test.data.face_plane:narrow(2,1,3)
   local face_verts = test.data.face_verts
   sys.tic()
   for i = 1,normals:size(1) do
      cnt = cnt + 1
      local n = geom.compute_normal(face_verts[i])
      local gtn = normals[i]
      local err = torch.sum(torch.abs(n - gtn))
      if err > maxerr then maxerr = err end
      if err > 1e-7 then
         e = e + 1
         print(string.format("%e : (%e,%e,%e) <-> (%e,%e,%e)",
                             err, n[1],n[2],n[3],
                             gtn[1],gtn[2],gtn[3]))
      end
   end
   print(string.format(" - Found %d/%d errors (max: %e) in %2.4fs",
                       e,cnt, maxerr,sys.toc()))
end

function test.spherical_coords_to_unit_cartesian()
   print("Testing spherical_coords_to_unit_cartesian")
   local pi  = math.pi
   local pi2 = math.pi / 2
   local pi4 = math.pi / 4
   local sp4 = math.sin(pi4)
   local angles = torch.Tensor(
      {
         {   0,    0}, -- right:  0 forward: 1 up:  0 
         { pi2,    0}, -- right:  1 forward: 0 up:  0 
         {-pi2,    0}, -- right: -1 forward: 0 up:  0 
         {   0,  pi2}, -- right:  0 forward: 0 up:  1 
         {   0, -pi2}, -- right:  0 forward: 0 up: -1 
         { pi2,  pi2}, -- right:  0 forward: 0 up:  1 
         {-pi2, -pi2}, -- right:  0 forward: 0 up: -1 
         { pi2, -pi2}, -- right:  0 forward: 0 up: -1 
         {-pi2,  pi2}, -- right:  0 forward: 0 up:  1 
         {  pi,   pi}, -- right:  0 forward: 1 up:  0 
         { pi2,   pi}, -- right: -1 forward: 0 up:  0 
         {-pi2,   pi}, -- right:  1 forward: 0 up:  0 
         {  pi,  pi2}, -- right:  0 forward: 0 up:  1 
         {  pi, -pi2}, -- right:  0 forward: 0 up: -1 
      })
   
   local unit = geom.spherical_coords_to_unit_cartesian(angles)
   
   local unit_check = torch.Tensor(
      {
         { 0,1, 0}, --    0,  0
         { 1,0, 0}, --  pi2,  0
         {-1,0, 0}, -- -pi2,  0
         { 0,0, 1}, --    0,  pi2
         { 0,0,-1}, --    0, -pi2
         { 0,0, 1}, --  pi2,  pi2
         { 0,0,-1}, -- -pi2, -pi2
         { 0,0,-1}, --  pi2, -pi2
         { 0,0, 1}, -- -pi2,  pi2
         { 0,1, 0}, --   pi,  pi
         {-1,0, 0}, --  pi2,  pi
         { 1,0, 0}, -- -pi2,  pi
         { 0,0, 1}, --   pi,  pi2
         { 0,0,-1}, --   pi, -pi2
      })
   
   local error = unit:dist(unit_check,1)
   if error < 1e-8 then error = 0 end
   print(string.format(" - Found %d errors", error)) 
end

function test.unit_cartesian_to_spherical_coords()
   print("Testing unit_cartesian_to_spherical_coords")
   local pi  = math.pi
   local pi2 = math.pi / 2
   local pi4 = math.pi / 4
   local sp4 = math.sin(pi4)
   
   local unit_cartesian = torch.Tensor(
      {
         { 0,1, 0}, --    0,  0
         { 1,0, 0}, --  pi2,  0
         {-1,0, 0}, -- -pi2,  0
         { 0,0, 1}, --    0,  pi2
         { 0,0,-1}, --    0, -pi2
         { 0,0, 1}, --  pi2,  pi2
         { 0,0,-1}, -- -pi2, -pi2
         { 0,0,-1}, --  pi2, -pi2
         { 0,0, 1}, -- -pi2,  pi2
         { 0,1, 0}, --   pi,  pi
         {-1,0, 0}, --  pi2,  pi
         { 1,0, 0}, -- -pi2,  pi
         { 0,0, 1}, --   pi,  pi2
         { 0,0,-1}, --   pi, -pi2
      })

   local angles = geom.unit_cartesian_to_spherical_coords(unit_cartesian)

   local angles_check = torch.Tensor(
      {
         {   0,    0}, -- right:  0 forward: 1 up:  0 
         { pi2,    0}, -- right:  1 forward: 0 up:  0 
         {-pi2,    0}, -- right: -1 forward: 0 up:  0 
         {   0,  pi2}, -- right:  0 forward: 0 up:  1 
         {   0, -pi2}, -- right:  0 forward: 0 up: -1 
         { pi2,  pi2}, -- right:  0 forward: 0 up:  1 
         {-pi2, -pi2}, -- right:  0 forward: 0 up: -1 
         { pi2, -pi2}, -- right:  0 forward: 0 up: -1 
         {-pi2,  pi2}, -- right:  0 forward: 0 up:  1 
         {  pi,   pi}, -- right:  0 forward: 1 up:  0 
         { pi2,   pi}, -- right: -1 forward: 0 up:  0 
         {-pi2,   pi}, -- right:  1 forward: 0 up:  0 
         {  pi,  pi2}, -- right:  0 forward: 0 up:  1 
         {  pi, -pi2}, -- right:  0 forward: 0 up: -1 
      })
   
   
   local error = 0 
 
   for i = 1,angles:size(1) do 
      local d = angles[i]:dist(angles_check[i])
      -- trig functions have error of 2 sig bits
      if d > 1e-14 then
         print("["..i.."] "..d)
         print(angles[i])
         print(angles_check[i])
         error = error + 1
      end
   end
   print(string.format(" - Found %d/%d errors", error,angles:size(1))) 
end

function test.sphere_to_unit_and_back()
   print("Testing spherical_coords_to_unit_cartesian and back")
   local npts = 1000
   local uc = torch.randn(npts,3)

   for i = 1,npts do uc[i]:mul(1/uc[i]:norm()) end

   local a = geom.unit_cartesian_to_spherical_coords(uc)

   local ucp = geom.spherical_coords_to_unit_cartesian(a)
   local error = 0
   for i = 1,npts do 
      local d = ucp[i]:dist(uc[i])
      -- trig functions have error of 2 sig bits
      if d > 1e-14 then
         print("["..i.."] "..d)
         print(uc[i])
         print(a[i])
         print(ucp[i])
         error = error + 1
      end
   end
   print(string.format(" - Found %d/%d errors", error, npts)) 
end

function test.all()
   test.quaternion_from_to()
   test.quat2rot()
   test.rot2quat()
   test.rotation_by_mat()
   test.rotation_by_quat()
   if util.geom.rotate_by_quatC then 
      test.rotation_by_quatC()
      test.rotate_translateC()
      test.translate_rotateC() 
   end
   test.quat_product()
   test.x_rotation()
   test.y_rotation()
   test.z_rotation()
   test.largest_rotation()
   test.ray_plane_intersection()
   test.ray_face_intersection()
   test.compute_normals()
   test.spherical_coords_to_unit_cartesian()
   -- test.unit_cartesian_to_spherical_coords()
   test.sphere_to_unit_and_back()
end

return test

