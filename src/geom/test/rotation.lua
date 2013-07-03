Class()

local rot  = geom.rotation
local quat = geom.quaternion
local geom = geom.util

data = require "geom.test.data"


function rotation_by_quat()
   print("Testing rotation with quaternion (C version)")
   local e      = 0
   local maxerr = 0
   local vecs   = data.vec
   local nvecs  = vecs:size(1)
   local quats  = data.quat
   local nquats = quats:size(1)
   local res    = torch.Tensor(nvecs*nquats,3)
   local groundt = data.result_rot_by_quat
   local i      = 1
   local offset = 1
   log.tic()

   rot.by_quaternion(res,quats,vecs)

      
   local d = res - groundt
   local ce  = d:abs():max()
   if (ce > maxerr) then maxerr = ce end
   e = e + d:abs():gt(1e-3):sum()
   print(string.format(" - Found %d/%d errors (max: %e) in %2.4fs",
                       e,res:size(1),maxerr,log.toc()))
end

function rotate_translate()
   print("Testing rotation with quaternion then translation (C version)")
   local e      = 0
   local maxerr = 0
   local vecs   = data.vec
   local nvecs  = vecs:size(1)
   local quats  = data.quat
   local nquats = quats:size(1)
   res    = torch.Tensor(nvecs*nquats,3)
   groundt = res:clone()
   local i      = 1
   local offset = 1

   log.tic()
   for j = 1,nquats do
      local q = quats[j]
      for k = 1,nvecs do
         groundt[i] = vecs[j] + quat.rotate(q,vecs[k])
         i = i + 1
      end
   end
   print(string.format(" - built ground truth in %2.4fs", log.toc()))

   sys.tic()
   for i = 1,nquats do
      local q   = quats[i]
      local out = res:narrow(1,offset,nvecs)
      local gt  = groundt:narrow(1,offset,nvecs) 
      offset = offset + nvecs

      rot.rotate_translateC(out,vecs,vecs[i],q)

      local d = out - gt
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
   end
   print(string.format(" - Found %d/%d errors (max: %e) in %2.4fs",
                       e,res:size(1),maxerr,sys.toc()))
end


function translate_rotate()
   print("Testing translation then rotation with quaternion (C version)")
   local e      = 0
   local maxerr = 0
   local vecs   = data.vec
   local nvecs  = vecs:size(1)
   local quats  = data.quat
   local nquats = quats:size(1)
   res    = torch.Tensor(nvecs*nquats,3)
   groundt = res:clone()
   local i      = 1
   local offset = 1

   log.tic()
   for j = 1,nquats do
      local q = quats[j]
      for k = 1,nvecs do
         local v   = vecs[k] + vecs[j]
         quat.rotate(groundt[i],q,v)
         i = i + 1
      end
   end
   print(string.format(" - built ground truth in %2.4fs", log.toc()))

   sys.tic()
   for i = 1,nquats do
      local q   = quats[i]
      local out = res:narrow(1,offset,nvecs)
      local gt  = groundt:narrow(1,offset,nvecs) 
      offset = offset + nvecs

      rot.translate_rotateC(out,vecs,vecs[i],q)

      local d = out - gt
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
   end
   print(string.format(" - Found %d/%d errors (max: %e) in %2.4fs",
                       e,res:size(1),maxerr,sys.toc()))
end

function rotation_by_mat()
   print("Testing rotation w/ rotation matrix")
   local e = 0
   local maxerr = 0
   local res = torch.Tensor(data.vec:size(1)*data.quat:size(1),3)
   local i = 1
   log.tic()
   for j = 1,data.quat:size(1) do
      local mat = data.result_rot_mat[j]
      for k = 1,data.vec:size(1) do
         local v = data.vec[k]
         rot.by_matrix(res[i],v,mat)
         local d = res[i] - data.result_rot_by_quat[i]
         local ce = d:abs():max()
         if (ce > maxerr) then maxerr = ce end
         if (ce > 1e-3) then
            print(ce,res[i],data.result_rot_by_quat[i])
            e = e + 1
         end
         i = i + 1
      end
   end
   print(string.format(" - Found %d/%d errors (max: %e) in %2.4fs",
                       e,res:size(1),maxerr,log.toc()))
end

function z_rotation()
   print("Testing z-rotation")
   local e = 0
   local maxerr = 0
   log.tic()
   for i = 1,data.vec:size(1) do
      local tvec = data.vec[i]
      local q = rot.z_axis(tvec)
      local rvec = quat.rotate(q,tvec)
      local ce = rvec[3] - rvec:sum()
      if (ce > maxerr) then maxerr = ce end
      if (ce > 1e-8) then
         print(tvec,rvec)
         e = e + 1
      end
   end
   print(string.format(" - Found %d/%d errors (max: %e) in %2.4fs",
                       e,data.vec:size(1),maxerr,log.toc()))
end

function x_rotation()
   print("Testing x-rotation")
   local e = 0
   local maxerr = 0
   log.tic()
   for i = 1,data.vec:size(1) do
      local tvec = data.vec[i]
      local q     = rot.x_axis(tvec)
      local rvec = quat.rotate(q,tvec)
      local ce = rvec[1] - rvec:sum()
      if (ce > maxerr) then maxerr = ce end
      if (ce > 1e-8) then
         print(tvec,rvec)
         e = e + 1
      end
   end
   print(string.format(" - Found %d/%d errors (max: %e) in %2.4fs",
                       e,data.vec:size(1),maxerr,log.toc()))
end

function y_rotation()
   print("Testing y-rotation")
   local e = 0
   local maxerr = 0
   log.tic()
   for i = 1,data.vec:size(1) do
      local tvec = data.vec[i]
      local q    = rot.y_axis(tvec)
      local rvec = quat.rotate(q,tvec)
      local ce = rvec[2] - rvec:sum()
      if (ce > maxerr) then maxerr = ce end
      if (ce > 1e-8) then
         print(tvec,rvec)
         e = e + 1
      end
   end
   print(string.format(" - Found %d/%d errors (max: %e) in %2.4fs",
                       e,data.vec:size(1),maxerr,log.toc()))
end

function largest_rotation()
   print("Testing largest rotation")
   local e = 0
   local maxerr = 0
   log.tic()
   for i = 1,data.vec:size(1) do
      local tvec = data.vec[i]
      local q,d = rot.largest(tvec)
      local rvec   = quat.rotate(q,tvec)
      local ce = rvec[d] - rvec:sum()
      if (ce > maxerr) then maxerr = ce end
      if (ce > 1e-8) then
         print(tvec,rvec)
         e = e + 1
      end
   end
   print(string.format(" - Found %d/%d errors (max: %e) in %2.4fs",
                       e,data.vec:size(1),maxerr,log.toc()))
end


function all()
   rotation_by_mat()
   rotation_by_quat()
   rotate_translate()
   translate_rotate() 
   x_rotation()
   y_rotation()
   z_rotation()
   largest_rotation()
end


