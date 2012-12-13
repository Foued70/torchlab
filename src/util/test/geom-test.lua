require 'torch'
require 'sys'

-- get vars passed to utils.include function
local utils = ...
local geom  = utils.geom

local test = {}

-- FIXME this isn't super clean as it depends on the torchPackageName
-- which is hardcoded to 'utils'
test.data  = utils.include('utils','geom-data.lua')

function test.quaternion_angle()
   print("Testing quaternion angle btw. 2 vectors")
   local e = 0
   local p = geom.normalize(test.data.vec[1])

   for i = 2,test.data.vec:size(1) do
      local v = geom.normalize(test.data.vec[i])
      local q = geom.quaternion_angle(p,v)
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
      local m = geom.rotation_matrix(test.data.quat[i])
      local d = m - test.data.result_rot_mat[i]
      if (d:abs():max() > 1e-6) then
         print(d,m,test.data.result_rot_mat[i])
         e = e + 1
      end
   end
   print(string.format(" - Found %d/%d errors",e,test.data.quat:size(1)))
end

function test.rotation_by_quat()
   print("Testing rotation with quaternion")
   local e = 0
   local maxerr = 0
   local res = torch.Tensor(test.data.vec:size(1)*test.data.quat:size(1),4)
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
            print(ce,res[i],test.data.result_rot_by_quat[i])
            e = e + 1
         end
         i = i + 1
      end
   end
   print(string.format(" - Found %d/%d errors (max: %f) in %2.4fs",
                       e,res:size(1),maxerr,sys.toc()))
end

function test.rotation_by_mat()
   print("Testing rotation w/ rotation matrix")
   local e = 0
   local maxerr = 0
   local res = torch.Tensor(test.data.vec:size(1)*test.data.quat:size(1),4)
   local i = 1
   sys.tic()
   for j = 1,test.data.quat:size(1) do 
      local q = test.data.quat[j]
      local mat = geom.rotation_matrix(test.data.quat[j])
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
   print(string.format(" - Found %d/%d errors (max: %f) in %2.4fs",
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
   print(string.format(" - Found %d/%d errors (max: %f) in %2.4fs",
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
   print(string.format(" - Found %d/%d errors (max: %f) in %2.4fs",
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
   print(string.format(" - Found %d/%d errors (max: %f) in %2.4fs",
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
      local rvec = geom.rotate_by_quat(tvec,quat)
      local ce = rvec[d] - rvec:sum()
      if (ce > maxerr) then maxerr = ce end
      if (ce > 1e-8) then
         print(tvec,rvec)
         e = e + 1
      end
   end
   print(string.format(" - Found %d/%d errors (max: %f) in %2.4fs",
                       e,test.data.vec:size(1),maxerr,sys.toc()))
end
   
function test.all()
   test.quaternion_angle()
   test.quat2rot()
   test.rotation_by_mat()
   test.rotation_by_quat()
   test.x_rotation()
   test.y_rotation()
   test.z_rotation()
   test.largest_rotation()
end

return test