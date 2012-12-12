require 'torch'
require 'sys'

require 'utils'

utils.test = {}

require 'utils-testdata'

function utils.test.quaternion_angle()
   print("Testing quaternion angle btw. 2 vectors")
   e = 0
   p = utils.normalize(utils.test.vec[1])

   for i = 2,utils.test.vec:size(1) do
      v = utils.normalize(utils.test.vec[i])
      q = utils.quaternion_angle(p,v)
      d = q - utils.test.result_vec_quat[i-1]
      if (d:abs():max() > 1e-6) then
         print(d,q,utils.test.result_vec_quat[i-1])
         e = e + 1
      end
      p = v
   end
   print(string.format(" - Found %d/%d errors",e,utils.test.vec:size(1)-1))
end

function utils.test.quat2rot()
   print("Testing quaternion to rotation matrix")
   e = 0
   for i = 1,utils.test.quat:size(1) do
      m = utils.rotation_matrix(utils.test.quat[i])
      d = m - utils.test.result_rot_mat[i]
      if (d:abs():max() > 1e-6) then
         print(d,m,utils.test.result_rot_mat[i])
         e = e + 1
      end
   end
   print(string.format(" - Found %d/%d errors",e,utils.test.quat:size(1)))
end

function utils.test.rotation_by_quat()
   print("Testing rotation with quaternion")
   e = 0
   maxerr = 0
   res = torch.Tensor(utils.test.vec:size(1)*utils.test.quat:size(1),4)
   i = 1
   sys.tic()
   for j = 1,utils.test.quat:size(1) do 
      q = utils.test.quat[j]
      for k = 1,utils.test.vec:size(1) do
         v = utils.test.vec[k]
         utils.rotate_by_quat(v,q,res[i]) 
         d = res[i] - utils.test.result_rot_by_quat[i]
         ce = d:abs():max()
         if (ce > maxerr) then maxerr = ce end
         if (ce > 1e-3) then
            print(ce,res[i],utils.test.result_rot_by_quat[i])
            e = e + 1
         end
         i = i + 1
      end
   end
   print(string.format(" - Found %d/%d errors (max: %f) in %2.4fs",
                       e,res:size(1),maxerr,sys.toc()))
end

function utils.test.rotation_by_mat()
   print("Testing rotation w/ rotation matrix")
   e = 0
   maxerr = 0
   res = torch.Tensor(utils.test.vec:size(1)*utils.test.quat:size(1),4)
   i = 1
   sys.tic()
   for j = 1,utils.test.quat:size(1) do 
      q = utils.test.quat[j]
      mat = utils.rotation_matrix(utils.test.quat[j])
      for k = 1,utils.test.vec:size(1) do
         v = utils.test.vec[k]
         utils.rotate_by_mat(v,mat,res[i]) 
         d = res[i] - utils.test.result_rot_by_quat[i]
         ce = d:abs():max()
         if (ce > maxerr) then maxerr = ce end
         if (ce > 1e-3) then
            print(ce,res[i],utils.test.result_rot_by_quat[i])
            e = e + 1
         end
         i = i + 1
      end
   end
   print(string.format(" - Found %d/%d errors (max: %f) in %2.4fs",
                       e,res:size(1),maxerr,sys.toc()))
end

function utils.test.z_rotation()
   print("Testing z-rotation")
   e = 0
   maxerr = 0
   sys.tic()
   for i = 1,utils.test.vec:size(1) do
      tvec = utils.test.vec[i]
      quat = utils.z_rotation(tvec)
      rvec = utils.rotate_by_quat(tvec,quat)
      ce = rvec[3] - rvec:sum()
      if (ce > maxerr) then maxerr = ce end
      if (ce > 1e-8) then
         print(tvec,rvec)
         e = e + 1
      end
   end
   print(string.format(" - Found %d/%d errors (max: %f) in %2.4fs",
                       e,utils.test.vec:size(1),maxerr,sys.toc()))
end

function utils.test.x_rotation()
   print("Testing x-rotation")
   e = 0
   maxerr = 0
   sys.tic()
   for i = 1,utils.test.vec:size(1) do
      tvec = utils.test.vec[i]
      quat = utils.z_rotation(tvec)
      rvec = utils.rotate_by_quat(tvec,quat)
      ce = rvec[1] - rvec:sum()
      if (ce > maxerr) then maxerr = ce end
      if (ce > 1e-8) then
         print(tvec,rvec)
         e = e + 1
      end
   end
   print(string.format(" - Found %d/%d errors (max: %f) in %2.4fs",
                       e,utils.test.vec:size(1),maxerr,sys.toc()))
end

function utils.test.y_rotation()
   print("Testing y-rotation")
   e = 0
   maxerr = 0
   sys.tic()
   for i = 1,utils.test.vec:size(1) do
      tvec = utils.test.vec[i]
      quat = utils.z_rotation(tvec)
      rvec = utils.rotate_by_quat(tvec,quat)
      ce = rvec[2] - rvec:sum()
      if (ce > maxerr) then maxerr = ce end
      if (ce > 1e-8) then
         print(tvec,rvec)
         e = e + 1
      end
   end
   print(string.format(" - Found %d/%d errors (max: %f) in %2.4fs",
                       e,utils.test.vec:size(1),maxerr,sys.toc()))
end

function utils.test.largest_rotation()
   print("Testing largest rotation")
   e = 0
   maxerr = 0
   sys.tic()
   for i = 1,utils.test.vec:size(1) do
      tvec = utils.test.vec[i]
      quat,d = utils.largest_rotation(tvec)
      rvec = utils.rotate_by_quat(tvec,quat)
      ce = rvec[d] - rvec:sum()
      if (ce > maxerr) then maxerr = ce end
      if (ce > 1e-8) then
         print(tvec,rvec)
         e = e + 1
      end
   end
   print(string.format(" - Found %d/%d errors (max: %f) in %2.4fs",
                       e,utils.test.vec:size(1),maxerr,sys.toc()))
end
   
function utils.test.all()
   utils.test.quaternion_angle()
   utils.test.quat2rot()
   utils.test.rotation_by_mat()
   utils.test.rotation_by_quat()
   utils.test.x_rotation()
   utils.test.y_rotation()
   utils.test.z_rotation()
   utils.test.largest_rotation()
end