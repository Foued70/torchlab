Class()

-- Below is a test for loading C backends using ffi
local ffi_utils = require 'util.ffi'

local ffi = ffi_utils.ffi

local geomlib = ffi_utils.lib_path("geom")

if paths.filep(geomlib) then
   -- load low level c functions
   ffi.cdef[[
               void rotate_by_quat(THDoubleTensor *result,
                                   THDoubleTensor *vectors,
                                   THDoubleTensor *quat);

               void rotate_translate(THDoubleTensor *result,
                                     THDoubleTensor *vectors,
                                     THDoubleTensor *trans,
                                     THDoubleTensor *quat);

               void translate_rotate(THDoubleTensor *result,
                                     THDoubleTensor *vectors,
                                     THDoubleTensor *trans,
                                     THDoubleTensor *quat);
            ]]

   -- don't want to call C functions directly
   local C  = ffi.load(geomlib)

   -- either there is a way to stick function on torch.DoubleTensor
   -- for automatic type cast or we just do everything in doubles.
   function by_quaternion (out, vec, quat)
      C.rotate_by_quat(torch.cdata(out), torch.cdata(vec),
                       torch.cdata(quat))
   end
   function rotate_translate (out, vec, trans, quat)
      C.rotate_translate(torch.cdata(out), torch.cdata(vec),
                         torch.cdata(trans), torch.cdata(quat))
   end
   function translate_rotate (out, vec, trans, quat)
      C.translate_rotate(torch.cdata(out), torch.cdata(vec),
                         torch.cdata(trans), torch.cdata(quat))
   end
end

-- end of loading C backend

local axes   = torch.eye(3)

x = axes[1]
y = axes[2]
z = axes[3]

local neg_axes   = torch.eye(3):mul(-1)

function axis(normal,d)
   local n   = geom.util.normalized(normal:narrow(1,1,3))
   return geom.quaternion.angle_between(n,d)
end

function x_axis(normal)
   return axis(normal,x)
end

function y_axis(normal)
   return axis(normal,y)
end

function z_axis(normal)
   return axis(normal,z)
end

-- 
function largest (normal)
   local n   = normal:narrow(1,1,3)
   local p   = n:clone():abs()
   local v,i = p:sort()
   local d   = i[-1]
   local a   = axes[d]
   if (n[d] < 0) then
      a = neg_axes[d]
   end
   return axis(n,a),d
end


-- rotate a vector around axis by angle radians
function axis_angle(vec, rot_axis, rot_angle)
   local quat = geom.quaternion.from_axis_angle(rot_axis, rot_angle)
   return geom.quaternion.rotate(vec, vec, quat)
end

-- rotate vector by rotation matrix
function by_matrix(...)
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



