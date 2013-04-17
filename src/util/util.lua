Class()

-- Below is a test for loading C backends using ffi 
local ffi = require 'ffi'

local ffi_utils = require 'util.ffi'

local function_templates = 
[[
    void THTensor_rotate_by_quat(THTensor *result, 
                                 THTensor *vectors, 
                                 THTensor *quat);
 ]]

-- FIXME make this loading cleaner for other ffi projects
-- ffi doesn't look in the right place by default
local utillib = paths.install_lib .. "/torch/lua/" .. "libutil.dylib"

if paths.filep(utillib) then 
   -- load low level c functions
   ffi.cdef(ffi_utils.generate_header_float_types() ..
            ffi_utils.generate_function_float_types(function_templates))

   -- don't want to call C functions directly
   local C  = ffi.load(utillib)

   function rotate_by_quatC (out, vec, q)
      C.THDoubleTensor_rotate_by_quat(ffi.cast("THDoubleTensor*",torch.pointer(out)),
                                      ffi.cast("THDoubleTensor*",torch.pointer(vec)),
                                      ffi.cast("THDoubleTensor*",torch.pointer(q)))
   end
end

-- end of loading C backend test


-- Like unfold but produces contiguous chunks which would replicate
-- data in overlaps.  Start with a less general version which takes
-- h,w,dims matrix as input and outputs r,c,s1*s2,dims as
-- output. (Contiguous unfold useful for bilateral filtering also).
function grid_contiguous (m,s1,s2)
   local uf = m:unfold(1,s1,s1):unfold(2,s2,s2)
   local out = torch.Tensor(uf:size(1),uf:size(2),uf:size(4)*uf:size(5),uf:size(3))
   for r = 1,uf:size(1) do
      for c = 1,uf:size(2) do
         out[r][c]:copy(uf[r][c])
      end
   end
   return out
end


-- <vals> sorted list of values
-- 
-- <val> value for which we want pointer into list where everything
--       before pointer is < val and after >= val

function get_index_lt_val(vals,val)
   return vals:lt(val):sum()
end
