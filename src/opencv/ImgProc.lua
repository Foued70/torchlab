ffi = require 'ffi'
libopencv = util.ffi.load("libopencv")
ctorch = util.ctorch

ffi.cdef [[
void Mat_convert (Mat* input, Mat* output, int cvttype);
]]

conversion = require './types/Mat_conversion'

-- not a "Class()" no self, just a bunch of functions in a namespace.
ImgProc = {}

function ImgProc.convert(...) 
   local input, output, type_str
   args = {...}
   nargs = #args
   if nargs == 2 then 
      input     = args[1]
      output    = input:clone() 
      type_str  = args[2]
   elseif nargs == 3 then 
      output    = args[1]
      input     = args[2]
      type_str  = args[3]
   else
      error("wrong number of args")
   end
   type_enum  = conversion[type_str]
   input_mat  = input.mat
   output_mat = output.mat
   if type_enum and input_mat and output_mat then 
      print("converting ".. type_str .. " " .. type_enum)
      libopencv.Mat_convert(input_mat,output_mat,type_enum)
      return output
   else
      error("Don't understand conversion type "..type_str)
   end 
   return output
end


return ImgProc
