setfenv(1, setmetatable({}, {__index = _G}))


-- local 
ffi = require 'ffi'


-- part of ffi for sending tensors to our own C funcs.
header_template = 
[[
typedef struct THStorage 
{
   real *data; 
   long size; 
   int refcount; 
   char flag;
} THStorage;
               
typedef struct THTensor 
{
   long *size; 
   long *stride; 
   int nDimension; 

   THStorage *storage; 
   long storageOffset; 
   int refcount; 
   char flag;
} THTensor;

 ]]

function generate_header_types(template,types)
   if not template then
      template = header_template
   end
   if not types then
      types = {
         THDouble = "double", 
         THFloat  = "float",
         THByte  = "unsigned char",
         THChar  = "char",
         THShort = "short",
         THInt   = "int", 
         THLong  = "long"
      }
   end
   local ret = ""
   for T,t in pairs(types) do 
      ret = ret .. template:gsub("TH",T):gsub("real",t)
   end
   return ret
end

function generate_header_float_types(template)
   local types = {
      THDouble = "double", 
      THFloat  = "float"
   }
   return generate_header_types(template,types)
end

function generate_header_int_types(template)
   local types = {
      THByte  = "unsigned char",
      THChar  = "char",
      THShort = "short",
      THInt   = "int", 
      THLong  = "long"
   }
   return generate_header_types(template,types)
end


function generate_function_types(template,types)
   if not types then 
      types = {'Double', 'Float', 'Long', 'Int', 'Short', 'Char', 'Byte'}
   end
   local ret = ""
   for _,t in pairs(types) do 
      ret = ret .. 
         template:gsub("THTensor_%((%w+)%)",
                       "TH"..t.."Tensor_%1"):gsub("THTensor",
                                                  "TH"..t.."Tensor") .. "\n"
   end
   return ret
end

function generate_function_int_types(template)
   return generate_function_types(template,{'Long', 'Int', 'Short', 'Char', 'Byte'})
end

function generate_function_float_types(template)
   return generate_function_types(template,{'Double', 'Float'})
end

return (getfenv())
