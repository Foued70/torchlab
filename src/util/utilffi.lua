require 'torch'

local ffi = require 'ffi'


-- part of ffi for sending tensors to our own C funcs.
local template = 
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

function generate_FloatTypes()
   local types = {
      THDouble = "double", 
      THFloat  = "float"
   }
   local ret = ""
   for T,t in pairs(types) do 
      ret = ret .. template:gsub("TH",T):gsub("real",t)
   end
   return ret
end

function generate_IntTypes()
   local types = {
      THByte  = "unsigned char",
      THChar  = "char",
      THShort = "short",
      THInt   = "int", 
      THLong = "long"
   }
   local ret = ""
   for T,t in pairs(types) do 
      ret = ret .. template:gsub("TH",T):gsub("real",t)
   end
   return ret
end

function generate_AllTypes()
   local ret = generate_IntTypes(template)
   return ret .. generate_FloatTypes(template)
end

ffi.cdef(generate_AllTypes() .. [[         ]])


-- FIXME make this loading cleaner for other ffi projects
-- ffi doesn't look in the right place by default
local ffidir = paths.install_lib .. "/torch/"
-- local utilC  = ffi.load(ffidir .. "libutil.dylib")

local utilffi = {}


return utilffi
