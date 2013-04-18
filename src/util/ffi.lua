setfenv(1, setmetatable({}, {__index = _G}))

-- local 
ffi = require 'ffi'

-- part of ffi for sending tensors to our own C funcs.
local header_template = 
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

local types = {
   THDouble = "double", 
   THFloat  = "float",
   THByte  = "unsigned char",
   THChar  = "char",
   THShort = "short",
   THInt   = "int", 
   THLong  = "long"
}

-- save memory by building table
header = {}
for T,t in pairs(types) do 
   local s = header_template:gsub("TH",T):gsub("real",t)
   table.insert(header, s)
end
-- convert table into string
header = table.concat(header,'\n')

-- Load header
ffi.cdef(header)

-- Method to return pointer to raw data
function torch.data(obj)
   -- first cast pointer into the right type
   local type_obj     = torch.typename(obj)
   local type_tensor  = type_obj:gfind('torch%.(.*)Tensor')()
   local type_storage = type_obj:gfind('torch%.(.*)Storage')()
   if type_tensor then
      -- return raw pointer to data
      return ffi.cast('TH' .. type_tensor .. 'Tensor*', torch.pointer(obj)).storage.data
   elseif type_storage then
      -- return raw pointer to data
      return ffi.cast('TH' .. type_storage .. 'Storage*', torch.pointer(obj)).data
   else
      print('Unknown data type: ' .. type_obj)
   end
end 

-- Method to return c pointer to tensor or storage struct to pass to C
function torch.cdata(obj)
   -- first cast pointer into the right type
   local type_obj     = torch.typename(obj)
   local type_tensor  = type_obj:gfind('torch%.(.*)Tensor')()
   local type_storage = type_obj:gfind('torch%.(.*)Storage')()
   if type_tensor then
      -- return raw pointer to data
      return ffi.cast('TH' .. type_tensor .. 'Tensor*', torch.pointer(obj))
   elseif type_storage then
      -- return raw pointer to data
      return ffi.cast('TH' .. type_storage .. 'Storage*', torch.pointer(obj))
   else
      print('Unknown data type: ' .. type_obj)
   end
end 

-- allows us to easily write functions for all tensor types
function generate_function_types(template,types)
   if not types then 
      types = {'Double', 'Float', 'Long', 'Int', 'Short', 'Char', 'Byte'}
   end
   local ret = {}
   for _,t in pairs(types) do 
      local s = template:gsub("THTensor", "TH"..t.."Tensor")
      table.insert(ret, s)
   end
   return table.concat(ret,'\n')
end

function generate_function_int_types(template)
   return generate_function_types(template,{'Long', 'Int', 'Short', 'Char', 'Byte'})
end

function generate_function_float_types(template)
   return generate_function_types(template,{'Double', 'Float'})
end

function lib_path(libname)
   local ext = ffi.os == "OSX" and ".dylib" or ".so"
   return paths.install_lib .. "/torch/lua/lib" .. libname..ext
end

return (getfenv())
