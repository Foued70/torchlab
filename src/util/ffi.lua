Class()

-- local 
local ffi = require 'ffi'
local path = require 'path'

local shared_lib_ext = ffi.os == 'OSX' and '.dylib' or '.so'

function load(name)
   local shared_lib_file = path.normalize(process.execPath..'/../../lib/'..name..shared_lib_ext)
   return ffi.load(shared_lib_file)
end


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

local type_map = {}
type_map['torch.DoubleTensor'] = {'THDouble', 'THDoubleTensor*', 'double'}
type_map['torch.FloatTensor']  = {'THFloat', 'THFloatTensor*', 'float'}
type_map['torch.ByteTensor']   = {'THByte', 'THByteTensor*', 'unsigned char'}
type_map['torch.CharTensor']   = {'THChar', 'THCharTensor*', 'char'}
type_map['torch.ShortTensor']  = {'THShort', 'THShortTensor*', 'short'}
type_map['torch.IntTensor']    = {'THInt', 'THIntTensor*', 'int'}
type_map['torch.LongTensor']   = {'THLong', 'THLongTensor*', 'long'}


-- save memory by building table
header = {}
for T,t in pairs(type_map) do 
   local s = header_template:gsub("TH",t[1]):gsub("real",t[3])
   ffi.cdef(s)
end

-- Method to return pointer to raw data
function torch.data(obj)
  local type_info = type_map[torch.typename(tensor)]
  local t = ffi.cast(type_info[2], torch.pointer(tensor))
  return t.storage.data
end 

-- Method to return c pointer to tensor or storage struct to pass to C
function torch.cdata(tensor)
  local type_info = type_map[torch.typename(tensor)]
  return ffi.cast(type_info[2], torch.pointer(tensor))
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


function storage_info(tensor)
  local type_info = type_map[torch.typename(tensor)]
  local t = ffi.cast(type_info[2], torch.pointer(tensor))
  return t.storage.data, t.storage.size * ffi.sizeof(type_info[3])
end

