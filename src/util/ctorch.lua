local ffi = require 'ffi'

Class()

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
type_map['torch.DoubleTensor'] = {'THDouble', 'THDoubleTensor', 'double'}
type_map['torch.FloatTensor']  = {'THFloat',  'THFloatTensor',  'float'}
type_map['torch.ByteTensor']   = {'THByte',   'THByteTensor',   'unsigned char'}
type_map['torch.CharTensor']   = {'THChar',   'THCharTensor',   'char'}
type_map['torch.ShortTensor']  = {'THShort',  'THShortTensor',  'short'}
type_map['torch.IntTensor']    = {'THInt',    'THIntTensor',    'int'}
type_map['torch.LongTensor']   = {'THLong',   'THLongTensor',   'long'}

function generateTypes(template)
   for T,t in pairs(type_map) do 
      s = template:gsub("TH",t[1]):gsub("real",t[3])
      ffi.cdef(s)
   end
end

generateTypes(header_template)

-- add the storage types to the type_map *after* the generateTypes has been called
type_map['torch.DoubleStorage'] = {'THDouble', 'THDoubleStorage', 'double'}
type_map['torch.FloatStorage']  = {'THFloat',  'THFloatStorage',  'float'}
type_map['torch.ByteStorage']   = {'THByte',   'THByteStorage',   'unsigned char'}
type_map['torch.CharStorage']   = {'THChar',   'THCharStorage',   'char'}
type_map['torch.ShortStorage']  = {'THShort',  'THShortStorage',  'short'}
type_map['torch.IntStorage']    = {'THInt',    'THIntStorage',    'int'}
type_map['torch.LongStorage']   = {'THLong',   'THLongStorage',   'long'}

-- Method to return c pointer to tensor or storage struct to pass to C
function torch.cdata(tensorOrStorage)
  local type_info = type_map[torch.typename(tensorOrStorage)]
  return ffi.cast(type_info[2].."*", torch.pointer(tensorOrStorage))
end 

-- Method to return pointer to raw data
function torch.data(tensor)
  local t = torch.cdata(tensor)
  return t.storage.data
end 

function storage_info(tensor)
  local type_info = type_map[torch.typename(tensor)]
   local t = torch.cdata(tensor) 
   return t.storage.data, t.storage.size * ffi.sizeof(type_info[3])
end

