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

-- Method to return c pointer to tensor or storage struct to pass to C
function torch.cdata(tensor)
  local type_info = type_map[torch.typename(tensor)]
  return ffi.cast(type_info[2].."*", torch.pointer(tensor))
end 

-- Method to return pointer to raw data
function torch.data(tensor)
  local t = torch.cdata(tensor)
  return t.storage.data
end 


function storage_info(tensor)
  -- log.trace(torch.typename(tensor))
  local type_info = type_map[torch.typename(tensor)]
  local t = ffi.cast(type_info[2], torch.pointer(tensor))
  return t.storage.data, t.storage.size * ffi.sizeof(type_info[3])
end

