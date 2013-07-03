Class()

-- local 
local ffi = require 'ffi'
local path = require 'path'

local shared_lib_ext = ffi.os == 'OSX' and '.dylib' or '.so'

function load(name)
   local shared_lib_file = path.normalize(process.execPath..'/../../lib/'..name..shared_lib_ext)
   return ffi.load(shared_lib_file)
end


