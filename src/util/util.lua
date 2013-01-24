-- unfortunately we need to cast the lua numbers properly for %d etc. to work.
-- ffi = require("ffi")
-- ffi.cdef[[ int printf(const char *fmt, ...); ]]

-- printf = ffi.C.printf

function printf (...)
   print(string.format(...))
end