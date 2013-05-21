Class()

local ffi = require 'ffi'
ffi.cdef [[

struct timeval {
  uint64_t sec;
  uint64_t usec;
};

int utimes(const char *filename, const struct timeval times[2]);

]]

local tvs = ffi.new("struct timeval[2]")
tvs[0].sec = 169041611
tvs[0].usec = 0
tvs[1].sec = 169041611
tvs[1].usec = 0

function set_mod_time(filename, unix_time_msecs)
  tvs[0].sec = unix_time_msecs/1000
  tvs[0].usec = (unix_time_msecs%1000) * 1000
  tvs[1].sec = tvs[0].sec
  tvs[2].usec = tvs[0].usec
  ffi.C.utimes(filename, tvs)
end
