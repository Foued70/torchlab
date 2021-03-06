local debug = require 'debug'
local uv = require 'uv_native'
local fs = require 'fs'
local path = require 'path'

setfenv(1, setmetatable({}, {__index = _G}))

TRACE = true

local logfilename = path.join(CLOUDLAB_ROOT, 'cloudlab.log')

function color(code, str)
  return '\27['..code..'m'..str..'\27[0m'
end

function red(str)     return color('31', str) end
function gray(str)    return color('37', str) end
function green(str)   return color('32', str) end
function blue(str)   return color('34', str) end
function cyan(str)   return color('36', str) end

function error(...)
  local arg = {...}
  print(red('(EE)')..' '..tostring(arg[1]), select(2, ...))

  local tb = debug.traceback()
  tb = tb:match('[^\n]*\n[^\n]*\n(.*)')
  print(tb)
end

function info(...)
  local args = {...}
  -- fs.appendFile(logfilename, green('(II)')..' '..(args[1] or 'nil'), select(2, ...) or '')
  fs.appendFile(logfilename, green('(II)')..' '..table.concat(args, '  ')..'\n')
end

function trace(...)
  trace_x(3, gray, ...)
end

-- allows printf style formating
function tracef(fmt,...)
   trace_x(3, gray, string.format(fmt,...))
end

function trace_x(level, color_func, ...)
  if TRACE then
    local info = debug.getinfo(level)
    local short_file = info.short_src:match('/([^/]+)$') or info.short_src
    local func = info.name or ''
    local rest = ''
    local delim = ''
    for i, o in ipairs({...}) do
      local str = tostring(o)
      -- if the string has newlines in it, assume it wants to start
      -- on it's own line (like torch tensors)
      if str:match('\n') then 
        rest = rest..'\n'..str
        delim = '' -- assume this ends with a newline, so no delim for the next one
      else
        rest = rest..delim..str
        delim = ' '
      end
    end
    print(color_func(short_file..':'..(info.name or '')..':'..info.currentline), rest) 
  end
end

local last_tic = 0
function tic()
 last_tic = uv.hrtime()
end

function toc()
 return uv.hrtime() - last_tic
end

return (getfenv())
