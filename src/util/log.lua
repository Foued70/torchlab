setfenv(1, setmetatable({}, {__index = _G}))

TRACE = true

function color(code, str)
  return '\27['..code..'m'..str..'\27[0m'
end

function red(str)
  return color('31', str)
end

function gray(str) return color('37', str) end

function error(...)
  local arg = {...}
  print(red('(EE)')..' '..(arg[1] or 'nil'), select(2, ...))

  local tb = debug.traceback()
  tb = tb:match('[^\n]*\n[^\n]*\n(.*)')
  print(tb)
end

function trace(...)
  if TRACE then
    local info = debug.getinfo(2)
    local short_file = info.short_src:match('/([^/]+)$')
    print(gray(short_file..':'..info.currentline), ...) 
  end
end

return (getfenv())
