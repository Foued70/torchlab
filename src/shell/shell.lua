--- Defines shell behavior and configuration settings.
-- This module is exposed as `shell` global variable in ILuaJIT. Unless stated
-- otherwise, all settings and functions can be changed at any time and changes
-- takes effect immediately.
-- However settings are used internally by callbacks so if you change these 
-- functions without take care to read relevant settings at each invocation, 
-- some of them could become ineffective.
-- 
-- Callbacks should not print someting directly to screen but return text to be
-- displayed.
-- @alias shell

local xlua = require 'xlua'
local debug = require 'debug'

--- Adds VT100 control codes to colorize text anr resets settings.
-- Result is `<ESC>[...m<text><ESC>[0m`
local function colorize(text, ...)
  local codes = { }
  for i, code in ipairs{...} do codes[i] = tostring(code) end
  return "\027[" .. table.concat(codes, ";") .. "m" .. text .. "\027[0m"
end

--- Highlights a line in a file.
-- Prints given line and some sorrounding lines in a file. The target line is highlighted.
local function highlight_line(file, target, area)
  local tmpl   = "    %04d: %s"
  local buffer = { }
  local lineno = 1


  local iter
  if type(file) == 'string' then
    iter = file:gmatch("[^\r\n]+")
  else
    iter = file:lines()
  end

  for line in iter do
    if lineno > target+area then break -- area to display is passed
    elseif lineno == target then
      buffer[#buffer+1] = tmpl:format(lineno, colorize(line, 31))
    elseif lineno >= target - area then
      buffer[#buffer+1] = tmpl:format(lineno, line)
    end
    lineno = lineno + 1
  end
  return table.concat(buffer, "\n")
end

local shell = { }



--------------------------------------------------------------------------------
-- Values display
-- @section value

shell.value = { }

--- Separator for each result.
-- This string is used to join results (in case of multiple results).
--
-- Default value is `\n`.
-- @setting value.separator
shell.value.separator = "\n"

--- Whether tables are pretty-printed.
-- If this setting is set to true, then tables are fully printed (insted of
-- traditional `table: 0x...` notation. This setting is currently not fully
-- implemented, it can result in huge outputs as tables are *fully* printed.
--
-- Default value is `true`.
-- @setting value.prettyprint_tables
shell.value.prettyprint_tables = true

--- Whether __tostring metamethod is honored.
-- Tells if table pretty printing is done or not if table has a `__tostring` 
-- metamethod. If set to `false`, tables will always be pretty printed even if 
-- they have the metamethod. This setting takes effect only if 
-- @{value.prettyprint_tables} is set to `true`.
--
-- Default value is `true`.
-- @setting value.table_use_tostring
shell.value.table_use_tostring = true -- when false, pretty print tables even if a __tostring method exists.

--------------------------------------------------------------------------------
-- Error handling
-- @section value
shell.onerror = { }

--- Prints code where error has happend.
-- If set to `true`, error handler will try to load source files to get the code
-- which caused error and print it to screen to help you to catch what happend.
--
-- Default value is `true`.
-- @setting onerror.print_code
shell.onerror.print_code = true

--- Number of lines before and after the error to print.
--
-- Default value is 3.
-- @setting onerror.area
shell.onerror.area = 3

-- mapping between typed commands (as chunk names) and corresponding source
local last_cmd

--- Error handler.
-- This function is directly called by @{xpcall} if command exection has failed
-- (so this will not be called for a syntax error).
-- @param  err  Error object (not always a string).
-- @return[type=string] Error message to be printed.
-- @function onerror.handler
function shell.onerror.handler(err)
  local buffer = { colorize(tostring(err), 31), "Stack traceback:" }
  local tmpl   = "  At %s:%d (in %s %s)"
  -- index in buffer of last xpcall call in stack, it will be used to strip ILuaJIT
  -- internal functions of the error traceback.
  local lastxpcall = 0
  for i=2, math.huge do
    local info = debug.getinfo(i)
    if not info then break end
    if info.func == xpcall then lastxpcall = #buffer end
    buffer[#buffer+1] = tmpl:format(info.source, info.currentline or -1, info.namewhat, info.name or "?")
    if shell.onerror.print_code and (info.what == "Lua" or info.what == "main") and info.currentline then
      local file
      if info.source:match("^@") then 
        file = io.open(info.source:sub(2), "r")
      elseif info.source == 'shell' then
        file = last_cmd
      end

      if file then
        buffer[#buffer+1] = highlight_line(file, info.currentline, shell.onerror.area)
      end
    end
  end
  return table.concat(buffer, "\n", 1, lastxpcall)
end


--------------------------------------------------------------------------------
-- Internal functions.
-- These functions should be changed with care. Take a look at source code to see
-- exectly what original functions do.
-- @section internal

function shell.is_complete(cmd)
  local func, err = loadstring('return '..cmd, 'shell')

  if func then
    return 1
  end

  func, err = loadstring(cmd, 'shell')
  if func then
    return 1
  elseif err:sub(-7) == "'<eof>'" then
    return 0
  end

  return 1
end


function shell.evaluate(cmd)
  reload()

  last_cmd = cmd
  local func, err = loadstring('return '..cmd, 'shell')

  if func then
    shell.result_handler(xpcall(func, shell.onerror.handler))
  else
    func, err = loadstring(cmd, 'shell')
    if func then
      shell.result_handler(xpcall(func, shell.onerror.handler))
    else
      print(err)
    end
  end
end


--- Handles command results
-- Called by @{try} to format execution results. Default implementation calls 
-- @{value.handler} for each result and returns the concatenation of calls.
-- @param[type=boolean]  success  Whether the call has been successfull.
-- @param  ...  Command results in case of success or error message in case of failure.
-- @return[type=string] Results as printable string.
function shell.result_handler(success, ...)
  if success then
    local n =  select("#", ...)
    if n == 1 then 
      shell.value.handler(false, select(1, ...))
    else
      for i = 1, n do
        shell.value.handler(i, select(i, ...))
      end
    end
  else
    -- error
    print (...)
  end
end

--- Callback to transform result into a printable string.
-- @param[type=number]  pos    Result position (starting at one).
-- @param               value  Result to print (can be any type).
-- @return[type=string] A printable representation of `value`.
-- @function value.handler
function shell.value.handler(pos, value)
  if pos then
    io.stdout:write(colorize("["..pos.."]", 1, 30)..' ')
  end

  local tvalue = type(value)
  if tvalue == "table" and shell.value.prettyprint_tables then
    -- if table has a __tostring metamethod, then use it
    local mt = getmetatable(value)
    if mt and mt.__tostring and shell.value.table_use_tostring then
      print(value)
    else
      -- otherwise pretty-print it
      xlua.print(value)
    end
  else -- fall back to default tostring
    print(value)
  end
end




return shell
