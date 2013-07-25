Class()

local uv = require 'uv'
local utils = require 'utils'
local debug = require('debug')
local libreadline_async = require 'libreadline_async'
local completer = require './completer'


local buffer = ''

local function gather_results(success, ...)
  return success, { ... }
end

local function print_results(results)
  for i = 1, #results do
    print(utils.dump(results[i]))
  end
end

local function evaluate_line(line)
  local chunk  = buffer .. line
  local f, err = loadstring('return ' .. chunk, 'REPL') -- first we prefix return

  if not f then
    f, err = loadstring(chunk, 'REPL') -- try again without return
  end

  if f then
    buffer = ''
    local success, results = gather_results(xpcall(f, debug.traceback))

    if success then
      -- successful call
      if #results > 0 then
        print_results(results)
      end
    else
      -- error
      print(results[1])
    end
  else
    if err:match "'<eof>'$" then
      -- Lua expects some more input; stow it away for next time
      buffer = chunk .. '\n'
      return false, chunk
    else
      print(err)
      buffer = ''
    end
  end

  return true, chunk
end


local ReadlineAsync = uv.Handle:extend()
function ReadlineAsync:initialize()
  libreadline_async.set_prompt('> ')
  self.userdata = libreadline_async.start_readline_loop()
end


function start()
  local rl = ReadlineAsync:new()

  local last_command = nil

  rl:on('complete', function(word, line, startpos, endpos)
    local completions = completer.complete(word, line, startpos, endpos)
    libreadline_async.set_completions(completions)
  end)

  rl:on('execute', function(line)
    local executed, full_command = evaluate_line(line)
    if executed and #full_command > 0 and full_command ~= last_command then -- executed a non-blank command
      last_command = full_command
      libreadline_async.add_to_history(full_command)
    end

    local prompt = '> '
    if not executed then prompt = '>> ' end
    libreadline_async.set_prompt(prompt)
  end)

  rl:on('close', function()
    process.exit()
  end)
end


completer.final_char_setter = function(char)
  libreadline_async.set_completion_append_character(#char > 0 and char:byte(1,1) or 0)
end