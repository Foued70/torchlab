local path = require'path'

_G.CLOUDLAB_SRC = __dirname
_G.CLOUDLAB_ROOT = path.dirname(path.dirname(process.execPath))
_G.HOME = process.env.HOME

_G.module = function(name)
  local m = _G[name] or {}
  _G[name] = m

  setfenv(2, m)
end

require 'torch'
require './util/Class'

local mod = require'module'
_G.math = require'math'
_G.string = require'string'
_G.table = require'table'
_G.os = mod.require'os'

_G.log = require './util/log'

require 'dok'
require 'nn'
require './image'

function _G.printf (...)
   print(string.format(...))
end


local d = net.Depot

require('repl').start = shell.repl.start
