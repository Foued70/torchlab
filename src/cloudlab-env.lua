_G.module = function(name)
  local m = _G[name] or {}
  _G[name] = m

  setfenv(2, m)
end

require'torch'
local mod = require'module'
_G.math = require'math'
_G.string = require'string'
_G.table = require'table'
_G.os = mod.require'os'

_G.log = require './util/log'

local path = require'path'

_G.CLOUDLAB_SRC = __dirname
_G.HOME = process.env.HOME

require './util/Class'

function printf (...)
   print(string.format(...))
end

-- local shell = require('./shell/shell')
-- _G.shell_evaluate = shell.evaluate
-- _G.shell_is_complete = shell.is_complete


-- Make sure value is converted to a valid string representation
-- for querystring use
function toquerystring(val, vtype)
  vtype = vtype or type(val)

  if 'table' == vtype then
    return ''
  elseif 'string' == vtype then
    return val
  end

  return tostring(val)
end

local querystring = require'querystring'

function querystring.urlencodecomponent(str)
  if str then
    str = str:gsub('\n', '\r\n')
    str = str:gsub('([^%w])', function(c)
      return string.format('%%%02X', c:byte())
    end)
  end
  return str
end

-- Insert a item into a querystring result table
function insertqueryitem(ret, key, val, sep, eq)
  local vtype = nil -- string
  local skey = nil -- string (Safe key)
  local count = 0

  vtype = type(val)
  skey = querystring.urlencodecomponent(key, sep, eq)

  -- only use numeric keys for table values
  if 'table' == vtype then
    for i, v in ipairs(val) do
      if nil ~= val then
        count = count + 1
        v = querystring.urlencodecomponent(toquerystring(v), sep, eq)
        table.insert(ret, table.concat({skey, v}, eq))
      end
    end

    if 0 == count then
      table.insert(ret, table.concat({skey, ''}, eq))
    end

    count = 0
  else
    val = querystring.urlencodecomponent(toquerystring(val, vtype), sep, eq)
    table.insert(ret, table.concat({skey, val}, eq))
  end
end

-- Create a querystring from the given table.
function querystring.stringify(params, order, sep, eq)
  if not params then
    return ''
  end

  order = order or nil
  sep = sep or '&'
  eq = eq or '='
  local ret = {}
  local vtype = nil -- string
  local count = 0
  local skey = nil -- string

  if order then
    local val = nil -- mixed
    for i, key in ipairs(order) do
      val = params[key]
      insertqueryitem(ret, key, val, sep, eq)
    end
  else
    for key, val in pairs(params) do
      insertqueryitem(ret, key, val, sep, eq)
    end
  end

  if 0 == #ret then
    return ''
  end

  return table.concat(ret, sep)
end


local d = net.Depot

return ''


