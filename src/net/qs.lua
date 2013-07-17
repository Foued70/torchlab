local querystring = require 'querystring'

local qs = Class()

function qs.unescape (s)
  s = string.gsub(s, "+", " ")
  s = string.gsub(s, "%%(%x%x)", function (h)
    return string.char(tonumber(h, 16))
    end)
  return s
end

function qs.escape (s)
  s = string.gsub(s, "([&=+%c])", function (c)
    return string.format("%%%02X", string.byte(c))
    end)
  s = string.gsub(s, " ", "+")
  return s
end

function qs.parse(query)
  local parsed = {}
  for name, value in string.gfind(query, "([^&=]+)=([^&=]+)") do
    name = qs.unescape(name)
    value = qs.unescape(value)
    parsed[name] = value
  end
  return parsed
end

function qs.urlencodecomponent(str)
  if str then
    str = str:gsub('\n', '\r\n')
    str = str:gsub('([^%w])', function(c)
      return string.format('%%%02X', c:byte())
    end)
  end
  return str
end

-- Make sure value is converted to a valid string representation
-- for querystring use
local function toquerystring(val, vtype)
  vtype = vtype or type(val)

  if 'table' == vtype then
    return ''
  elseif 'string' == vtype then
    return val
  end

  return tostring(val)
end

-- Insert a item into a querystring result table
local function insertqueryitem(ret, key, val, sep, eq)
  local vtype = nil -- string
  local skey = nil -- string (Safe key)
  local count = 0

  vtype = type(val)
  skey = qs.urlencodecomponent(key, sep, eq)

  -- only use numeric keys for table values
  if 'table' == vtype then
    for i, v in ipairs(val) do
      if nil ~= val then
        count = count + 1
        v = qs.urlencodecomponent(toquerystring(v), sep, eq)
        table.insert(ret, table.concat({skey, v}, eq))
      end
    end

    if 0 == count then
      table.insert(ret, table.concat({skey, ''}, eq))
    end

    count = 0
  else
    val = qs.urlencodecomponent(toquerystring(val, vtype), sep, eq)
    table.insert(ret, table.concat({skey, val}, eq))
  end
end

-- Create a querystring from the given table.
function qs.stringify(params, order, sep, eq)
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

