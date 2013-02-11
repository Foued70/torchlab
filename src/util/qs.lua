-- http://www.lua.org/pil/20.3.html

local qs = {}

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

function qs.stringify (t)
  local s = ""
  for k,v in pairs(t) do
    s = s .. "&" .. qs.escape(k) .. "=" .. qs.escape(v)
  end
  -- remove first `&'
  return string.sub(s, 2)    
end


return qs