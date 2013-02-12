-- https://github.com/dvv/luvit-curl/blob/master/lib/cookie.lua
-- https://github.com/mikeal/request/tree/master/vendor/cookie

local _ = require "util/underscore"

local strptime = function(str, format)
  return str:find('2013') and 2000000000 or 0
end

local parse_url = function(str)
  local url = require("socket.url").parse(str)
  --p('URL', str, url)
  return {
    secure = url.protocol == 'https',
    domain = url.host or '',
    path = url.path
  }
end

-- http://tools.ietf.org/html/rfc6265#section-5.1.3
local function domain_match(a, b)
  a = a:lower()
  b = b:lower()
  -- The domain string and the string are identical.
  if a == b then return true end
  -- The domain string is a suffix of the string.
  -- The last character of the string that is not included in the
  -- domain string is a %x2E (".") character.
  -- TODO: The string is a host name (i.e., not an IP address).
  if a:match('^%w[%w_-]*%.' .. b) then return true end
  return false
end

-- http://tools.ietf.org/html/rfc6265#section-5.1.4
local function path_match(a, b)
  -- The cookie-path and the request-path are identical.
  if a == b then return true end
  -- The cookie-path is a prefix of the request-path, and the last
  -- character of the cookie-path is %x2F ("/").
  local start, finish = a:find(b, 1, true)
  if start == 1 and b:sub(#b) == '/' then return true end
  -- The cookie-path is a prefix of the request-path, and the first
  -- character of the request-path that is not included in the cookie-path
  -- is a %x2F ("/") character.
  if start == 1 and a:sub(finish + 1, finish + 1) == '/' then return true end
  return false
end

local Cookie = {}

function Cookie:new(str, url)
  local c = {}
  setmetatable(c, self)
  self.__index = self
  
  c.str = str
  
  local uri = url and parse_url(url)
  if uri and uri.path == '' then uri.path = '/' end  
  
  local name, value, attrs = (str..';'):match('%s*(.-)=(.-)%s*;(.*)')
  
  c.name = name
  c.value = value
  
  -- parse key/value attributes, if any
  -- p('ATTRS', attrs)
  if attrs then
    attrs = attrs:gsub('%s*([%a_][%w_-]*)%s*=%s*(.-)%s*;', function (attr, value)
      -- p('ATTR', attr, value)
      attr = attr:lower()
      -- http://tools.ietf.org/html/rfc6265#section-5.2.1
      if attr == 'expires' then
        local expires = strptime(value, '%Y-%m-%d %H:%M:%S %z')
        c[attr] = expires
      -- http://tools.ietf.org/html/rfc6265#section-5.2.2
      elseif attr == 'max-age' then
        local delta = tonumber(value)
        if delta then
          if delta > 0 then
            c.expires = os.time() + delta
          else
            c.expires = 0
          end
        end
      -- http://tools.ietf.org/html/rfc6265#section-5.2.3
      elseif attr == 'domain' then
        if value ~= '' then
          -- drop leading dot
          if value:sub(1, 1) == '.' then
            value = value:sub(2)
          end
          c[attr] = value:lower()
        end
      -- http://tools.ietf.org/html/rfc6265#section-5.2.4
      elseif attr == 'path' then
        c[attr] = value
      end
      -- consume attribute
      return ''
    end)

    -- parse flag attributes
    -- p('ATTR1', attrs)
    for attr in attrs:gmatch('%s*([%w_-]-)%s*;') do
      -- p('ATTR', attr)
      attr = attr:lower()
      -- http://tools.ietf.org/html/rfc6265#section-5.2.5
      -- http://tools.ietf.org/html/rfc6265#section-5.2.6
      if attr == 'httponly' or attr == 'secure' then
        c[attr] = true
      end
    end

  end

  -- set default values for optional attributes
  if not c.domain then
    --c.host_only = true
    c.domain = (uri and uri.domain) or ''
  end 
  -- http://tools.ietf.org/html/rfc6265#section-5.1.4
  if not c.path then
    c.path = uri and uri.path:match('^(.*)/')
    if c.path == '' then
      c.path = '/'
    end
  end

  -- check attributes validity
  -- http://tools.ietf.org/html/rfc6265#section-5.3
  local valid = true

  if not c.name then
    valid = false
  end

  -- The value for the Domain attribute contains no embedded dots,
  -- and the value is not .local
  if c.domain then
    local dot = c.domain:find('.', 2, true)
    if not dot or dot == #c.domain then
      valid = false
    end
  end

  -- If the canonicalized request-host does not domain-match the
  -- domain-attribute.
  if c.domain and uri and not domain_match(uri.domain, c.domain) then
    valid = false
  end

  -- update the c
  -- http://tools.ietf.org/html/rfc6265#section-5.3
  if valid then

    -- if expires <= now, remove the c
    if c.expires and c.expires <= os.time() then
      c.value = nil
      c.expires = nil
    end

  end
  
  return c
end


local CookieJar = {}

function CookieJar:new()
  local cj = {}
  setmetatable(cj, self)
  self.__index = self
  
  cj.cookies = {}
  
  return cj
end

function CookieJar:reset()
  self.cookies = {}
end

function CookieJar:add(cookie)
  -- Avoid duplication (same path, same name)
  local function checkDup(c)
    return not (c.name == cookie.name and c.domain == cookie.domain and c.path == cookie.path)
  end
  self.cookies = _.filter(self.cookies, checkDup)
  table.insert(self.cookies, cookie)
end

function CookieJar:get(name, domain, path)
  return _.find(self.cookies, function(v)
    return v.name == name and (not domain or v.domain == domain) and (not path or v.path == path) 
    end)
end

function CookieJar:get_for_url(url)
  local uri = parse_url(url or '/')
  local now = os.time()
  
  local function relevant(cookie)
    return domain_match(uri.domain, cookie.domain)
      and path_match(uri.path, cookie.path)
      -- filter out expired cookies
      and (not cookie.expires or cookie.expires < now)    
  end
  
  return _.filter(self.cookies, relevant)
end

function CookieJar:serialize(url)
  local cookies = self:get_for_url(url)
  local result = {}
  
  if #cookies > 0 then    
    local function join(cookie)
      return cookie.name..'='..cookie.value
    end
    result = _.map(cookies, join)
  end
  
  return table.concat(result, '; ')
end

local exports = {}

exports.cookie = Cookie
exports.jar = CookieJar

return exports
